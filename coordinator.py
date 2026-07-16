# -*- coding: utf-8 -*-
"""FMS 코디네이터 — 작업 할당(greedy) + 고장 자가치유(claim-heartbeat 파생).
통합 원리: 시나리오는 원인(로봇 alive=False)만 주입, coordinator가 침묵(claim 미제출)에서 down을 '파생',
selfcheck는 coordinator가 파생했음을 assert. 벽시계 아님 — 시뮬 tick 결정론(arm-twin recent-window 비계승)."""
from dataclasses import dataclass, replace

import sim

FAULT_TICKS = 12    # claim 미제출 이 tick 넘으면 고장 파생. > 최대 aging 대기(교착 해소 지연) 보장. ponytail knob
RECOVER_TICKS = 12  # 고장 로봇을 이 tick 뒤 제자리 idle로 회복(towed→복귀, 정적 봉쇄 해제). ponytail knob
STUCK_TICKS = 30    # 태스크 보유 로봇이 이 tick 목표거리 무개선(정지 or 진동)이면 태스크 재개방. > RECOVER_TICKS라 회복이 먼저
MAX_REASSIGN = 3    # churn 진단 임계(터미널 게이트 아님 — 터미널은 아래 시간 캡). >2배 누적 시 task_churn 로그
# 터미널 '차단(개입 필요)' 승격 = 연속 도달불가 지속 >= BLOCK_CAP_TICKS일 때만.
# ponytail knob — 관계식: 최대 정상 폐쇄 지속(run.py OBST close→open = 120tick) + 마진(재배분 1사이클 = STUCK_TICKS).
# 교차파일 결합 주의: run.py의 폐쇄 지속을 늘리면 이 값도 그걸 초과하도록 함께 갱신할 것(폐쇄 지속 < CAP 불변).
BLOCK_CAP_TICKS = 120 + STUCK_TICKS   # = 150


@dataclass
class Task:
    id: str
    pickup: tuple
    dropoff: tuple
    stage: str = "open"        # open/topickup/todropoff/done
    robot: str = None
    priority: int = 5          # 낮을수록 긴급(emergency=0). 할당 시 우선 처리


def _man(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def gen_stream(wmap, depots, total=90, spawn_every=2, seed=42):
    """결정론적 연속 임무 스트림 — 자유 셀(비-선반·비-depot)에서 픽업/배송 쌍. 일부 긴급(12개마다 1).
    seed 고정으로 재현 가능(random.Random). 반환: {tick:[Task,...]}."""
    import random
    rng = random.Random(seed)
    dep = set(depots)
    free = [(x, y) for y in range(wmap.height) for x in range(wmap.width)
            if wmap.is_free((x, y)) and (x, y) not in dep]
    stream = {}
    for i in range(total):
        p, d = rng.choice(free), rng.choice(free)
        while d == p:
            d = rng.choice(free)
        pr = 0 if i % 12 == 5 else 5
        stream.setdefault(1 + i * spawn_every, []).append(Task(f"T{i}", p, d, priority=pr))
    return stream


def package_spawner(pickups, dropoffs, every=2, per=1, seed=42):
    """끝없는 물류 발생기 — run_dynamic(spawn=...)에 넘길 spawn(tick) 클로저 반환.
    every tick마다 per개씩 픽업(rack)→배송(dock) 패키지 생성. 12개마다 1개 긴급. seed 고정=재현."""
    import random
    rng = random.Random(seed)
    ctr = [0]

    def spawn(tick):
        if tick % every:
            return []
        out = []
        for _ in range(per):
            i = ctr[0]
            ctr[0] += 1
            p, d = rng.choice(pickups), rng.choice(dropoffs)
            out.append(Task(f"P{i}", p, d, priority=0 if i % 12 == 5 else 5))
        return out
    return spawn


def assign(world, tasks):
    """열린 작업을 유휴 로봇에 greedy 배정(픽업에 가까운 로봇, 동거리 tie=id)."""
    robots = {r.id: r for r in world.robots}
    blocked = sim.blocked_all(world)
    idle = sorted([r for r in world.robots
                   if r.alive and r.status != "down" and r.task is None],   # 태스크 없으면 유휴(어디 있든 배정 가능)
                  key=lambda r: r.id)
    used = set()
    for t in sorted([t for t in tasks if t.stage == "open"], key=lambda t: (t.priority, t.id)):  # 긴급 우선
        avail = [r for r in idle if r.id not in used]
        if not avail:
            break
        best = min(avail, key=lambda r: (_man(r.pos, t.pickup), r.id))
        used.add(best.id)
        t.stage, t.robot = "topickup", best.id
        robots[best.id] = sim.plan_robot(world.wmap, replace(best, goal=t.pickup, task=t.id), blocked)
    return replace(world, robots=tuple(robots[r.id] for r in world.robots))


def advance(world, tasks):
    """로봇이 픽업/배송 도달 시 작업 단계 진행(2단 여정: 픽업 도달→배송 목표 재설정→배송 도달=완주)."""
    robots = {r.id: r for r in world.robots}
    blocked = sim.blocked_all(world)
    for t in tasks:
        if t.robot is None or t.stage == "done":
            continue
        r = robots.get(t.robot)
        if r is None or r.status == "down":
            continue
        if t.stage == "topickup" and r.pos == t.pickup:
            t.stage = "todropoff"
            robots[r.id] = sim.plan_robot(world.wmap, replace(r, goal=t.dropoff), blocked)
        elif t.stage == "todropoff" and r.pos == t.dropoff:
            t.stage = "done"
            robots[r.id] = replace(r, task=None, status="arrived")
    return replace(world, robots=tuple(robots[r.id] for r in world.robots))


def detect_and_heal(world, tasks, noclaim):
    """claim 미제출 streak≥FAULT_TICKS인 활성작업 로봇 → down 파생 + 작업 재개방(재배분). 반환: world', downed."""
    robots = {r.id: r for r in world.robots}
    downed = []
    for r in world.robots:
        if r.status == "down" or r.task is None:
            continue
        if noclaim.get(r.id, 0) >= FAULT_TICKS:
            robots[r.id] = replace(r, status="down")         # coordinator가 파생(주입한 라벨 아님)
            downed.append(r.id)
            for t in tasks:
                if t.robot == r.id and t.stage != "done":
                    t.stage, t.robot = "open", None
    return replace(world, robots=tuple(robots[r.id] for r in world.robots)), downed


def run_fms(world, tasks, faults=None, max_ticks=600, on_tick=None):
    """FMS 루프 — 할당·이동·작업진행·고장 파생/재배분. faults={tick: robot_id}=원인 주입(inert).
    on_tick(tick, telem, downed) 콜백(예: 발행+sleep). 반환: (final_world, tasks, log, frames)."""
    faults = faults or {}
    noclaim, log, frames = {}, [], []
    for tick in range(1, max_ticks + 1):
        if tick in faults:                          # 원인만 주입: inert(alive=False), status는 안 건드림
            rid = faults[tick]
            world = replace(world, robots=tuple(
                replace(r, alive=False) if r.id == rid else r for r in world.robots))
        world = assign(world, tasks)
        world, telem, events = sim.step(world)
        world = advance(world, tasks)
        for t in telem:                             # claim 추적(침묵 streak)
            rid = t["robot_id"]
            noclaim[rid] = 0 if t["metrics"]["claimed"] else noclaim.get(rid, 0) + 1
        world, downed = detect_and_heal(world, tasks, noclaim)
        for rid in downed:
            log.append({"tick": tick, "type": "fault_derived", "robot": rid})
            noclaim[rid] = 0
        frames.append((tick, telem, events + [{"type": "fault_derived", "robot": rid} for rid in downed]))
        if on_tick:
            on_tick(tick, telem, downed)
        if all(t.stage == "done" for t in tasks):
            break
    return world, tasks, log, frames


def run_dynamic(world, tasks=None, task_stream=None, obstacle_events=None,
                faults=None, max_ticks=800, on_tick=None, spawn=None, homes=None):
    """동적 FMS — 연속 임무 스트림·동적 통로 폐쇄/개방·연쇄 고장·긴급 임무에 온라인 재조율.
    고정 스케줄이 아니라 매 tick 바뀌는 세상에 fleet이 반응(재할당·재경로·자가치유).
    task_stream={tick:[Task,...]} · obstacle_events={tick:('close'|'open',[cells])} · faults={tick:robot_id}.
    spawn(tick)->[Task]: 지정 시 끝없이 물류 발생(연속 운영, 조기종료 없음, 완료 태스크 pruning).
    homes={robot_id:cell}: 지정 시 유휴 로봇을 제 staging 셀로 복귀(분산 — 통로 뭉침 방지).
    on_tick(tick, telem, world, tasks, log) 콜백. 반환: (world, tasks, log, metrics)."""
    from dataclasses import replace as _replace
    tasks = list(tasks or [])
    task_stream, obstacle_events, faults, homes = task_stream or {}, obstacle_events or {}, faults or {}, homes or {}
    noclaim, log = {}, []
    down_since, best_dist, goal_at, stuck, reassigns = {}, {}, {}, {}, {}   # 회복·정체 재배분·차단 추적
    unreach_since = {}   # (task_id, stage) → 연속 도달불가 시작 tick(시간 캡 터미널 판정용, 도달 가능 관측 시 리셋)
    delivered = blocked_total = 0                         # 연속 모드 누적 집계(완료 태스크는 pruning으로 제거)
    for tick in range(1, max_ticks + 1):
        if tick in task_stream:                          # 연속 임무 스폰
            for t in task_stream[tick]:
                tasks.append(t)
                log.append({"tick": tick, "type": "task_spawn", "task": t.id, "urgent": t.priority == 0})
        if spawn is not None:                            # 끝없는 물류 발생(연속 운영)
            for t in spawn(tick):
                tasks.append(t)
                log.append({"tick": tick, "type": "task_spawn", "task": t.id, "urgent": t.priority == 0})
        if tick in obstacle_events:                      # 통로 실시간 폐쇄/개방
            kind, cells = obstacle_events[tick]
            cset = frozenset(tuple(c) for c in cells)
            world = _replace(world, dyn_blocked=(world.dyn_blocked | cset) if kind == "close"
                             else (world.dyn_blocked - cset))
            log.append({"tick": tick, "type": "aisle_" + kind, "cells": len(cset)})
        if tick in faults:                               # 고장 원인만 주입(inert)
            rid = faults[tick]
            world = _replace(world, robots=tuple(
                _replace(r, alive=False) if r.id == rid else r for r in world.robots))

        world = assign(world, tasks)                     # 온라인 재할당(긴급 우선)
        world, telem, stepev = sim.step(world)           # 이동(폐쇄·고장 우회 자동)
        for e in stepev:                                 # 자동주행 결정 트레이스(ASPIRE식) — 재경로·양보·교착해소를 버리지 않고 기록
            log.append({"tick": tick, "type": "nav_" + e.get("type", "?"),
                        "robot": e.get("robot_id") or e.get("robot"), "status": e.get("status")})
        world = advance(world, tasks)

        for t in telem:
            rid = t["robot_id"]
            noclaim[rid] = 0 if t["metrics"]["claimed"] else noclaim.get(rid, 0) + 1
        world, downed = detect_and_heal(world, tasks, noclaim)   # 고장 파생·재배분
        for rid in downed:
            log.append({"tick": tick, "type": "fault_derived", "robot": rid})
            noclaim[rid] = 0
            down_since[rid] = tick

        # --- 감독(supervise): 영구 정지 방지 ---
        robs = {r.id: r for r in world.robots}
        for rid, since in list(down_since.items()):   # ① 회복: 고장 로봇을 제자리 idle로(정적 봉쇄 해제)
            if tick - since >= RECOVER_TICKS and robs[rid].status == "down":
                robs[rid] = _replace(robs[rid], status="idle", alive=True, task=None,
                                     goal=robs[rid].pos, path=(), stuck_ticks=0)
                down_since.pop(rid, None)
                log.append({"tick": tick, "type": "recovered", "robot": rid})
        for r in list(robs.values()):                 # ② 정체 태스크 재배분: 목표거리 무개선(정지 or 진동) 감지
            if r.task is None or r.status in ("down", "arrived") or not r.alive:
                stuck[r.id] = 0
                best_dist.pop(r.id, None)
                continue
            d = _man(r.pos, r.goal)
            if goal_at.get(r.id) != r.goal or r.id not in best_dist or d < best_dist[r.id]:
                best_dist[r.id], stuck[r.id] = d, 0     # 목표 바뀜/더 가까워짐 = 진행
            else:
                stuck[r.id] = stuck.get(r.id, 0) + 1    # 정지 or 진동 = 무진행
            goal_at[r.id] = r.goal
            if stuck[r.id] >= STUCK_TICKS:
                tid = r.task
                reassigns[tid] = reassigns.get(tid, 0) + 1   # 진단용 카운터(터미널 게이트 아님)
                # 터미널 '차단'은 시간 기반: 연속 도달불가 지속 >= BLOCK_CAP_TICKS일 때만(임시 폐쇄 오탐 방지).
                # 신호원 = 기존 plan_robot 결과 재사용(추가 astar 0회): 결정점에서 status=="blocked" 또는
                # path 없음 = 그 시점 도달불가(현재 goal 기준이라 레그별 판정이 자연 성립).
                task_obj = next((t for t in tasks if t.id == tid), None)
                leg = (tid, task_obj.stage if task_obj else "?")
                unreachable = (r.status == "blocked") or (len(r.path) < 2 and r.pos != r.goal)
                if unreachable:
                    if unreach_since.get(leg) is None:
                        unreach_since[leg] = tick             # 연속 도달불가 시작
                    newstage = "blocked" if tick - unreach_since[leg] >= BLOCK_CAP_TICKS else "open"
                else:
                    unreach_since.pop(leg, None)              # 도달 가능(plan 성공 = 혼잡일 뿐) → 리셋, 계속 재시도
                    newstage = "open"
                    if reassigns[tid] > 2 * MAX_REASSIGN:     # churn 안전판: 진단 로그만(관측 전용, 터미널화 금지)
                        log.append({"tick": tick, "type": "task_churn", "task": tid, "count": reassigns[tid]})
                for t in tasks:
                    if t.robot == r.id and t.stage != "done":
                        t.stage, t.robot = newstage, None
                robs[r.id] = _replace(r, task=None, status="idle", goal=r.pos, stuck_ticks=0)
                stuck[r.id] = 0
                best_dist.pop(r.id, None)
                log.append({"tick": tick, "type": "task_blocked" if newstage == "blocked" else "task_reassign",
                            "robot": r.id, "task": tid})
        world = _replace(world, robots=tuple(robs[r.id] for r in world.robots))

        if homes:                                        # ③ 유휴 로봇 staging 복귀(분산)
            blk = sim.blocked_all(world)
            parked = {r.id: r for r in world.robots}
            for r in world.robots:
                h = homes.get(r.id)
                if r.task is None and r.alive and r.status != "down" and h and r.pos != h and r.goal != h:
                    parked[r.id] = sim.plan_robot(world.wmap, _replace(r, goal=h), blk)
            world = _replace(world, robots=tuple(parked[r.id] for r in world.robots))

        if on_tick:
            on_tick(tick, telem, world, tasks, log)
        if spawn is not None and len(log) > 4000:        # 연속 모드 로그 경계(트레이스 무한 성장 방지)
            del log[:-2000]

        if spawn is not None:                            # 완료/차단 태스크 pruning(무한 성장 방지) + 누적 집계
            keep = []
            for t in tasks:
                if t.stage == "done":
                    delivered += 1
                elif t.stage == "blocked":
                    blocked_total += 1
                else:
                    keep.append(t)
                if t.stage in ("done", "blocked"):       # 종결 태스크의 도달불가 추적 정리(유계)
                    unreach_since.pop((t.id, "topickup"), None)
                    unreach_since.pop((t.id, "todropoff"), None)
            tasks = keep
        else:                                            # 스트림 모드: 전부 완결 시 조기종료(기존 동작)
            future = any(k > tick for k in task_stream)
            if not future and tasks and all(t.stage in ("done", "blocked") for t in tasks):
                break

    cont = spawn is not None
    done = delivered if cont else sum(1 for t in tasks if t.stage == "done")
    blk = blocked_total if cont else sum(1 for t in tasks if t.stage == "blocked")
    metrics = {"completed": done, "ticks": world.tick,
               "throughput": round(done / max(1, world.tick), 3),
               "faults": len([e for e in log if e["type"] == "fault_derived"]),
               "blocked": blk,
               "spawned": delivered + blocked_total + len(tasks) if cont else len(tasks)}
    return world, tasks, log, metrics
