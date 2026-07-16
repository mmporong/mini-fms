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

# 배터리(opt-in, run_dynamic(battery=...)) — 현업 AMR 충전 정책:
#   LOW 이하 = 새 임무 할당 제외, 현재 임무는 완주 후 충전소행(opportunistic)
#   CRIT 이하 = 운반 중이어도 임무 반납(재개방) 후 즉시 충전소행(critical 예외)
#   방전(0%) = 정지(정적 장애물, 기존 고장 파이프라인 재사용: inert→down 파생→towed 견인)
#   견인 회복 시 현장 배터리 교체 가정으로 응급 잔량 부여 → LOW라 자연히 충전소행
BATT_LOW = 25
BATT_CRIT = 10
BATT_EMERGENCY = 20   # (tow 미사용 시) 방전 제자리 회복 응급 잔량
# battery["tow"]={"id":..,"home":cell} 지정 시 진짜 견인 로봇 모드:
#   주둔(home, 충전지역 좌하단) → 방전 발생 시 출동(dispatch) → 인접 도달 시 탑재(haul, 월드에서 들어올림
#   = 충돌0 불변과 무충돌) → 빈 충전소에 내려놓음(0%부터 충전 시작) → 주둔지 복귀. 제자리 회복은 비활성.


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


def assign(world, tasks, exclude=frozenset()):
    """열린 작업을 유휴 로봇에 greedy 배정(픽업에 가까운 로봇, 동거리 tie=id). exclude=할당 제외(저배터리·충전 중)."""
    robots = {r.id: r for r in world.robots}
    blocked = sim.blocked_all(world)
    idle = sorted([r for r in world.robots
                   if r.alive and r.status != "down" and r.task is None and r.id not in exclude],
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
                faults=None, max_ticks=800, on_tick=None, spawn=None, homes=None, battery=None):
    """동적 FMS — 연속 임무 스트림·동적 통로 폐쇄/개방·연쇄 고장·긴급 임무에 온라인 재조율.
    고정 스케줄이 아니라 매 tick 바뀌는 세상에 fleet이 반응(재할당·재경로·자가치유).
    task_stream={tick:[Task,...]} · obstacle_events={tick:('close'|'open',[cells])} · faults={tick:robot_id}.
    spawn(tick)->[Task]: 지정 시 끝없이 물류 발생(연속 운영, 조기종료 없음, 완료 태스크 pruning).
    homes={robot_id:cell}: 지정 시 유휴 로봇을 제 staging 셀로 복귀(분산 — 통로 뭉침 방지).
    battery(opt-in)={"chargers":[cells], "seed":int, "drain_every":tick, "charge_per":%/tick, ...}:
      지정 시 배터리 시뮬 — 랜덤 초기잔량, 주기 방전, LOW/CRIT 충전 정책, 0%=정지(고장 파이프라인 재사용).
      levels/charging은 이 dict 안에 채워져 호출측(관제)이 참조 공유로 읽음. 기본 None=기존 동작 불변(골든 보호).
    on_tick(tick, telem, world, tasks, log) 콜백. 반환: (world, tasks, log, metrics)."""
    from dataclasses import replace as _replace
    tasks = list(tasks or [])
    task_stream, obstacle_events, faults, homes = task_stream or {}, obstacle_events or {}, faults or {}, homes or {}
    if battery:
        import random as _rnd
        _brng = _rnd.Random(battery.get("seed", 7))
        batt = battery.setdefault("levels", {})
        for r in world.robots:                        # 랜덤 초기 잔량(시드 결정론) — 일부는 시작부터 저배터리
            batt.setdefault(r.id, battery.get("init", {}).get(r.id, _brng.randint(15, 100)))
        charging = battery.setdefault("charging", {})  # rid → 예약 충전소 셀
        chargers = list(battery["chargers"])
        drain_every = battery.get("drain_every", 42)   # 1×속도(~8.3tick/s) 기준 5초 ≈ 42tick당 1%
        charge_per = battery.get("charge_per", 1.25)   # 완충 10초 ≈ 83tick에 0→100 = 1.2%/tick
        tow = battery.get("tow")                       # 견인 로봇 모드(선택)
        if tow:
            tow.setdefault("state", "idle")            # idle/dispatch/haul/return
            tow.setdefault("hauled", None)             # 탑재된 방전 로봇(Robot 객체)
            batt[tow["id"]] = 100                      # 견인 로봇은 배터리 회계 제외(항상 100)
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

        exclude = frozenset()
        if battery:
            robs0 = {r.id: r for r in world.robots}
            for r in world.robots:
                if not r.alive or r.status == "down" or (tow and r.id == tow["id"]):
                    continue                             # 견인 로봇은 배터리 회계·임무 대상 아님
                if charging.get(r.id) == r.pos:          # 충전소 도착 → 충전(완충 10초 상당)
                    batt[r.id] = min(100.0, batt[r.id] + charge_per)
                    if batt[r.id] >= 100:
                        batt[r.id] = 100.0
                        charging.pop(r.id, None)         # 완충 → 예약 반납, 일반 복귀(다음 assign 대상)
                        log.append({"tick": tick, "type": "charged", "robot": r.id})
                elif tick % drain_every == 0:            # 주기 방전(대기·이동 공통 베이스로드)
                    batt[r.id] -= 1
                if batt[r.id] <= 0 and r.alive:
                    # 방전 = 정지(정적 장애물). 통신두절과 달리 BMS가 원인을 직접 알므로 down을 즉시 부여
                    # (침묵 파생 불필요 — detect_and_heal은 태스크 보유 로봇만 봄) + 견인 타이머 시작.
                    batt[r.id] = 0
                    robs0[r.id] = _replace(r, alive=False, status="down", task=None, path=())
                    charging.pop(r.id, None)
                    if not tow:
                        down_since[r.id] = tick          # tow 미사용 시에만 제자리 회복(RECOVER_TICKS)
                    for t in tasks:
                        if t.robot == r.id and t.stage != "done":
                            t.stage, t.robot = "open", None   # 들고 있던 임무 재개방(재배분)
                    log.append({"tick": tick, "type": "battery_dead", "robot": r.id})
                elif batt[r.id] <= BATT_CRIT and r.task is not None:   # critical: 임무 반납 후 즉시 충전行
                    for t in tasks:
                        if t.robot == r.id and t.stage != "done":
                            t.stage, t.robot = "open", None
                    robs0[r.id] = _replace(r, task=None, status="idle", goal=r.pos, stuck_ticks=0)
                    log.append({"tick": tick, "type": "battery_return", "robot": r.id})
            world = _replace(world, robots=tuple(robs0[r.id] for r in world.robots))
            exclude = frozenset(rid for rid in batt      # LOW 이하·충전 중·견인 로봇 = 새 임무 할당 제외
                                if batt[rid] <= BATT_LOW or rid in charging
                                or (tow and rid == tow["id"]))

        world = assign(world, tasks, exclude)            # 온라인 재할당(긴급 우선, 저배터리 제외)
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
                if battery and batt.get(rid, 100) <= 0:   # 방전 견인 회복 = 현장 응급 배터리 교체 → LOW라 곧 충전소행
                    batt[rid] = BATT_EMERGENCY
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

        if battery:                                      # ④ 충전소 파견: 유휴 저배터리 → 빈 충전소 예약·이동
            blk = sim.blocked_all(world)
            occupied_ch = set(charging.values())
            dispatch = {r.id: r for r in world.robots}
            for r in sorted(world.robots, key=lambda x: batt.get(x.id, 100)):   # 잔량 낮은 로봇부터
                if (r.task is None and r.alive and r.status != "down"
                        and batt.get(r.id, 100) <= BATT_LOW and r.id not in charging):
                    free_ch = [c for c in chargers if c not in occupied_ch and c not in blk]
                    if not free_ch:
                        break                            # 충전소 만석 → 다음 tick 재시도
                    target = min(free_ch, key=lambda c: _man(r.pos, c))
                    charging[r.id] = target
                    occupied_ch.add(target)
                    dispatch[r.id] = sim.plan_robot(world.wmap, _replace(r, goal=target), blk)
                    log.append({"tick": tick, "type": "charge_go", "robot": r.id})
            world = _replace(world, robots=tuple(dispatch[r.id] for r in world.robots))

        if battery and tow:                              # ⑤ 견인 로봇 상태기계(주둔→출동→탑재→하역→복귀)
            robs_t = {r.id: r for r in world.robots}
            tw = robs_t.get(tow["id"])
            if tw is not None:
                blk = sim.blocked_all(world)
                occupied_ch = set(charging.values())
                if tow["state"] == "idle":
                    dead = sorted([r for r in world.robots
                                   if r.status == "down" and batt.get(r.id, 100) <= 0], key=lambda r: r.id)
                    if dead:
                        tow["target"] = dead[0].id       # 출동(방전 셀은 막혀 있으니 인접 자유 셀로)
                        adj = [n for n in sorted(world.wmap.neighbors(dead[0].pos)) if n not in blk]
                        if adj:
                            tow["state"] = "dispatch"
                            robs_t[tw.id] = sim.plan_robot(world.wmap, _replace(tw, goal=adj[0]), blk)
                            log.append({"tick": tick, "type": "tow_dispatch", "robot": tow["target"]})
                elif tow["state"] == "dispatch":
                    victim = robs_t.get(tow.get("target"))
                    if victim is None or batt.get(tow.get("target"), 100) > 0:
                        tow["state"], tow["target"] = "return", None   # 대상 소실 → 복귀
                    elif abs(tw.pos[0] - victim.pos[0]) + abs(tw.pos[1] - victim.pos[1]) <= 1:
                        tow["hauled"] = victim           # 탑재: 방전 로봇을 월드에서 들어올림(충돌0 무충돌)
                        world = _replace(world, robots=tuple(r for r in world.robots if r.id != victim.id))
                        robs_t = {r.id: r for r in world.robots}
                        tw = robs_t[tow["id"]]
                        free_ch = [c for c in chargers if c not in occupied_ch and c not in blk]
                        target_ch = min(free_ch, key=lambda c: _man(tw.pos, c)) if free_ch else tow["home"]
                        tow["drop"] = target_ch
                        charging[victim.id] = target_ch  # 충전소 선예약(다른 로봇 파견 차단)
                        tow["state"] = "haul"
                        robs_t[tw.id] = sim.plan_robot(world.wmap, _replace(tw, goal=target_ch), blk)
                        log.append({"tick": tick, "type": "tow_haul", "robot": victim.id})
                elif tow["state"] == "haul":
                    if tw.pos == tow.get("drop"):        # 충전소 도착 → 복귀 시작(하역은 셀을 비운 다음 tick, return 분기)
                        tow["state"] = "return"
                        robs_t[tw.id] = sim.plan_robot(world.wmap, _replace(tw, goal=tow["home"]), blk)
                elif tow["state"] == "return":
                    if tow.get("hauled") is not None and tw.pos != tow.get("drop"):
                        v = tow["hauled"]                # 하역: tow가 충전소 셀을 비우면 방전 로봇을 그 자리에 배치
                        drop = tow["drop"]
                        if all(r.pos != drop for r in world.robots):
                            placed = _replace(v, pos=drop, goal=drop, path=(), status="idle",
                                              alive=True, task=None, stuck_ticks=0)
                            world = _replace(world, robots=world.robots + (placed,))
                            robs_t = {r.id: r for r in world.robots}
                            tw = robs_t[tow["id"]]
                            tow["hauled"] = None         # 0%부터 충전 시작(charging 예약 유지)
                            log.append({"tick": tick, "type": "tow_drop", "robot": v.id})
                    if tw.pos == tow["home"] and tow.get("hauled") is None:
                        tow["state"], tow["target"], tow["drop"] = "idle", None, None
                        log.append({"tick": tick, "type": "tow_done"})
                world = _replace(world, robots=tuple(robs_t[r.id] for r in world.robots if r.id in robs_t))

        if homes:                                        # ③ 유휴 로봇 staging 복귀(분산) — 충전行 로봇 제외
            blk = sim.blocked_all(world)
            parked = {r.id: r for r in world.robots}
            for r in world.robots:
                h = homes.get(r.id)
                if (r.task is None and r.alive and r.status != "down" and h and r.pos != h and r.goal != h
                        and not (battery and r.id in battery.get("charging", {}))
                        and not (battery and tow and r.id == tow["id"])):
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
