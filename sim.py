# -*- coding: utf-8 -*-
"""시뮬 코어 — 순수함수 step(). 입력 world 불변, 출력만 되돌린다.
M1 단일 로봇 → M2 다중 로봇 회피기동(우선순위+대기)+교착 감지·해소.
결정론: 같은 (world, 주입 이벤트) → 같은 결과(시드 재실행 대체). 순회는 전부 (우선순위,id) 정렬.

충돌 0 보장 규칙(보수적): 로봇은 목표 셀이 '이번 tick 현재 비어있고' + '이번 tick 예약 안 됐을 때만' 이동.
→ 정점·간선 충돌 원천 차단(비워지는 칸은 한 tick 뒤 진입 = 안전). 교착은 자연 발생 → _resolve_deadlocks가 해소."""
from dataclasses import dataclass, replace

import planner

DEADLOCK_TICKS = 8   # 무진행 백스톱(사이클 미탐지 livelock 방지). ponytail: 튜닝 knob
REROUTE_TICKS = 3    # 대기 이 tick 넘으면 혼잡(정적 로봇) 우회 재경로 — 다중 병목 분산. < DEADLOCK_TICKS라 양보보다 먼저
AGING = 3            # 대기 1 tick당 이동 우선순위 가산 — 배송이 지연될수록 먼저 이동하고 양보 대상서 빠짐(기아 방지)


def _eff_prio(r):
    """유효 이동 우선순위 = 기본 - AGING·대기tick. 낮을수록 먼저 이동/양보 안 함. 오래 막힐수록 낮아짐(오래 굶은 배송 우선)."""
    return r.priority - AGING * r.stuck_ticks


@dataclass(frozen=True)
class Robot:
    id: str
    pos: tuple
    goal: tuple
    path: tuple = ()          # 남은 경로(pos 포함)
    status: str = "idle"      # idle/moving/arrived/waiting/down/blocked
    priority: int = 0         # 낮을수록 우선(먼저 이동); tie-break=id
    stuck_ticks: int = 0      # 이동 의도했으나 막힌 연속 tick
    alive: bool = True        # False=고장(정적 장애물, claim 미제출) — 라벨 'down'은 coordinator가 파생
    task: str = None          # 배정된 작업 id

    @property
    def next_cell(self):
        return self.path[1] if len(self.path) >= 2 else None


@dataclass(frozen=True)
class World:
    wmap: object
    robots: tuple             # (Robot,...)
    tick: int = 0
    dyn_blocked: frozenset = frozenset()   # 동적 폐쇄 셀(통로 closure 등 — 런타임 장애물)


def plan_robot(wmap, r, extra_blocked=frozenset(), cost=None):
    """로봇 경로 (재)계획 → path 채운 Robot. 단일 replan() 사용. cost=혼잡 페널티(선택)."""
    path = planner.replan(wmap, r.pos, r.goal, extra_blocked, cost)
    if path is None:
        return replace(r, path=(), status="blocked")
    return replace(r, path=tuple(path), status="arrived" if len(path) <= 1 else "moving")


CONGEST_COST = 2     # 밀집 셀 1개 인접 로봇당 추가 이동비용 → 여럿 막힌 상태서 우회로 분산. ponytail: 튜닝 knob


def _congestion(robots, wmap):
    """로봇 밀집 비용맵 {cell: penalty} — 각 로봇의 셀+이웃에 가산. 재계획이 붐비는 곳을 피하게."""
    cong = {}
    for r in robots:
        for c in (r.pos, *wmap.neighbors(r.pos)):
            cong[c] = cong.get(c, 0) + CONGEST_COST
    return cong


def _find_cycle(waitfor):
    """functional 그래프(노드당 out-edge ≤1)에서 사이클 반환(정렬), 없으면 None."""
    for start in sorted(waitfor):
        seen, cur = [], start
        while cur in waitfor and cur not in seen:
            seen.append(cur)
            cur = waitfor[cur]
        if cur in seen:
            return sorted(seen[seen.index(cur):])
    return None


def down_cells(world):
    """고장/inert 로봇이 점유한 셀(정적 장애물처럼 우회 대상)."""
    return frozenset(r.pos for r in world.robots if r.status == "down" or not r.alive)


def blocked_all(world):
    """계획·이동이 피해야 할 런타임 셀 = 고장 로봇 + 동적 폐쇄(통로 closure)."""
    return down_cells(world) | world.dyn_blocked


def _ensure_paths(world):
    """활성 로봇이 항상 목표까지 경로를 갖게 보장(비거나 막힌 셀을 지나면 재계획) — 양보·고장·폐쇄 우회 자동.
    재계획은 혼잡 비용을 반영 → 폐쇄로 여럿이 재계획할 때 같은 우회로에 몰리지 않고 분산."""
    blocked = blocked_all(world)
    cong = _congestion(world.robots, world.wmap)
    out = []
    for r in world.robots:
        if r.status == "down" or r.status == "blocked" or not r.alive:
            out.append(r)
        elif r.pos == r.goal:
            out.append(replace(r, status="arrived"))
        elif len(r.path) < 2 or any(c in blocked for c in r.path):
            out.append(plan_robot(world.wmap, r, blocked, cong))   # 고장·폐쇄 우회 + 혼잡 분산 재계획
        else:
            out.append(r)
    return replace(world, robots=tuple(out))


def _corridor_state(world, desired):
    """1-wide 통로별 상태(공유 프리미티브) — run 셀·허용 방향(allow)·parties·drain 여부.
    _corridor_gates(게이팅)와 corridor_locks(관측)의 단일 진실원(발산 구현 2개 금지). world 변형 없음."""
    blk = blocked_all(world)

    def free(c):
        return world.wmap.is_free(c) and c not in blk

    def nbrs(c):
        return [n for n in world.wmap.neighbors(c) if free(n)]

    def narrow(c):
        ns = nbrs(c)
        return len(ns) == 2 and (ns[0][0] == c[0] == ns[1][0] or ns[0][1] == c[1] == ns[1][1])  # 이웃 2개 일직선

    states, done_runs = [], []
    for r in world.robots:
        d = desired.get(r.id)
        if not d or d == r.pos or narrow(r.pos) or not narrow(d):
            continue                           # 교차점→통로 '진입'만 처리(통로 안 로봇은 대상 아님)
        run, stack = set(), [d]                # d를 포함한 통로 run 수집(BFS)
        while stack:
            c = stack.pop()
            if c in run or not narrow(c):
                continue
            run.add(c)
            stack.extend(nbrs(c))
        if run in done_runs:
            continue
        done_runs.append(run)
        parties = []                           # (eff_prio, id, dir, 통로안?)
        for rr in world.robots:
            dd = desired.get(rr.id)
            if rr.pos in run:
                mv = dd if dd else rr.pos
                parties.append((_eff_prio(rr), rr.id, (mv[0] - rr.pos[0], mv[1] - rr.pos[1]), True))
            elif dd in run and not narrow(rr.pos):
                parties.append((_eff_prio(rr), rr.id, (dd[0] - rr.pos[0], dd[1] - rr.pos[1]), False))
        inside = [p for p in parties if p[3] and p[2] != (0, 0)]
        if not parties:
            continue
        if inside:                             # 통로 점유 중 → 방향 잠금(안 로봇 방향)
            allow = min(inside, key=lambda p: (p[0], p[1]))[2]
            best_in = min(p[0] for p in inside)
            draining = any(p[0] < best_in for p in parties   # 역방향 대기자가 더 오래 굶음 → 통로 비우기(drain)
                           if not p[3] and p[2] == (-allow[0], -allow[1]))
        else:                                  # 통로 비었음 → 진입자 최고 우선순위가 방향 결정
            allow = min(parties, key=lambda p: (p[0], p[1]))[2]
            draining = False
        states.append({"cells": run, "allow": allow, "parties": parties, "inside": bool(inside), "draining": draining})
    return states


def _corridor_gates(world, desired):
    """1-wide 통로 정면 교착 예방(soft one-way) — 역방향 진입 로봇을 통로 밖(교차점)에서 대기시킴.
    _corridor_state를 소비. 반환: 대기시킬 robot id 집합(추출 전과 바이트 동일)."""
    gated = set()
    for s in _corridor_state(world, desired):
        allow, draining = s["allow"], s["draining"]
        for eff, rid, mv, ins in s["parties"]:
            if s["inside"]:
                if ins:
                    continue                   # 안 로봇은 계속 진행(나가면 통로 비어감)
                if mv == (-allow[0], -allow[1]) or (draining and mv == allow):
                    gated.add(rid)             # 역방향 진입 대기 + drain 중엔 같은방향 신규진입도 대기
            elif mv == (-allow[0], -allow[1]):
                gated.add(rid)
    return gated


def corridor_locks(world):
    """관제 관측용 순수 헬퍼(읽기전용, world 변형 없음) — 현재 경합 중인 1-wide 통로별 (cells, dir).
    lock은 경합 시에만 존재(무경합 통로는 미포함). desired를 step()과 동일하게 next_cell에서 재구성."""
    desired = {}
    for r in world.robots:
        if r.status == "down" or not r.alive:
            desired[r.id] = r.pos
        else:
            nc = r.next_cell
            desired[r.id] = nc if nc is not None else r.pos
    return [{"cells": sorted(s["cells"]), "dir": list(s["allow"])} for s in _corridor_state(world, desired)]


def smooth_locks(dir_seq, hold=3):
    """통로 방향 시퀀스 hysteresis(순수·표시 전용 — step/결정론 무관). aging이 방향을 뒤집어 lock.dir이
    flip 근처서 volatile → 단발 flip은 억제, 새 방향이 hold tick 연속되면 전환(지속 flip 전파).
    새 lock/재등장(None→방향)은 즉시 채택(flip 아님). 반환: 입력과 동일 길이 평활 시퀀스."""
    out, cur, cand, cnt = [], None, None, 0
    for d in dir_seq:
        if d == cur:
            cand, cnt = None, 0
        elif cur is None:
            cur, cand, cnt = d, None, 0          # 새 lock/재등장은 즉시(flip 아님)
        else:
            cnt = cnt + 1 if d == cand else 1
            cand = d
            if cnt >= hold:
                cur, cand, cnt = d, None, 0       # 새 방향 hold연속 → 전환
        out.append(cur)
    return out


def step(world):
    """한 tick 진행 → (world', telemetry, events). 순수함수."""
    world = _ensure_paths(world)
    robots = list(world.robots)
    by_prio = sorted(robots, key=lambda r: (_eff_prio(r), r.id))   # 오래 기다린 로봇이 먼저 이동(기아 방지)

    def desired_of(r):
        if r.status == "down" or not r.alive:
            return r.pos                       # 고장/inert 로봇 = 정적 장애물(안 움직임·claim 없음)
        nc = r.next_cell
        return nc if nc is not None else r.pos

    occ_by = {r.pos: r.id for r in robots}     # 현재 셀 점유자
    desired = {r.id: desired_of(r) for r in robots}
    gated = _corridor_gates(world, desired)    # 1-wide 통로 정면 교착 예방(역방향 진입 대기)
    committed, reserved, wanted = {}, set(), {}
    for r in by_prio:                          # 우선순위 순 이동 커밋
        want = desired[r.id]
        if want != r.pos:
            wanted[r.id] = want              # 게이트돼도 '이동 의도'는 기록(대기 누적→aging이 통로 방향 뒤집음)
        d = r.pos if r.id in gated else want   # 통로 게이트: 역방향 진입은 대기
        can = False
        if d != r.pos and d not in reserved and d not in world.dyn_blocked:
            occ = occ_by.get(d)
            if occ is None:
                can = True                     # 빈 칸으로 이동
            elif occ in committed and committed[occ] != d and committed[occ] != r.pos:
                can = True                     # 앞 로봇이 비켜줌 → train 따라가기(스왑 아님, 충돌 0 유지)
        if can:
            committed[r.id] = d
            reserved.add(d)                    # r.pos는 예약 안 함 → 뒷 로봇이 따라 들어올 수 있게
        else:
            committed[r.id] = r.pos
            reserved.add(r.pos)

    new_robots = []
    for r in robots:
        if r.status == "down" or not r.alive:
            new_robots.append(r)
            continue
        newpos = committed[r.id]
        moved = newpos != r.pos
        if newpos == r.goal:
            status = "arrived"
        elif moved:
            status = "moving"
        else:
            status = "waiting" if r.id in wanted else r.status
        path = r.path[1:] if (moved and len(r.path) >= 2) else r.path
        stuck = 0 if moved else (r.stuck_ticks + 1 if r.id in wanted else 0)
        new_robots.append(replace(r, pos=newpos, path=path, status=status, stuck_ticks=stuck))

    w2 = replace(world, robots=tuple(new_robots), tick=world.tick + 1)
    w2, events = _resolve_deadlocks(w2)

    def _wait_reason(r):                        # 대기 원인 파생(관측 출력 전용, control flow 무변경)
        if r.status != "waiting":
            return "none"
        if r.id in gated:
            return "corridor_gate"             # 통로 방향잠금에 걸림
        want = wanted.get(r.id)
        if want is not None and want in world.dyn_blocked:
            return "dyn_blocked"               # 동적 폐쇄 셀 앞 대기
        return "vertex_contention"             # 목표 셀 점유·예약 경합

    telemetry = [
        {"robot_id": r.id,
         "metrics": {"x": r.pos[0], "y": r.pos[1], "status": r.status,
                     "goal_x": r.goal[0], "goal_y": r.goal[1], "stuck": r.stuck_ticks,
                     "claimed": bool(r.alive and r.status != "down"),  # 하트비트=살아서 발행 중(대기·정체도 발행). 이동 여부 아님
                     "wait_reason": _wait_reason(r), "task": r.task, "tick": w2.tick}}
        for r in w2.robots
    ]
    return w2, telemetry, events


def _resolve_deadlocks(world):
    """다중 로봇 그리드락 해소 — 길이 여럿으로 막힌 상태 대응.
    ① 혼잡 회피 재경로(여러 대 동시): 대기가 길어진 로봇을 '곧 안 비는 로봇'(정적 혼잡) 우회로 재계획 → 병목 분산.
    ② 남은 wait-for 사이클/무진행 백스톱: 수직 bay 양보(백오프+우선순위 하향, oscillation 방지)."""
    robots = list(world.robots)
    base = blocked_all(world)
    static_pos = {r.pos for r in robots if r.status in ("waiting", "down", "arrived", "blocked")}

    cong = _congestion(robots, world.wmap)      # 혼잡 비용맵(밀집 회피 → 분산)
    rerouted, events = {}, []                   # ① 혼잡 회피 재경로(다중 동시, 오래 기다린 로봇 먼저)
    for r in sorted(robots, key=lambda r: (_eff_prio(r), r.id)):
        if r.status != "waiting" or r.stuck_ticks < REROUTE_TICKS or r.pos == r.goal:
            continue
        alt = planner.replan(world.wmap, r.pos, r.goal, (base | static_pos) - {r.pos, r.goal}, cong)
        if alt and len(alt) >= 2 and tuple(alt) != r.path:   # 정적 혼잡 우회로 발견 → 분산
            rerouted[r.id] = replace(r, path=tuple(alt), status="moving", stuck_ticks=0)
            events.append({"robot_id": r.id, "type": "reroute", "status": "congestion"})
    if rerouted:
        robots = [rerouted.get(r.id, r) for r in robots]
        world = replace(world, robots=tuple(robots))

    id2r = {r.id: r for r in robots}
    pos2id = {r.pos: r.id for r in robots}
    occupied = set(pos2id)

    yielded = False                            # ②.5 정지된(유휴/도착) 로봇이 대기 로봇의 유일 경로를 막으면 비켜세움
    for r in sorted(robots, key=lambda x: (_eff_prio(x), x.id)):   # 오래 기다린 로봇의 길부터 터줌
        if r.status != "waiting" or r.id in rerouted or r.stuck_ticks < REROUTE_TICKS or r.pos == r.goal:
            continue
        d = r.next_cell
        bid = pos2id.get(d) if d is not None else None
        if bid is None or bid == r.id:
            continue
        b = id2r[bid]
        if b.task is not None or b.status not in ("arrived", "idle"):
            continue                           # 임무 수행 중 로봇은 안 건드림(자기 일 함)
        bay = [n for n in sorted(world.wmap.neighbors(b.pos))
               if n not in occupied and n != r.pos and n not in base]
        if bay:                                # 정지 로봇을 자유 bay로 옮겨 길 터줌(idle 재배치)
            nb = bay[0]
            id2r[bid] = replace(b, pos=nb, goal=nb, path=(), status="idle", stuck_ticks=0)
            occupied.discard(b.pos)
            occupied.add(nb)
            pos2id.pop(b.pos, None)
            pos2id[nb] = bid
            events.append({"robot_id": bid, "type": "yield_idle", "for": r.id})
            yielded = True
    if yielded:
        robots = [id2r[x.id] for x in robots]
        world = replace(world, robots=tuple(robots))

    waitfor = {}                               # ② 대기 로봇 → 자기 다음 칸 점유 로봇(사이클/백스톱 → bay 양보)
    for r in robots:
        if r.status != "waiting" or r.id in rerouted:
            continue
        d = r.next_cell
        occ = pos2id.get(d) if d is not None else None
        if occ is not None and occ != r.id:
            waitfor[r.id] = occ

    cycle = _find_cycle(waitfor)
    stuck_ids = sorted(r.id for r in robots
                       if r.status == "waiting" and r.id not in rerouted and r.stuck_ticks >= DEADLOCK_TICKS)
    if not cycle and not stuck_ids:
        return world, events
    pool = cycle if cycle else stuck_ids

    def side_and_free(rid):
        r = id2r[rid]
        free = [n for n in sorted(world.wmap.neighbors(r.pos)) if n not in occupied]
        blk = id2r[waitfor[rid]].pos if rid in waitfor else None
        if blk:
            vx, vy = r.pos
            dd = (blk[0] - vx, blk[1] - vy)
            retreat = (vx - dd[0], vy - dd[1])
            side = [n for n in free if n != retreat and n != blk]   # 수직 bay 선호(후퇴·전진 제외)
        else:
            side = free
        return side, free

    with_side = [rid for rid in pool if side_and_free(rid)[0]]     # 양보 가능 로봇 우선
    victim = max(with_side or pool, key=lambda rid: (_eff_prio(id2r[rid]), rid))   # 가장 안 기다린(freshest) 로봇이 양보 → 오래 굶은 배송 통과
    vr = id2r[victim]
    side, free = side_and_free(victim)
    if not free:                               # 탈출로 없음(막다른) → 시끄럽게 남김(tick-cap FAIL 노출)
        return world, events + [{"robot_id": victim, "type": "deadlock", "status": "stuck", "cycle": pool}]

    aside = side[0] if side else free[0]
    blocker_cell = id2r[waitfor[victim]].pos if victim in waitfor else None
    blocked = frozenset([blocker_cell]) if blocker_cell else frozenset()
    rest = planner.replan(world.wmap, aside, vr.goal, blocked)
    newpath = (vr.pos, aside) + (tuple(rest[1:]) if rest else ())
    # 양보 로봇은 우선순위를 최저로 낮춘다 → 상대가 통로를 먼저 통과, oscillation(재교착) 방지
    lowest = max(r.priority for r in robots) + 1
    new_vr = replace(vr, path=newpath, status="moving", stuck_ticks=0, priority=lowest)
    new_robots = tuple(new_vr if r.id == victim else r for r in robots)
    return replace(world, robots=new_robots), events + [{"robot_id": victim, "type": "deadlock", "status": "resolving", "cycle": pool}]


def run(world, max_ticks=400):
    """목표 도달까지(또는 max_ticks) 시뮬. 결정론. 반환: (최종 world, frames)."""
    frames = []
    for _ in range(max_ticks):
        world, telem, events = step(world)
        frames.append((world.tick, telem, events))
        if all(r.status in ("arrived", "down", "blocked") for r in world.robots):
            break
    return world, frames
