# -*- coding: utf-8 -*-
"""동적 FMS 시뮬 + 관제 서버 (현업 규모·연속 운영) — py run.py → http://127.0.0.1:8820
살아있는 창고: 38x27 격자, 로봇 40대가 끝없이 들어오는 물류를 rack(픽업)에서 적재→dock(배송)에서 하역.
통로 실시간 폐쇄/개방, 로봇 연쇄 고장→회복(towed), 긴급 물류 끼어들기에 매 tick 반응(재할당·우회·자가치유·
정체 재배분·도달불가 차단). 유휴 로봇은 staging으로 분산 복귀(뭉침 방지). 한 번 끝나지 않고 계속 운영.
대시보드가 2D 맵·적재 상태·집계지표·이벤트로 표시."""
import json
import sqlite3
import sys
import threading
import time
from collections import deque

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

sys.path.insert(0, ".")
import uvicorn
import gridmap as M
import sim
import coordinator as C
import app as app_module
import store   # 직접 삽입(HTTP 왕복 제거 → 속도 배율 실효)

HOST, PORT = "127.0.0.1", 8820
BASE = f"http://{HOST}:{PORT}"

W, DEPOTS = M.warehouse(varied=True)                        # 38x27 · 변화형(블록 크기·방향·빈공간 다양화, 시각용)
N = 40
CHARGERS = [c for c in DEPOTS if c[0] == 0]                 # 좌측 열(x=0) 한 줄 = 배터리 충전소(통로 밖이라 길 안 막음)
BATTERY = {"chargers": CHARGERS, "seed": 7,                 # 랜덤 초기 잔량(15~100%, 시드 결정론)
           "drain_every": 42,                               # 1×속도(~8.3tick/s) 기준 5초당 1% 방전
           "charge_per": 1.25}                              # 완충 10초 ≈ 83tick(0→100)
STARTS = M.spread(W, N, reserved=frozenset(CHARGERS))       # 분산 초기 배치(충전소와 안 겹치게)
HOMES = {f"r{i}": STARTS[i] for i in range(N)}              # 유휴 시 각자 staging 복귀 → 분산
PICKUPS, DROPOFFS = M.stations(W)                           # 물류 지점(rack 픽업·dock 배송)
SPAWN = C.package_spawner(PICKUPS, DROPOFFS, every=2, per=1)  # 끝없는 물류
_BR = __import__("random").Random(7)
BURST = {1: [C.Task(f"B{i}", _BR.choice(PICKUPS), _BR.choice(DROPOFFS)) for i in range(N + 8)]}  # 초기 일괄투입=전 로봇 즉시 가동
def _connected(w, blocked):                                                # 폐쇄 후 맵이 한 덩어리인지(분단 방지)
    from collections import deque
    free = [(x, y) for y in range(w.height) for x in range(w.width) if w.is_free((x, y)) and (x, y) not in blocked]
    if not free:
        return False
    seen, q = {free[0]}, deque([free[0]])
    while q:
        x, y = q.popleft()
        for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if w.is_free(n) and n not in blocked and n not in seen:
                seen.add(n)
                q.append(n)
    return len(seen) == len(free)


def _closures(w):                                                          # 다양한 폐쇄 후보 — 위치·방향(세로/가로) 다양, 각각 세그먼트라 연결성 유지
    cands = []
    for cx in range(5, w.width - 4, 3):                                    # 세로 세그먼트(여러 x)
        seg = [(cx, y) for y in range(3, w.height - 3) if w.is_free((cx, y))]
        if len(seg) >= 4 and _connected(w, set(seg)):
            cands.append(seg)
    for cy in range(4, w.height - 4, 3):                                   # 가로 세그먼트(여러 y)
        seg = [(x, cy) for x in range(3, w.width - 3) if w.is_free((x, cy))]
        if len(seg) >= 4 and _connected(w, set(seg)):
            cands.append(seg)
    return cands or [[(_CX, y) for y in range(3, w.height - 3) if w.is_free((_CX, y))]]


_CX = W.width // 2
_CANDS = _closures(W)                                                      # 연결성 유지되는 다양한 폐쇄들
__import__("random").Random(11).shuffle(_CANDS)                            # 시드 셔플 — 좌측(x=5부터 순차) 편중 방지, 위치·방향 골고루 순환
FAULTS = {t: f"r{(t // 90) % N}" for t in range(90, 10 ** 7, 90)}          # 90tick마다 한 대 고장(→회복)
OBST = {}
for _k in range(0, 200000, 200):                                          # 200tick 주기로 다른 위치·방향 폐쇄 순환
    _c = _CANDS[(_k // 200) % len(_CANDS)]
    OBST[_k + 120], OBST[_k + 240] = ("close", _c), ("open", _c)


def robots():
    return tuple(sim.Robot(id=f"r{i}", pos=STARTS[i], goal=STARTS[i], priority=i) for i in range(N))


_LOCK_HIST = {}    # run키(정렬 셀 튜플) → 최근 방향들 — hysteresis용
_BLOCKED_SEEN = {}  # 차단(개입 필요) 태스크 누적 — 연속 모드는 blocked를 pruning하므로 등장 시 보존
_NAV_TRACE = deque(maxlen=400)   # 자동주행 결정 트레이스 큐(ASPIRE식 flight recorder) — 재경로·양보·교착해소
_TASK_START = {}                 # robot_id → (task_id, 시작 tick) — 배송 경과시간(색=지연) 계산, 반납 시 리셋

# SLA/구역 지표(관측 전용 파생 — world/tasks 무변형)
SLA_TARGET = 75                  # 무폐쇄 baseline p95 실측(74tick, spawner every=3·40대) 기반. ponytail knob
_TASK_BORN = {}                  # task_id → 스폰 tick(배송소요 계산)
_DELIVERY = deque(maxlen=400)    # (소요 tick, dropoff 좌표) — 완료 배송 이력(유계)
_ZONE_OCC = [0, 0, 0, 0]         # quadrant별 로봇-tick 점유 누적(0=좌상,1=우상,2=좌하,3=우하)


def _quad(x, y):
    return (0 if x < W.width // 2 else 1) + (0 if y < W.height // 2 else 2)

# 핫루프 DB 발행 전용 영속 커넥션(store.insert는 건당 커넥션+commit → 병목). store.py 무변경, 같은 파일에 WAL로 배치 기록.
_DB = sqlite3.connect(str(store.DB_PATH), check_same_thread=False)
_DB.execute("CREATE TABLE IF NOT EXISTS telemetry (ts REAL NOT NULL, robot_id TEXT NOT NULL, metrics TEXT NOT NULL)")
_DB.execute("PRAGMA journal_mode=WAL")   # 리더(/recent)와 동시 접근 허용


def smoothed_oneway(world):
    """경합 통로별 방향을 hysteresis 평활해 반환(표시 안정화). 현재 경합 중인 run만 렌더(정직)."""
    raw = {tuple(map(tuple, lk["cells"])): tuple(lk["dir"]) for lk in sim.corridor_locks(world)}
    for key in set(_LOCK_HIST) | set(raw):
        h = _LOCK_HIST.setdefault(key, [])
        h.append(raw.get(key))                       # 이번 tick 방향(무경합=None)
        del h[:-6]                                    # 최근 6개만
    out = [{"cells": [list(c) for c in key], "dir": list(sim.smooth_locks(_LOCK_HIST[key])[-1] or d)}
           for key, d in raw.items()]
    for key in [k for k, h in _LOCK_HIST.items() if all(x is None for x in h[-4:])]:
        del _LOCK_HIST[key]                           # 오래 무경합 키 정리
    return out


def on_tick(tick, telem, world, tasks, log):
    stage = {t.id: t.stage for t in tasks}
    dropoff_of = {t.id: t.dropoff for t in tasks}                            # dock 대기 넘버링용(read-only)
    wr = {t["robot_id"]: t["metrics"].get("wait_reason", "none") for t in telem}
    for r in world.robots:                                                   # 레그 경과시간 추적(픽업하러/배송중 각각 리셋)
        if r.task is None:
            _TASK_START.pop(r.id, None)
        else:
            leg = (r.task, stage.get(r.task))                               # 임무+단계 = 레그(픽업하러 vs 배송중)
            if _TASK_START.get(r.id, (None,))[0] != leg:
                _TASK_START[r.id] = (leg, tick)                             # 새 임무 or 픽업함(단계 전환) → 흰색부터 리셋
    for t in tasks:                                                          # SLA: 스폰 tick 기록 + 완료 소요 수집(read-only)
        _TASK_BORN.setdefault(t.id, tick)
        if t.stage == "done":                                               # 이번 tick 완료분(직후 pruning되므로 여기서 latch)
            _DELIVERY.append((tick - _TASK_BORN.pop(t.id), t.dropoff))
        elif t.stage == "blocked":
            _TASK_BORN.pop(t.id, None)                                      # 차단 종결분 정리(유계)
    for r in world.robots:                                                   # 구역 점유(로봇-tick) 누적
        _ZONE_OCC[_quad(r.pos[0], r.pos[1])] += 1
    snap = [{"id": r.id, "x": r.pos[0], "y": r.pos[1], "status": r.status, "task": r.task,
             "pr": r.priority,                                                # 기본 이동 우선순위(번호와 함께 표시)
             "eff": r.priority - sim.AGING * r.stuck_ticks,                   # 유효 우선순위(대기 누적=aging 반영)
             "stuck": r.stuck_ticks,
             "age": tick - _TASK_START[r.id][1] if r.id in _TASK_START else 0,  # 배송 경과(색=지연, 반납 시 0)
             "carrying": bool(r.task and stage.get(r.task) == "todropoff"),   # 적재 중(하역지로 운반)
             "dest": (list(dropoff_of[r.task])                                # 배송 목적 dock(대기 넘버링용)
                      if r.task and stage.get(r.task) == "todropoff" else None),
             "batt": round(BATTERY["levels"].get(r.id, 100)),                 # 배터리 잔량(%)
             "charging": r.id in BATTERY.get("charging", {}),                 # 충전行/충전 중
             "wait_reason": wr.get(r.id, "none")}
            for r in world.robots]
    if tick % 2 == 0:                                       # DB 발행(드릴다운·파이프라인) — 배치 1커밋(2tick마다)
        now = time.time()
        _DB.executemany("INSERT INTO telemetry (ts, robot_id, metrics) VALUES (?, ?, ?)",
                        [(now, t["robot_id"], json.dumps(t["metrics"])) for t in telem])
        _DB.commit()
    # 카운터는 이번 tick 델타를 STATE에 누적 = 로그 경계(trim)에도 안전한 누적 집계
    prev = app_module.STATE.get("metrics", {})
    d_of = lambda typ: sum(1 for e in log if e.get("tick") == tick and e["type"] == typ)
    delivered = prev.get("delivered", 0) + sum(1 for t in tasks if t.stage == "done")
    faults = prev.get("faults", 0) + d_of("fault_derived")
    recovered = prev.get("recovered", 0) + d_of("recovered")
    blocked = prev.get("blocked", 0) + d_of("task_blocked")
    for t in tasks:                            # 차단 태스크 누적(pruning 전에 등장 시 보존) — tasks read-only
        if t.stage == "blocked" and t.id not in _BLOCKED_SEEN:
            _BLOCKED_SEEN[t.id] = {"id": t.id, "pickup": list(t.pickup), "dropoff": list(t.dropoff)}
            if len(_BLOCKED_SEEN) > 200:        # 형제 누산기처럼 경계(가장 오래된 것 제거)
                del _BLOCKED_SEEN[next(iter(_BLOCKED_SEEN))]
    for e in log:                              # 자동주행 결정 트레이스 큐(이번 tick 신규 nav 레코드)
        if e.get("tick") == tick and e["type"].startswith("nav_"):
            _NAV_TRACE.append({"tick": tick, "kind": e["type"][4:], "robot": e.get("robot"), "cause": e.get("status")})
    app_module.STATE = {
        "robots": snap,
        "dyn_blocked": [list(c) for c in world.dyn_blocked],
        "oneway": smoothed_oneway(world),      # 경합 통로 방향(hysteresis 평활 — 깜빡임 억제)
        "blocked_queue": list(_BLOCKED_SEEN.values())[-20:],   # 개입 필요 태스크(도달불가)
        "nav_trace": list(_NAV_TRACE),         # 주행 결정 전체(≤400) — /trace.jsonl flight recorder가 전량 서빙
        "sla": (lambda d: {"n": len(d), "p50": d[len(d) // 2], "p95": d[int(len(d) * 0.95)],
                           "target": SLA_TARGET,
                           "violation": round(sum(1 for v in d if v > SLA_TARGET) / len(d), 3)}
                if d else {"n": 0, "target": SLA_TARGET})(sorted(v for v, _ in _DELIVERY)),
        "zones": [{"q": i, "done": sum(1 for _, dp in _DELIVERY if _quad(dp[0], dp[1]) == i),
                   "occ": _ZONE_OCC[i]} for i in range(4)],
        "metrics": {"delivered": delivered, "active": len(tasks),
                    "carrying": sum(1 for s in snap if s["carrying"]),
                    "throughput": round(delivered / max(1, tick), 3),
                    "faults": faults, "recovered": recovered, "blocked": blocked, "ticks": tick,
                    "charging": sum(1 for s in snap if s["charging"]),
                    "batt_dead": prev.get("batt_dead", 0) + d_of("battery_dead"),
                    "batt_avg": round(sum(s["batt"] for s in snap) / max(1, len(snap)))},
        "events": [{"tick": e["tick"], "type": e["type"], "robot": e.get("robot"), "task": e.get("task")}
                   for e in log[-16:]],
    }
    time.sleep(0.12 / max(0.25, getattr(app_module, "SPEED", 1.0)))   # 속도 배율(기본 1배, 대시보드서 0.5~8× 조절)


if __name__ == "__main__":
    app_module.MAP = {"width": W.width, "height": W.height,
                      "obstacles": [list(c) for c in W.obstacles],
                      "pickups": [list(c) for c in PICKUPS], "dropoffs": [list(c) for c in DROPOFFS],
                      "chargers": [list(c) for c in CHARGERS]}
    threading.Thread(target=uvicorn.Server(
        uvicorn.Config(app_module.app, host=HOST, port=PORT, log_level="error")).run, daemon=True).start()
    time.sleep(1.5)
    print(f"연속 FMS 관제(40대·끝없는 물류·배터리) → {BASE}   (브라우저 열기, Ctrl+C 종료)")
    while True:                                             # 안전 재진입(정상적으론 한 세션이 계속 운영)
        C.run_dynamic(sim.World(wmap=W, robots=robots()), task_stream=BURST, spawn=SPAWN, homes=HOMES,
                      obstacle_events=OBST, faults=FAULTS, battery=BATTERY, max_ticks=10 ** 7, on_tick=on_tick)
