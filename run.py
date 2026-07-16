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

W, DEPOTS = M.warehouse()                                   # 38x27 ≈1000셀, 소형 대비 10배
N = 40
STARTS = M.spread(W, N)                                     # 분산 초기 배치(시작부터 안 뭉침)
HOMES = {f"r{i}": STARTS[i] for i in range(N)}              # 유휴 시 각자 staging 복귀 → 분산
PICKUPS, DROPOFFS = M.stations(W)                           # 물류 지점(rack 픽업·dock 배송)
SPAWN = C.package_spawner(PICKUPS, DROPOFFS, every=2, per=1)  # 끝없는 물류
_BR = __import__("random").Random(7)
BURST = {1: [C.Task(f"B{i}", _BR.choice(PICKUPS), _BR.choice(DROPOFFS)) for i in range(N + 8)]}  # 초기 일괄투입=전 로봇 즉시 가동
_CX = W.width // 2
# 통로 세그먼트만 폐쇄(양끝 마진 개방) — 열 전체를 닫으면 맵이 분단돼 건너편 로봇이 못 움직임.
# 세그먼트면 연결 유지 → 로봇이 폐쇄 앞까지 와서 우회(사용자 지적 반영).
AISLE = [(_CX, y) for y in range(3, W.height - 3) if W.is_free((_CX, y))]
FAULTS = {t: f"r{(t // 90) % N}" for t in range(90, 10 ** 7, 90)}          # 90tick마다 한 대 고장(→회복)
OBST = {}
for _c in range(0, 10 ** 7, 300):                                          # 중앙 통로 주기적 폐쇄→개방
    OBST[_c + 120], OBST[_c + 220] = ("close", AISLE), ("open", AISLE)


def robots():
    return tuple(sim.Robot(id=f"r{i}", pos=STARTS[i], goal=STARTS[i], priority=i) for i in range(N))


_LOCK_HIST = {}    # run키(정렬 셀 튜플) → 최근 방향들 — hysteresis용
_BLOCKED_SEEN = {}  # 차단(개입 필요) 태스크 누적 — 연속 모드는 blocked를 pruning하므로 등장 시 보존
_NAV_TRACE = deque(maxlen=400)   # 자동주행 결정 트레이스 큐(ASPIRE식 flight recorder) — 재경로·양보·교착해소

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
    wr = {t["robot_id"]: t["metrics"].get("wait_reason", "none") for t in telem}
    snap = [{"id": r.id, "x": r.pos[0], "y": r.pos[1], "status": r.status, "task": r.task,
             "pr": r.priority,                                                # 기본 이동 우선순위(번호와 함께 표시)
             "eff": r.priority - sim.AGING * r.stuck_ticks,                   # 유효 우선순위(대기 누적=aging 반영)
             "stuck": r.stuck_ticks,
             "carrying": bool(r.task and stage.get(r.task) == "todropoff"),   # 적재 중(하역지로 운반)
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
        "nav_trace": list(_NAV_TRACE)[-30:],   # 최근 주행 결정(전체는 /trace.jsonl)
        "metrics": {"delivered": delivered, "active": len(tasks),
                    "carrying": sum(1 for s in snap if s["carrying"]),
                    "throughput": round(delivered / max(1, tick), 3),
                    "faults": faults, "recovered": recovered, "blocked": blocked, "ticks": tick},
        "events": [{"tick": e["tick"], "type": e["type"], "robot": e.get("robot"), "task": e.get("task")}
                   for e in log[-16:]],
    }
    time.sleep(0.12 / max(0.25, getattr(app_module, "SPEED", 1.0)))   # 속도 배율(기본 1배, 대시보드서 0.5~8× 조절)


if __name__ == "__main__":
    app_module.MAP = {"width": W.width, "height": W.height,
                      "obstacles": [list(c) for c in W.obstacles],
                      "pickups": [list(c) for c in PICKUPS], "dropoffs": [list(c) for c in DROPOFFS]}
    threading.Thread(target=uvicorn.Server(
        uvicorn.Config(app_module.app, host=HOST, port=PORT, log_level="error")).run, daemon=True).start()
    time.sleep(1.5)
    print(f"연속 FMS 관제(40대·끝없는 물류) → {BASE}   (브라우저 열기, Ctrl+C 종료)")
    while True:                                             # 안전 재진입(정상적으론 한 세션이 계속 운영)
        C.run_dynamic(sim.World(wmap=W, robots=robots()), task_stream=BURST, spawn=SPAWN, homes=HOMES,
                      obstacle_events=OBST, faults=FAULTS, max_ticks=10 ** 7, on_tick=on_tick)
