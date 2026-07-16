# -*- coding: utf-8 -*-
"""M2+ 시나리오 셀프체크 — 다중 로봇 충돌 0 + 교착 감지·해소 assert. 서버 불필요.
실행: py scenario_selfcheck.py"""
import hashlib
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import gridmap as M
import sim
import coordinator as C

# 골든-궤적 baseline(그린 코드에서 동결) — 코어 리팩터가 궤적을 바꾸는지 감지.
# ticks==ticks2는 tick 총수만 비교라 궤적 드리프트(같은 총tick, 다른 경로)를 못 잡음 → frame-단위 지문으로 보완.
GOLDEN = {
    "crossing": "830e0b604499c7c7430fd1eb8df4b3bb22445c316ed19a2bd01c6e6c2cca7b63",
    "deadlock": "f228d7fc32cdf3c6c7e2958b53e5e5ec2ec64cc714e347218565c96ace2a7f63",
    "scale": "7709bf2092528c2264094678c52b026330d941baadd5d812662cf05f16aa2084",       # 유휴좀비 봉합 재동결: 90/90 유지·tick 270→245
    "continuous": "c65277a2b8551f2642011279bfcaed94479adb4da2096b5d36f92c010dbdf48c",   # 유휴좀비 봉합 재동결: 배송107 유지
    "gridlock": "906936df3f43c064b197de151e5ebab19150436bdb39c40b309074345779716a",     # 시간캡(M1) 재동결: 완료63→72·오탐차단 10건 회복
}


def _hash_frames(frames):
    return hashlib.sha256(repr(frames).encode()).hexdigest()


def _frames_sim(world, max_ticks=80):
    """sim.step 루프 궤적 지문 — tick별 정렬 (id,x,y) 시퀀스."""
    fr = []
    for _ in range(max_ticks):
        world, telem, _ = sim.step(world)
        fr.append(tuple(sorted((t["robot_id"], t["metrics"]["x"], t["metrics"]["y"]) for t in telem)))
        if all(r.status in ("arrived", "down", "blocked") for r in world.robots):
            break
    return fr


def _frames_dynamic(world, **kw):
    """run_dynamic 궤적 지문 — on_tick으로 tick별 정렬 (id,x,y) 수집."""
    fr = []
    kw.pop("on_tick", None)
    C.run_dynamic(world, on_tick=lambda t, tl, wo, ts, lg:
                  fr.append(tuple(sorted((x["robot_id"], x["metrics"]["x"], x["metrics"]["y"]) for x in tl))), **kw)
    return fr


def _golden_hashes():
    """골든게이트 대상 시나리오(코어 _corridor_gates를 강하게 경유)의 궤적 지문 재계산."""
    o = {}
    wm = M.from_ascii(["........"] * 8)
    corners = [((0, 0), (7, 7)), ((7, 0), (0, 7)), ((0, 7), (7, 0)), ((7, 7), (0, 0))]
    o["crossing"] = _hash_frames(_frames_sim(sim.World(wmap=wm, robots=tuple(
        sim.plan_robot(wm, sim.Robot(id=f"r{i}", pos=s, goal=g, priority=i)) for i, (s, g) in enumerate(corners)))))
    wm2 = M.from_ascii(["#######", "#.....#", "###.###", "#######"])
    o["deadlock"] = _hash_frames(_frames_sim(sim.World(wmap=wm2, robots=(
        sim.plan_robot(wm2, sim.Robot(id="A", pos=(1, 1), goal=(5, 1), priority=0)),
        sim.plan_robot(wm2, sim.Robot(id="B", pos=(5, 1), goal=(1, 1), priority=1))))))
    w, dep = M.warehouse()
    cx = w.width // 2
    ai = [(cx, y) for y in range(w.height) if w.is_free((cx, y))]
    o["scale"] = _hash_frames(_frames_dynamic(
        sim.World(wmap=w, robots=tuple(sim.Robot(id=f"r{i}", pos=dep[i], goal=dep[i], priority=i) for i in range(40))),
        task_stream=C.gen_stream(w, dep, total=90, spawn_every=2), obstacle_events={35: ("close", ai), 95: ("open", ai)},
        faults={40: "r5", 70: "r12", 110: "r20", 150: "r8"}, max_ticks=1500))
    st = M.spread(w, 40)
    hm = {f"r{i}": st[i] for i in range(40)}
    pk, dp = M.stations(w)
    o["continuous"] = _hash_frames(_frames_dynamic(
        sim.World(wmap=w, robots=tuple(sim.Robot(id=f"r{i}", pos=st[i], goal=st[i], priority=i) for i in range(40))),
        spawn=C.package_spawner(pk, dp, every=3, per=1), homes=hm,
        obstacle_events={80: ("close", ai), 160: ("open", ai)}, faults={60: "r5", 140: "r12"}, max_ticks=400))
    segs = frozenset((c, y) for c in (7, 13, 19) for y in range(3, w.height - 3) if w.is_free((c, y)))
    o["gridlock"] = _hash_frames(_frames_dynamic(
        sim.World(wmap=w, robots=tuple(sim.Robot(id=f"r{i}", pos=dep[i], goal=dep[i], priority=i) for i in range(40))),
        task_stream=C.gen_stream(w, dep, total=90, spawn_every=2),
        obstacle_events={15: ("close", list(segs))}, max_ticks=2500))
    return o


def scenario_golden():
    """골든-궤적 회귀 게이트 — 코어 리팩터(M1 _corridor_state 추출 등) 후에도 궤적이 frame-단위 동일함을 assert."""
    cur = _golden_hashes()
    for name, h in GOLDEN.items():
        assert cur.get(name) == h, ("골든 궤적 드리프트", name, "expected", h, "got", cur.get(name))
    assert cur == _golden_hashes(), "골든 캡처 비결정론"   # 2회 동일
    print(f"  [골든게이트] {len(GOLDEN)}개 시나리오 궤적 frame-동일(드리프트 0) · 캡처 결정론 OK")


def _run_checked(world, max_ticks=400):
    """run하되 매 tick 위치 유일성(충돌 0) 검사. 반환: (final, ticks, events_all)."""
    assert len({r.pos for r in world.robots}) == len(world.robots), "초기 충돌"
    events_all, ticks = [], 0
    for _ in range(max_ticks):
        world, telem, events = sim.step(world)
        events_all += events
        cells = [(t["metrics"]["x"], t["metrics"]["y"]) for t in telem]
        assert len(cells) == len(set(cells)), ("충돌 발생", world.tick, cells)   # 충돌 0
        ticks = world.tick
        if all(r.status in ("arrived", "down", "blocked") for r in world.robots):
            break
    return world, ticks, events_all


def scenario_crossing():
    """4대 교차 이동 → 충돌 0 + 전원 도착 + 결정론."""
    wmap = M.from_ascii(["........"] * 8)
    corners = [((0, 0), (7, 7)), ((7, 0), (0, 7)), ((0, 7), (7, 0)), ((7, 7), (0, 0))]
    make = lambda: sim.World(wmap=wmap, robots=tuple(
        sim.plan_robot(wmap, sim.Robot(id=f"r{i}", pos=s, goal=g, priority=i))
        for i, (s, g) in enumerate(corners)))
    final, ticks, _ = _run_checked(make())
    assert all(r.pos == r.goal for r in final.robots), ("미도착", [(r.id, r.pos, r.goal) for r in final.robots])
    final2, ticks2, _ = _run_checked(make())
    assert ticks == ticks2, "비결정론"
    print(f"  [교차] 충돌 0 · 4대 전원 도착 · {ticks}tick · 결정론 OK")


def scenario_deadlock():
    """1-wide 통로 face-off + passing bay(3,2) → 교착 감지·해소 → 전원 도착."""
    wmap = M.from_ascii([
        "#######",
        "#.....#",   # 통로 (1,1)-(5,1)
        "###.###",   # bay (3,2)
        "#######",
    ])
    a = sim.plan_robot(wmap, sim.Robot(id="A", pos=(1, 1), goal=(5, 1), priority=0))
    b = sim.plan_robot(wmap, sim.Robot(id="B", pos=(5, 1), goal=(1, 1), priority=1))
    final, ticks, events = _run_checked(sim.World(wmap=wmap, robots=(a, b)))
    resolved = any(e.get("type") == "deadlock" and e.get("status") == "resolving" for e in events)
    assert resolved, "교착 해소 이벤트 없음(교착이 발생·해소됐어야)"
    assert all(r.pos == r.goal for r in final.robots), ("교착 후 미도착", [(r.id, r.pos) for r in final.robots])
    print(f"  [교착] 통로 face-off 감지·bay 양보 해소 · 전원 도착 · {ticks}tick")


def scenario_fms_normal():
    """정상 FMS: 2 로봇·2 작업 → 전원 완주(고장 없음)."""
    wmap = M.from_ascii(["........"] * 8)
    world = sim.World(wmap=wmap, robots=(
        sim.Robot("r0", (0, 0), (0, 0)), sim.Robot("r1", (7, 7), (7, 7))))
    tasks = [C.Task("t1", (2, 2), (6, 6)), C.Task("t2", (5, 1), (1, 5))]
    _, tasks, _, _ = C.run_fms(world, tasks)
    assert all(t.stage == "done" for t in tasks), ("미완 작업", [(t.id, t.stage) for t in tasks])
    print("  [FMS 정상] 2작업 전원 완주")


def scenario_fault_heal():
    """고장 자가치유: r0 작업 중 원인(alive=False) 주입 → coordinator가 down 파생 → 재배분 → r1 완주.
    개방 맵이라 고장 로봇이 재배분 경로의 유일통로를 봉쇄하지 않음(MAJOR-2 대칭 보장)."""
    wmap = M.from_ascii(["........"] * 8)
    world = sim.World(wmap=wmap, robots=(
        sim.Robot("r0", (0, 0), (0, 0)), sim.Robot("r1", (7, 0), (7, 0))))
    tasks = [C.Task("t1", (3, 3), (6, 6))]
    final, tasks, log, _ = C.run_fms(world, tasks, faults={3: "r0"})   # 원인만 주입(alive=False)
    derived = [e for e in log if e["type"] == "fault_derived" and e["robot"] == "r0"]
    assert derived, "coordinator가 r0 고장을 파생하지 않음(claim-heartbeat 미작동)"
    r0 = next(r for r in final.robots if r.id == "r0")
    assert r0.status == "down", ("r0 down 파생 안 됨", r0.status)   # 라벨은 coordinator가 붙임
    assert all(t.stage == "done" for t in tasks), ("재배분 후 미완", [(t.id, t.stage, t.robot) for t in tasks])
    print(f"  [FMS 고장치유] r0 원인주입→coordinator가 down 파생(tick {derived[0]['tick']})→재배분→완주")


def scenario_reroute():
    """통로 막힘→재경로: 정적 장애 로봇 B가 r0의 직선 경로를 막아 r0가 우회(개방 맵이라 대체 경로 존재)."""
    wmap = M.from_ascii(["........"] * 3)
    blocker = sim.Robot(id="B", pos=(4, 0), goal=(4, 0), status="down")   # (4,0) 영구 점유
    r0 = sim.plan_robot(wmap, sim.Robot(id="r0", pos=(0, 0), goal=(7, 0)))  # 초기 직선 계획이 (4,0) 관통
    final, ticks, _ = _run_checked(sim.World(wmap=wmap, robots=(r0, blocker)))
    r0f = next(r for r in final.robots if r.id == "r0")
    assert r0f.pos == (7, 0), ("재경로 후 미도착", r0f.pos)   # (4,0) 막혔으므로 도착=우회 증명
    print(f"  [재경로] 막힌 통로 우회 도착 · {ticks}tick")


def scenario_emergency():
    """긴급 임무(priority 0)가 일반 임무보다 먼저 배정된다(온라인 우선순위)."""
    wmap = M.from_ascii(["........"] * 4)
    world = sim.World(wmap=wmap, robots=(sim.Robot("r0", (0, 0), (0, 0)),))
    tasks = [C.Task("NORM", (1, 1), (2, 2), priority=5), C.Task("EMG", (6, 1), (7, 2), priority=0)]
    world = C.assign(world, tasks)
    assert world.robots[0].task == "EMG", ("긴급 우선 배정 실패", world.robots[0].task)
    print("  [긴급] 긴급 임무가 일반보다 먼저 배정")


def scenario_dynamic():
    """동적 세계 — 연속 임무 스트림 + 통로 실시간 폐쇄/개방 + 연쇄 고장 + 긴급 임무에 온라인 재조율.
    고정 스케줄이 아니라 매 tick 바뀌는 상황에 fleet이 반응(재할당·폐쇄우회·자가치유)."""
    wmap = M.from_ascii([
        "..............",
        ".##..##..##...",
        ".##..##..##...",
        "..............",
        ".##..##..##...",
        ".##..##..##...",
        "..............",
    ])
    robots = tuple(sim.Robot(id=f"r{i}", pos=p, goal=p, priority=i)
                   for i, p in enumerate([(0, 3), (4, 3), (9, 3), (13, 3)]))
    world = sim.World(wmap=wmap, robots=robots)
    stream = {                                   # 연속 임무(창고가 멈추지 않음)
        1: [C.Task("t1", (0, 0), (13, 6)), C.Task("t2", (13, 0), (0, 6))],
        10: [C.Task("t3", (0, 6), (13, 0))],
        18: [C.Task("EMG", (6, 0), (6, 6), priority=0)],   # 긴급
    }
    obst = {12: ("close", [(6, 3), (7, 3)]), 30: ("open", [(6, 3), (7, 3)])}   # 통로 폐쇄→개방
    faults = {15: "r1", 25: "r2"}                # 연쇄 고장
    final, tasks, log, metrics = C.run_dynamic(world, task_stream=stream, obstacle_events=obst, faults=faults)
    assert all(t.stage == "done" for t in tasks), ("미완 임무", [(t.id, t.stage) for t in tasks])
    assert len([e for e in log if e["type"] == "fault_derived"]) >= 2, "연쇄 고장 2건 파생 안 됨"
    assert any(e["type"] == "aisle_close" for e in log), "통로 폐쇄 이벤트 없음"
    assert next(t for t in tasks if t.id == "EMG").stage == "done", "긴급 임무 미완"
    print(f"  [동적] 임무 {metrics['spawned']}개(긴급 포함) 전원 완주 · 통로 폐쇄·개방 · "
          f"연쇄고장 {metrics['faults']}건 자가치유 · throughput {metrics['throughput']} · {metrics['ticks']}tick")


def scenario_scale():
    """현업 규모 — 38x27 창고·로봇 40대·임무 90건·연쇄 고장·통로 실시간 폐쇄.
    매 tick 충돌 0 + 영구 정지 없음(정상 종료)을 assert. 회복(towed 복귀)·정체 재배분·
    도달불가 차단으로 fleet이 어떤 상황에도 멈추지 않음을 실증(스크린샷 전원정지 버그 회귀 방지)."""
    w, depots = M.warehouse()
    robots = tuple(sim.Robot(id=f"r{i}", pos=depots[i], goal=depots[i], priority=i) for i in range(40))
    stream = C.gen_stream(w, depots, total=90, spawn_every=2)
    faults = {40: "r5", 70: "r12", 110: "r20", 150: "r8"}
    cx = w.width // 2
    aisle = [(cx, y) for y in range(w.height) if w.is_free((cx, y))]
    obst = {35: ("close", aisle), 95: ("open", aisle)}
    breaches = []

    def check(tick, telem, world, tasks, log):
        cells = [(t["metrics"]["x"], t["metrics"]["y"]) for t in telem]
        if len(cells) != len(set(cells)):
            breaches.append(tick)

    final, tasks, log, m = C.run_dynamic(sim.World(wmap=w, robots=robots), task_stream=stream,
                                         obstacle_events=obst, faults=faults, max_ticks=1500, on_tick=check)
    assert not breaches, ("충돌 발생 tick", breaches[:5])
    assert m["ticks"] < 1500, ("영구 정지(타임아웃) — 전원정지 회귀", m)      # 정상 종료 = 안 멈춤
    assert all(t.stage in ("done", "blocked") for t in tasks), "미종결 임무 존재"
    assert m["completed"] >= 80, ("완주율 저조", m["completed"])              # 90 중 대부분 완주
    assert len([e for e in log if e["type"] == "recovered"]) >= 1, "고장 회복 미작동"
    print(f"  [현업규모] 40대·90임무 · 충돌 0 · 완료 {m['completed']}/90 차단 {m['blocked']} · "
          f"회복 {len([e for e in log if e['type']=='recovered'])}건 · 처리량 {m['throughput']} · {m['ticks']}tick(정상종료)")


def scenario_continuous():
    """연속 운영 — 끝없는 물류 스폰(spawn) + 유휴 로봇 분산 복귀(homes). 한 번 끝나고 마는 게 아니라
    조기종료 없이 계속 배송. 태스크 리스트 유한 유지(pruning), 매 tick 정점+간선 충돌 0,
    적재(carrying=하역지로 운반) 상태 파생, 초기 분산 배치(시작부터 안 뭉침) 검증."""
    w, depots = M.warehouse()
    N = 40
    starts = M.spread(w, N)
    assert len(set(starts)) == N, "초기 분산 배치 중복"
    homes = {f"r{i}": starts[i] for i in range(N)}
    robots = tuple(sim.Robot(id=f"r{i}", pos=starts[i], goal=starts[i], priority=i) for i in range(N))
    pickups, dropoffs = M.stations(w)
    spawn = C.package_spawner(pickups, dropoffs, every=3, per=1)
    faults = {60: "r5", 140: "r12"}
    cx = w.width // 2
    aisle = [(cx, y) for y in range(w.height) if w.is_free((cx, y))]
    obst = {80: ("close", aisle), 160: ("open", aisle)}
    prev, vertex, edge, sizes, carry = {}, [], [], [], []

    def check(tick, telem, world, tasks, log):
        pos = {t["robot_id"]: (t["metrics"]["x"], t["metrics"]["y"]) for t in telem}
        cells = list(pos.values())
        if len(cells) != len(set(cells)):
            vertex.append(tick)                       # 정점 겹침(같은 칸)
        for a in pos:
            for b in pos:
                if a < b and prev.get(a) == pos.get(b) and prev.get(b) == pos.get(a) and prev.get(a) != pos.get(a):
                    edge.append(tick)                 # 간선 겹침(자리 맞바꿈=서로 통과)
        prev.clear()
        prev.update(pos)
        sizes.append(len(tasks))
        stage = {t.id: t.stage for t in tasks}
        carry.append(sum(1 for r in world.robots if r.task and stage.get(r.task) == "todropoff"))

    _, tasks, log, m = C.run_dynamic(sim.World(wmap=w, robots=robots), spawn=spawn, homes=homes,
                                     obstacle_events=obst, faults=faults, max_ticks=400, on_tick=check)
    assert m["ticks"] == 400, ("연속 모드가 조기종료됨", m["ticks"])       # 끝없이 운영(한 번에 안 끝남)
    assert not vertex and not edge, ("겹침 발생", vertex[:3], edge[:3])     # 정점+간선 충돌 0
    assert max(sizes) < 200, ("태스크 리스트 무한 성장(pruning 실패)", max(sizes))
    assert m["completed"] >= 50, ("배송 저조", m["completed"])
    assert max(carry) > 0, "적재(carrying) 로봇 파생 안 됨"
    print(f"  [연속물류] 끝없이 운영({m['ticks']}tick) · 배송 {m['completed']}건 · 최대 적재 {max(carry)}대 · "
          f"백로그 최대 {max(sizes)}(유한) · 정점·간선 충돌 0 · 분산 배치")


def scenario_gridlock():
    """1-wide 양방향 교착 완전 해소 — ① 정지(도착) 로봇이 대기 로봇의 유일 경로를 막으면 bay로 비켜섬(길 터주기)
    ② 1-wide 통로 정면충돌: 통로 방향 잠금(soft one-way) + 재경로로 해소 ③ 3중 반쪽-폐쇄서 '실제 교착 0'
    (차단은 물류지점 셀이 폐쇄돼 도달불가한 경우뿐 = 정답. 통로 게이트가 정면 교착 자체를 형성 안 시킴)."""
    # ① 정지 로봇 길막음: d가 목표(1,1) 도착 후 c의 유일 통로 봉쇄 → d가 bay로 비켜야 c 완주
    w = M.from_ascii(["..#....", ".......", "..#...."])
    rob = (sim.plan_robot(w, sim.Robot("a", (0, 1), (6, 1), priority=0)),
           sim.plan_robot(w, sim.Robot("b", (1, 1), (5, 1), priority=1)),
           sim.plan_robot(w, sim.Robot("c", (6, 1), (0, 1), priority=2)),
           sim.plan_robot(w, sim.Robot("d", (5, 1), (1, 1), priority=3)))
    final, ticks, ev = _run_checked(sim.World(wmap=w, robots=rob), max_ticks=100)
    assert all(r.pos == r.goal for r in final.robots), ("정지로봇 길막음 미해소", [(r.id, r.pos, r.goal) for r in final.robots])
    assert any(e.get("type") == "yield_idle" for e in ev), "유휴 양보(길 터주기) 이벤트 없음"
    # ② 1-wide 정면충돌: bay 있는 통로 양끝에서 마주봄 → 통로 게이트로 한쪽 대기·한쪽 통과
    w2 = M.from_ascii(["..#..#..", "........", "..#..#.."])
    face = (sim.plan_robot(w2, sim.Robot("x", (0, 1), (7, 1), priority=0)),
            sim.plan_robot(w2, sim.Robot("y", (7, 1), (0, 1), priority=1)))
    fin2, t2, _ = _run_checked(sim.World(wmap=w2, robots=face), max_ticks=60)
    assert all(r.pos == r.goal for r in fin2.robots), ("1-wide 정면충돌 미해소", [(r.id, r.pos) for r in fin2.robots])
    # ③ 3중 반쪽-아이슬 폐쇄 → 통로 게이트/재경로로 실제 교착 0 (차단은 물류지점이 폐쇄된 것뿐)
    w3, depots = M.warehouse()
    segs = frozenset((cx, y) for cx in (7, 13, 19) for y in range(3, w3.height - 3) if w3.is_free((cx, y)))
    robots = tuple(sim.Robot(id=f"r{i}", pos=depots[i], goal=depots[i], priority=i) for i in range(40))
    stream = C.gen_stream(w3, depots, total=90, spawn_every=2)
    _, tasks, log, m = C.run_dynamic(sim.World(wmap=w3, robots=robots), task_stream=stream,
                                     obstacle_events={15: ("close", list(segs))}, max_ticks=2500)
    blocked = [t for t in tasks if t.stage == "blocked"]
    import planner as _P
    reach = lambda a, b: _P.astar(w3, a, b, segs) is not None            # 폐쇄 맵에서 실제 도달 가능?
    true_gridlock = [t for t in blocked if reach(depots[0], t.pickup) and reach(t.pickup, t.dropoff)]
    assert not true_gridlock, ("실제 교착 발생(도달 가능한데 차단)", [t.id for t in true_gridlock])
    print(f"  [교착해소] 정지로봇 길막음({ticks}t·yield_idle) + 1-wide 정면충돌 통과({t2}t) + "
          f"3중 반쪽폐쇄서 실제 교착 0 (차단 {len(blocked)}건 전부 도달불가=정답)")


def scenario_aging():
    """기아 방지(aging) — 배송이 지연될수록 이동 우선순위↑, 양보 대상서 빠짐. 오래 기다린 로봇이 먼저 통과.
    aging 켬(기본 3) vs 끔(0)에서 '최장 배송 소요'가 줄어듦을 assert(굶는 배송 방지)."""
    w, depots = M.warehouse()
    N = 40
    starts = M.spread(w, N)
    homes = {f"r{i}": starts[i] for i in range(N)}
    pickups, dropoffs = M.stations(w)
    cx = w.width // 2
    ai = [(cx, y) for y in range(w.height) if w.is_free((cx, y))]

    def worst(aging):
        old = sim.AGING
        sim.AGING = aging
        try:
            robots = tuple(sim.Robot(id=f"r{i}", pos=starts[i], goal=starts[i], priority=i) for i in range(N))
            spawn = C.package_spawner(pickups, dropoffs, every=3, per=1)
            born, life = {}, []

            def cb(t, tl, wo, ts, lg):
                for x in ts:
                    born.setdefault(x.id, t)
                for x in ts:
                    if x.stage == "done":
                        life.append(t - born[x.id])   # 스폰→완료 소요
            C.run_dynamic(sim.World(wmap=w, robots=robots), spawn=spawn, homes=homes,
                          obstacle_events={80: ("close", ai), 160: ("open", ai)}, max_ticks=400, on_tick=cb)
            return max(life) if life else 0
        finally:
            sim.AGING = old

    off, on = worst(0), worst(3)
    assert on <= off * 0.95, ("aging 개선폭 미달(<5% — 사실상 무력화)", off, on)   # 보수적 하한(정확값 과적합 금지)
    print(f"  [기아방지] 최장 배송 소요 aging끔 {off}tick → 켬 {on}tick (지연된 배송 우선 이동)")


def scenario_oneway():
    """one-way 관측(corridor_locks) — 경합 통로에서 lock 방향이 허용 방향과 일치, 허용방향 로봇 전진·역방향 gate,
    wait_reason=corridor_gate, corridor_locks는 순수 읽기(step 결과 불변). 원인주입→파생→비자명 assert."""
    w = M.from_ascii(["..#..", ".....", "..#.."])   # (2,1)=단일칸 pinch(상하 벽)
    mk = lambda: (sim.plan_robot(w, sim.Robot("x", (1, 1), (4, 1), priority=0)),   # 서→동(높은 우선)
                  sim.plan_robot(w, sim.Robot("y", (3, 1), (0, 1), priority=1)))   # 동→서
    world = sim.World(wmap=w, robots=mk())
    locks = sim.corridor_locks(world)
    assert locks and [tuple(c) for c in locks[0]["cells"]] == [(2, 1)] and locks[0]["dir"] == [1, 0], \
        ("경합 lock 방향 틀림", locks)                                             # x(동쪽=+x) 우선 → dir=[1,0]
    a1, _, _ = sim.step(sim.World(wmap=w, robots=mk()))
    sim.corridor_locks(sim.World(wmap=w, robots=mk()))                            # 순수성: 호출이 step에 되먹임 없음
    a2, _, _ = sim.step(sim.World(wmap=w, robots=mk()))
    assert tuple(r.pos for r in a1.robots) == tuple(r.pos for r in a2.robots), "corridor_locks가 step에 되먹임"
    w2, tl, _ = sim.step(world)
    pos = {r.id: r.pos for r in w2.robots}
    assert pos["x"] == (2, 1), ("허용방향 로봇 미전진", pos["x"])                  # lock.dir 방향 로봇 전진
    assert pos["y"] == (3, 1), ("역방향 로봇 gate 안 됨", pos["y"])                # 반대방향 gated(대기)
    wr = {t["robot_id"]: t["metrics"]["wait_reason"] for t in tl}
    assert wr["y"] == "corridor_gate", ("wait_reason 원인 불일치", wr)
    fin, ticks, _ = _run_checked(sim.World(wmap=w, robots=mk()), max_ticks=40)
    assert all(r.pos == r.goal for r in fin.robots), ("one-way 후 미완주", [(r.id, r.pos) for r in fin.robots])
    print(f"  [one-way] 경합 lock dir 정확 + 허용방향 전진·역방향 gate + wait_reason=corridor_gate + 순수읽기 + 완주 {ticks}t")


def scenario_blocked_queue():
    """차단(개입) 큐 — stage=='blocked' 파생이 오탐 0(전부 물류지점 폐쇄=도달불가) + 되먹임 가드(관측이 tasks 불변).
    원인주입(물류지점 셀 폐쇄)→파생(blocked)→비자명 assert(열린 지점인데 차단 = FAIL)."""
    w, dep = M.warehouse()
    segs = frozenset((c, y) for c in (7, 13, 19) for y in range(3, w.height - 3) if w.is_free((c, y)))
    rb = tuple(sim.Robot(id=f"r{i}", pos=dep[i], goal=dep[i], priority=i) for i in range(40))
    _, tasks, log, m = C.run_dynamic(sim.World(wmap=w, robots=rb),
                                     task_stream=C.gen_stream(w, dep, total=90, spawn_every=2),
                                     obstacle_events={15: ("close", list(segs))}, max_ticks=2500)
    before = [(t.id, t.stage, t.robot) for t in tasks]                 # 되먹임 가드 baseline
    queue = [{"id": t.id, "pickup": t.pickup, "dropoff": t.dropoff}    # 관측 파생(read-only)
             for t in tasks if t.stage == "blocked"]
    after = [(t.id, t.stage, t.robot) for t in tasks]
    assert before == after, "관측 파생이 tasks를 변형(되먹임 금지 위반)"
    assert queue, "차단 큐 비어있음(도달불가 임무 주입됐는데)"
    import planner as _P
    reach = lambda a, b: _P.astar(w, a, b, segs) is not None            # 폐쇄 맵에서 실제 도달 가능?
    false_pos = [q["id"] for q in queue if reach(dep[0], q["pickup"]) and reach(q["pickup"], q["dropoff"])]
    assert not false_pos, ("차단 큐 오탐(도달 가능한데 차단)", false_pos)
    print(f"  [차단큐] 개입 큐 {len(queue)}건 전부 도달불가(물류지점 폐쇄)=오탐0 · 되먹임 가드(tasks 불변)")


def scenario_temp_closure_recovery():
    """차단 오탐 방지(시간 기반 캡) — 임시 폐쇄로 도달불가였던 태스크는 개방 후 회복(완주),
    영구 폐쇄는 CAP 초과 시 여전히 차단(터미널). 원인주입(폐쇄)→파생(회복/차단)→결과 기반 assert.
    회귀 근거: 횟수 기반(reassigns>MAX) 터미널은 개방 후 도달 가능해도 영구 차단하는 오탐(RED 재현됨)."""
    old = C.STUCK_TICKS
    C.STUCK_TICKS = 10                                     # 재배분 주기 압축(판별창 확대: 폐쇄 60t << CAP 150)
    try:
        w = M.from_ascii(["........"] * 8)
        mk = lambda: (sim.Robot("r0", (0, 0), (0, 0)), sim.Robot("r1", (7, 0), (7, 0)))
        # ① 임시 폐쇄(60t) → 개방 후 완주(오탐 차단 없음)
        _, tasks, log, _ = C.run_dynamic(sim.World(wmap=w, robots=mk()), tasks=[C.Task("T", (7, 7), (0, 7))],
                                         obstacle_events={5: ("close", [(7, 7)]), 65: ("open", [(7, 7)])},
                                         max_ticks=400)
        assert tasks[0].stage == "done", ("임시 폐쇄 후 미회복(오탐 차단)", tasks[0].stage)
        assert len([e for e in log if e["type"] == "task_reassign"]) >= 3, "재배분 리트라이 미작동"
        # ② 영구 폐쇄 → CAP(BLOCK_CAP_TICKS) 초과 시 터미널 차단 유지(진짜 도달불가는 정직하게 차단)
        _, tasks2, log2, _ = C.run_dynamic(sim.World(wmap=w, robots=mk()), tasks=[C.Task("U", (7, 7), (0, 7))],
                                           obstacle_events={5: ("close", [(7, 7)])}, max_ticks=400)
        assert tasks2[0].stage == "blocked", ("영구 도달불가인데 차단 안 됨", tasks2[0].stage)
        bt = next(e["tick"] for e in log2 if e["type"] == "task_blocked")
        assert bt >= C.BLOCK_CAP_TICKS, ("CAP 전에 조기 차단(시간 캡 미작동)", bt)
        print(f"  [차단오탐방지] 임시폐쇄(60t)→개방 후 완주(회복) · 영구폐쇄→tick{bt}(≥CAP {C.BLOCK_CAP_TICKS}) 정직 차단")
    finally:
        C.STUCK_TICKS = old


def scenario_varied_map():
    """데모 구성(varied=True) 스모크 — 다양화 맵의 건전성: 완전 연결·물류지점 충분·분산 배치 유일.
    데모(run.py)가 쓰는 분기가 자동검증 밖이던 사각(B-4) 봉합."""
    from collections import deque as _dq
    w, dep = M.warehouse(varied=True)
    free = [(x, y) for y in range(w.height) for x in range(w.width) if w.is_free((x, y))]
    seen, q = {free[0]}, _dq([free[0]])
    while q:
        x, y = q.popleft()
        for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if w.is_free(n) and n not in seen:
                seen.add(n)
                q.append(n)
    assert len(seen) == len(free), ("varied 맵 분단", len(seen), len(free))
    pk, dp = M.stations(w)
    assert len(pk) >= 14 and len(dp) > 0, ("물류지점 부족", len(pk), len(dp))
    st = M.spread(w, 40)
    assert len(set(st)) == 40, "분산 배치 중복"
    w2, _ = M.warehouse(varied=True)
    assert w.obstacles == w2.obstacles, "varied 맵 비결정론(seed 고정 위반)"
    print(f"  [varied맵] 완전연결({len(free)}셀) · 픽업 {len(pk)}·dock {len(dp)} · 분산 40 유일 · 시드 결정론")


def scenario_sla_zones():
    """SLA/구역 지표(관측 전용) 인과 검증 — ①폐쇄 주입군 p95 배송소요 > 무폐쇄 대조군(원인=혼잡)
    ②특정 구역(quadrant) 집중 스폰 시 그 구역 완료 수가 최대(부하가 원인에서 파생)
    + 되먹임 가드(지표 수집이 tasks 상태 무변형). 자명통과 아님: 지표가 주입 원인을 추적하는지 비교."""
    w, dep = M.warehouse()
    st = M.spread(w, 40)
    hm = {f"r{i}": st[i] for i in range(40)}
    pk, dp = M.stations(w)
    cx = w.width // 2
    ai = [(cx, y) for y in range(w.height) if w.is_free((cx, y))]

    def run(obst, spawn):
        rb = tuple(sim.Robot(id=f"r{i}", pos=st[i], goal=st[i], priority=i) for i in range(40))
        born, life, zone_done = {}, [], [0, 0, 0, 0]
        quad = lambda x, y: (0 if x < w.width // 2 else 1) + (0 if y < w.height // 2 else 2)

        def cb(t, tl, wo, ts, lg):
            snap = [(x.id, x.stage) for x in ts]                     # 되먹임 가드 스냅샷
            for x in ts:
                born.setdefault(x.id, t)
                if x.stage == "done":
                    life.append(t - born[x.id])
                    zone_done[quad(x.dropoff[0], x.dropoff[1])] += 1
            assert snap == [(x.id, x.stage) for x in ts], "지표 수집이 tasks를 변형(되먹임)"
        C.run_dynamic(sim.World(wmap=w, robots=rb), spawn=spawn, homes=hm,
                      obstacle_events=obst, max_ticks=400, on_tick=cb)
        life.sort()
        return (life[int(len(life) * 0.95)] if life else 0), zone_done

    base_spawn = lambda: C.package_spawner(pk, dp, every=3, per=1)
    p95_base, _ = run({}, base_spawn())                               # 무폐쇄 대조군
    p95_closed, _ = run({80: ("close", ai), 200: ("open", ai)}, base_spawn())   # 폐쇄 주입군
    assert p95_closed > p95_base, ("폐쇄 주입에도 p95 미상승(지표가 원인 추적 실패)", p95_base, p95_closed)
    q3 = [c for c in pk if c[0] >= w.width // 2] or pk                # 우측 픽업만
    d3 = [c for c in dp if c[1] >= w.height // 2] or dp               # 우하 dock만 → quadrant 3 집중
    _, zd = run({}, C.package_spawner(q3, d3, every=3, per=1))
    assert zd[3] == max(zd) and zd[3] > 0, ("집중 스폰 구역이 완료 최대 아님", zd)
    print(f"  [SLA·구역] 폐쇄 주입 p95 {p95_base}→{p95_closed}(상승=원인 추적) · 우하 집중 스폰 구역완료 {zd} · 되먹임 0")


def scenario_spawn_no_overlap():
    """물류 지점 겹침 방지(avoid_busy) — 활성 물류의 픽업·배송 셀에는 새 물류가 스폰되지 않음.
    대조: 기본(avoid_busy=False)은 중복 허용(기존 동작·골든 불변) vs True는 매 tick 중복 0."""
    w, dep = M.warehouse()
    st = M.spread(w, 40)
    hm = {f"r{i}": st[i] for i in range(40)}
    pk, dp = M.stations(w, n_pick=40)
    def run(avoid):
        rb = tuple(sim.Robot(id=f"r{i}", pos=st[i], goal=st[i], priority=i) for i in range(40))
        dups = [0]
        def cb(t, tl, wo, ts, lg):
            pts = [c for x in ts if x.stage not in ("done", "blocked") for c in (x.pickup, x.dropoff)]
            if len(pts) != len(set(pts)):
                dups[0] += 1
        C.run_dynamic(sim.World(wmap=w, robots=rb), homes=hm, max_ticks=200, on_tick=cb,
                      spawn=C.package_spawner(pk, dp, every=2, per=2, avoid_busy=avoid))
        return dups[0]
    d_on, d_off = run(True), run(False)
    assert d_on == 0, ("avoid_busy=True인데 물류 지점 중복 발생", d_on)
    assert d_off > 0, "대조군(기본)서 중복이 없어 검증이 자명"     # 원인(avoid) → 파생(중복 0) 비자명 확인
    print(f"  [물류겹침방지] avoid_busy 켬: 중복 0 / 끔(대조군): 중복 {d_off}tick — 스포너가 활성 지점 회피")


def scenario_idle_fault():
    """유휴 좀비 봉합 — '임무 없는' 로봇에 고장 주입(alive=False)해도 침묵에서 down 파생·회복돼야.
    회귀 근거: 이전엔 detect_and_heal이 task None을 스킵해 유휴/충전行 로봇이 status=moving인 채
    영구 정지(대시보드에 고장 표시도 안 되는 좀비 — 실사용 스크린샷 12·21번 재현)."""
    w = M.from_ascii(["........"] * 8)
    rb = (sim.Robot("r0", (4, 4), (4, 4)), sim.Robot("r1", (0, 0), (0, 0)))
    spawn = C.package_spawner([(6, 6)], [(1, 1)], every=100, per=1)   # r0는 유휴로 남게
    _, _, log, _ = C.run_dynamic(sim.World(wmap=w, robots=rb), spawn=spawn, faults={10: "r0"}, max_ticks=120)
    ev = lambda t: [e for e in log if e["type"] == t and e.get("robot") == "r0"]
    assert ev("fault_derived"), "유휴 로봇 고장이 down 파생 안 됨(좀비 회귀)"
    assert ev("recovered"), "유휴 로봇 고장이 회복 안 됨(영구 정지 회귀)"
    print(f"  [유휴좀비] 무임무 로봇 고장 → down 파생 t{ev('fault_derived')[0]['tick']} → 회복 t{ev('recovered')[0]['tick']} (영구 정지 봉합)")


def scenario_battery():
    """배터리+견인 로봇(opt-in) — ①LOW 로봇 충전소行→완충→복귀 ②방전=에러 송신(battery_dead)+정지(down)
    ③견인 로봇이 출동→탑재(월드에서 들어올림=충돌0 무충돌)→충전소 하역→0%부터 충전→완충, tow 주둔 복귀
    ④battery 미지정 경로는 기존과 동일(골든 보호). 원인주입(잔량·drain)→시스템 파생→결과 assert."""
    w = M.from_ascii(["........"] * 8)
    chargers = [(0, y) for y in range(0, 6)]
    home = (0, 7)                                          # 충전지역 맨 왼쪽 하단 주둔지
    rb = (sim.Robot("r0", (7, 7), (7, 7)), sim.Robot("r1", (3, 1), (3, 1)), sim.Robot("tow", home, home))
    B = {"chargers": chargers, "init": {"r0": 2, "r1": 90}, "drain_every": 1, "charge_per": 5,
         "tow": {"id": "tow", "home": home}}
    spawn = C.package_spawner([(6, 6)], [(1, 1)], every=60, per=1)
    seen_r0 = []
    _, _, log, _ = C.run_dynamic(sim.World(wmap=w, robots=rb), spawn=spawn, battery=B, max_ticks=200,
                                 on_tick=lambda t, tl, wo, ts, lg:
                                 seen_r0.append(any(x["robot_id"] == "r0" for x in tl)))
    ev = lambda t: [e for e in log if e["type"] == t]
    assert ev("battery_dead"), "방전 이벤트(에러 송신) 미발생"
    assert ev("tow_dispatch") and ev("tow_haul") and ev("tow_drop") and ev("tow_done"), "견인 사이클 미완"
    assert not all(seen_r0), "탑재 구간(월드에서 들어올림) 미관측"
    order = [ev(t)[0]["tick"] for t in ("battery_dead", "tow_dispatch", "tow_haul", "tow_drop")]
    assert order == sorted(order), ("견인 순서 이상", order)
    assert ev("charged"), "하역 후 충전 미완(0%→100 사이클)"
    assert ev("charge_go") and 0 < B["levels"]["r1"] <= 100, "일반 충전行/잔량 회계 이상"
    print(f"  [배터리·견인] 방전 t{order[0]}(에러)→tow 출동 t{order[1]}→탑재 t{order[2]}(운반 {sum(1 for s in seen_r0 if not s)}t)"
          f"→하역 t{order[3]}→완충 t{ev('charged')[0]['tick']} · tow 주둔 복귀")


def scenario_nav_trace():
    """자동주행 결정 트레이스(ASPIRE식) — sim.step의 주행 결정(재경로·양보·교착)을 버리지 않고 구조화 기록.
    원인주입(통로 폐쇄→혼잡)→파생(nav_reroute 등)→구조화 레코드 assert + 무혼잡 대조군 비교(혼잡이 원인임 실증)."""
    w, dep = M.warehouse()
    segs = frozenset((c, y) for c in (7, 13, 19) for y in range(3, w.height - 3) if w.is_free((c, y)))
    rb = tuple(sim.Robot(id=f"r{i}", pos=dep[i], goal=dep[i], priority=i) for i in range(40))
    _, _, log, _ = C.run_dynamic(sim.World(wmap=w, robots=rb),
                                 task_stream=C.gen_stream(w, dep, total=90, spawn_every=2),
                                 obstacle_events={15: ("close", list(segs))}, max_ticks=800)
    nav = [e for e in log if e["type"].startswith("nav_")]
    kinds = sorted(set(e["type"] for e in nav))
    assert "nav_reroute" in kinds, ("혼잡 주입인데 재경로 트레이스 없음", kinds)
    assert all("tick" in e and "robot" in e for e in nav), "트레이스 레코드 비구조화"
    rb2 = tuple(sim.Robot(id=f"r{i}", pos=dep[i], goal=dep[i], priority=i) for i in range(40))
    _, _, log2, _ = C.run_dynamic(sim.World(wmap=w, robots=rb2),
                                  task_stream=C.gen_stream(w, dep, total=90, spawn_every=2), max_ticks=800)
    closed = sum(1 for e in log if e["type"] == "nav_reroute")
    openrun = sum(1 for e in log2 if e["type"] == "nav_reroute")
    assert closed > openrun, ("재경로가 혼잡에서 파생됨을 실증 못함", closed, openrun)
    print(f"  [주행트레이스] nav 레코드 {len(nav)}건{kinds} 구조화 · 혼잡시 재경로 {closed} > 무혼잡 {openrun}(원인=혼잡 실증)")


def scenario_hysteresis():
    """방향 hysteresis(순수함수 smooth_locks) — aging이 통로 방향을 뒤집어 lock.dir이 volatile.
    단발 flip 억제 / 새 방향 hold연속 시 전파 / 새 lock 즉시 채택. 데이터계층 검증(육안·DOM 아님)."""
    A, B = (1, 0), (-1, 0)
    assert sim.smooth_locks([A, A, B, A, A], hold=3) == [A, A, A, A, A], "단발 flip 미억제"
    assert sim.smooth_locks([A, A, B, B, B, B], hold=3) == [A, A, A, A, B, B], "지속 flip 미전파"
    assert sim.smooth_locks([None, A, A], hold=3) == [None, A, A], "새 lock 즉시 채택 실패"
    print("  [hysteresis] 단발 flip 억제 · 지속 flip(hold=3) 전파 · 새 lock 즉시 채택")


def demo():
    scenario_golden()        # 골든-궤적 게이트(코어 리팩터 드리프트 감지)
    scenario_oneway()        # one-way 관측(corridor_locks·wait_reason)
    scenario_hysteresis()    # 방향 hysteresis(smooth_locks)
    scenario_blocked_queue() # 차단(개입) 큐(오탐0·되먹임 가드)
    scenario_temp_closure_recovery()  # 차단 오탐 방지(시간 캡: 임시폐쇄 회복·영구폐쇄 정직 차단)
    scenario_sla_zones()     # SLA/구역 지표(인과 2종·되먹임 가드)
    scenario_varied_map()    # 데모 varied 맵 스모크(B-4 사각 봉합)
    scenario_spawn_no_overlap()  # 물류 지점 겹침 방지(avoid_busy, 대조군 비교)
    scenario_idle_fault()    # 유휴 좀비 봉합(무임무 고장도 down 파생·회복)
    scenario_battery()       # 배터리 사이클(충전行·방전 에러·견인·완충, opt-in)
    scenario_nav_trace()     # 자동주행 결정 트레이스(ASPIRE식·원인→파생)
    scenario_crossing()      # (a) 정상 완주
    scenario_deadlock()      # (d) 교착→해소
    scenario_gridlock()      # 교착 심화(정지로봇 길막음·다중 우회 혼잡)
    scenario_aging()         # 기아 방지(지연된 배송 우선 이동)
    scenario_reroute()       # (c) 통로 막힘→재경로
    scenario_fms_normal()
    scenario_fault_heal()    # (b) 고장→자가치유
    scenario_emergency()     # 긴급 우선 배정
    scenario_dynamic()       # 동적 세계 통합(연속·폐쇄·연쇄고장·긴급)
    scenario_scale()         # 현업 규모(40대·90임무·충돌0·영구정지 없음)
    scenario_continuous()    # 연속 물류 운영(끝없음·적재/하역·분산·충돌0)
    print("시나리오 셀프체크 통과 — (a)정상 (b)고장자가치유 (c)재경로 (d)교착해소 + 교착심화·기아방지·긴급·동적세계 + 현업규모 + 연속물류 + 충돌0")


if __name__ == "__main__":
    demo()
