# -*- coding: utf-8 -*-
"""실험·평가 축 (관측 전용, 코어 무변경) — py experiments.py
①스케일 곡선: 로봇 수 N별 처리량·p95 배송·대기율 (충돌0·정상종료 유지 실증)
②정책 ablation: aging / one-way 통로잠금 / 혼잡비용 재경로를 각각 끄고 fleet 지표 비교
  → 각 메커니즘이 왜 존재하는지 숫자로 증명. 토글은 모듈 상수/함수 대체 후 원복(코어 파일 무변경).
전 실험 시드 고정 결정론(2회 동일 수치 assert는 demo()가 수행). 차트는 ../../docs/assets/fms/에 PNG."""
import sys
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import gridmap as M
import sim
import coordinator as C

ASSETS = Path(__file__).resolve().parent / "assets"          # 리포 로컬(README 렌더용)
DOC_ASSETS = Path(__file__).resolve().parents[2] / "docs" / "assets" / "fms"   # 랩 문서용(존재 시에만)
MAX_TICKS = 400


def _run(n_robots, collect_wait=False):
    """균일 창고에서 N대 연속 물류 1회 실행 → 지표. 매 tick 정점+간선 충돌 검사(불변식 실증)."""
    w, _ = M.warehouse()
    starts = M.spread(w, n_robots)
    homes = {f"r{i}": starts[i] for i in range(n_robots)}
    robots = tuple(sim.Robot(id=f"r{i}", pos=starts[i], goal=starts[i], priority=i) for i in range(n_robots))
    pk, dp = M.stations(w)
    spawn = C.package_spawner(pk, dp, every=3, per=1)
    prev, born, life, waits = {}, {}, [], []

    def cb(t, tl, wo, ts, lg):
        pos = {x["robot_id"]: (x["metrics"]["x"], x["metrics"]["y"]) for x in tl}
        cells = list(pos.values())
        assert len(cells) == len(set(cells)), ("정점 충돌", t)
        for a in pos:
            for b in pos:
                if a < b and prev.get(a) == pos.get(b) and prev.get(b) == pos.get(a) and prev.get(a) != pos.get(a):
                    raise AssertionError(("간선 충돌", t, a, b))
        prev.clear()
        prev.update(pos)
        for x in ts:
            born.setdefault(x.id, t)
            if x.stage == "done":
                life.append(t - born[x.id])
        if collect_wait:
            waits.append(sum(1 for x in tl if x["metrics"]["status"] == "waiting") / len(tl))

    _, _, log, m = C.run_dynamic(sim.World(wmap=w, robots=robots), spawn=spawn, homes=homes,
                                 max_ticks=MAX_TICKS, on_tick=cb)
    life.sort()
    return {"n": n_robots, "delivered": m["completed"], "throughput": m["completed"] / MAX_TICKS,
            "p95": life[int(len(life) * 0.95)] if life else 0,
            "max_delivery": life[-1] if life else 0, "blocked": m["blocked"],
            "wait_rate": round(sum(waits) / len(waits), 3) if waits else None}


def scale_curve(ns=(10, 20, 40, 70, 100)):
    """스케일 곡선 — N별 처리량·p95·대기율. 모든 N에서 충돌0(assert)·정상 완주 유지."""
    rows = [_run(n, collect_wait=True) for n in ns]
    for r in rows:
        print(f"  N={r['n']:>3}: 배송 {r['delivered']:>3} · 처리량 {r['throughput']:.3f}/t · "
              f"p95 {r['p95']:>3}t · 대기율 {r['wait_rate']:.1%}")
    return rows


ABL_KEYS = ("baseline", "no aging", "no one-way", "no congestion cost")   # 차트 라벨(영문 — 글꼴 호환)


def ablation():
    """정책 ablation — 기본(전부 켬) vs aging 끔 vs one-way 끔 vs 혼잡비용 끔. N=40 고정.
    토글은 모듈 상수/함수 대체 후 finally 원복(코어 파일 무변경, 실험 스코프 한정)."""
    results = {}
    results["baseline"] = _run(40)
    old_aging = sim.AGING
    try:
        sim.AGING = 0
        results["no aging"] = _run(40)
    finally:
        sim.AGING = old_aging
    old_gates = sim._corridor_gates
    try:
        sim._corridor_gates = lambda world, desired: set()   # one-way 통로잠금 무력화
        results["no one-way"] = _run(40)
    finally:
        sim._corridor_gates = old_gates
    old_cong = sim.CONGEST_COST
    try:
        sim.CONGEST_COST = 0                                 # 혼잡비용 재경로 무력화
        results["no congestion cost"] = _run(40)
    finally:
        sim.CONGEST_COST = old_cong
    for k, r in results.items():
        print(f"  {k:<18}: 배송 {r['delivered']:>3} · p95 {r['p95']:>3}t · 최장 {r['max_delivery']:>3}t · 차단 {r['blocked']}")
    return results


def oneway_stress():
    """one-way 실증 — 개방 창고 ablation에선 1-wide 경합이 드물어 차이가 안 보임(정직: 소형 정면충돌도
    bay 양보·재경로 계층이 해소함). 통로잠금의 실증 무대는 3중 반쪽-아이슬 폐쇄(1-wide 통로 다수 강제):
    켬/끔 완료율 비교로 '교착 예방'의 fleet 수준 가치를 측정."""
    w, dep = M.warehouse()
    segs = frozenset((c, y) for c in (7, 13, 19) for y in range(3, w.height - 3) if w.is_free((c, y)))

    def run_once():
        rb = tuple(sim.Robot(id=f"r{i}", pos=dep[i], goal=dep[i], priority=i) for i in range(40))
        _, _, _, m = C.run_dynamic(sim.World(wmap=w, robots=rb),
                                   task_stream=C.gen_stream(w, dep, total=90, spawn_every=2),
                                   obstacle_events={15: ("close", list(segs))}, max_ticks=2500)
        return m["completed"]

    on = run_once()
    old = sim._corridor_gates
    try:
        sim._corridor_gates = lambda world, desired: set()
        off = run_once()
    finally:
        sim._corridor_gates = old
    print(f"  3중 반쪽폐쇄 완료율: one-way 켬 {on}/90 · 끔 {off}/90 (Δ+{on - off})")
    return on, off


def charts(rows, ab):
    ASSETS.mkdir(parents=True, exist_ok=True)
    ns = [r["n"] for r in rows]
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.4))
    ax[0].plot(ns, [r["throughput"] for r in rows], "o-", color="#2da44e")
    ax[0].set_title("Throughput (deliveries/tick)")
    ax[1].plot(ns, [r["p95"] for r in rows], "o-", color="#cf222e")
    ax[1].set_title("p95 delivery time (ticks)")
    ax[2].plot(ns, [r["wait_rate"] for r in rows], "o-", color="#bf8700")
    ax[2].set_title("Mean waiting rate")
    for a in ax:
        a.set_xlabel("robots (N)")
        a.grid(alpha=0.3)
    fig.suptitle("Scale curve — collision-0 & deadlock-free maintained at every N")
    fig.tight_layout()
    fig.savefig(ASSETS / "scale_curve.png", dpi=110)
    plt.close(fig)

    keys = list(ab.keys())
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    ax[0].bar(keys, [ab[k]["delivered"] for k in keys], color=["#2da44e", "#bf8700", "#cf222e", "#8250df"])
    ax[0].set_title("Deliveries (400 ticks)")
    ax[1].bar(keys, [ab[k]["max_delivery"] for k in keys], color=["#2da44e", "#bf8700", "#cf222e", "#8250df"])
    ax[1].set_title("Worst-case delivery (ticks)")
    for a in ax:
        a.tick_params(axis="x", labelrotation=12)
        a.grid(alpha=0.3, axis="y")
    fig.suptitle("Policy ablation — why each mechanism exists (N=40)")
    fig.tight_layout()
    fig.savefig(ASSETS / "ablation.png", dpi=110)
    plt.close(fig)
    if DOC_ASSETS.parent.parent.exists():                    # physical-ai-lab 랩 문서에도 복사(분리 리포에선 스킵)
        DOC_ASSETS.mkdir(parents=True, exist_ok=True)
        import shutil
        for f in ("scale_curve.png", "ablation.png"):
            shutil.copy2(ASSETS / f, DOC_ASSETS / f)
    print(f"  차트 저장: {ASSETS / 'scale_curve.png'} · {ASSETS / 'ablation.png'}")


def demo():
    print("[스케일 곡선] N=10~100 (매 tick 충돌0 assert)")
    rows = scale_curve()
    assert all(r["delivered"] > 0 for r in rows), "스케일서 배송 0"
    print("[정책 ablation] N=40")
    ab = ablation()
    assert ab["baseline"]["max_delivery"] <= ab["no aging"]["max_delivery"], "aging이 최악 배송을 못 줄임"
    assert ab["baseline"]["max_delivery"] <= ab["no congestion cost"]["max_delivery"], "혼잡비용이 최악 배송을 못 줄임"
    print("[one-way 실증] 3중 반쪽폐쇄 스트레스(1-wide 통로 다수)")
    ow_on, ow_off = oneway_stress()
    assert ow_on > ow_off, ("one-way가 폐쇄 스트레스서 완료율 개선 못 함", ow_on, ow_off)
    print("[결정론] 대표 구성(N=40) 2회 재실행 비교")
    a, b = _run(40), _run(40)
    assert a == b, ("실험 비결정론", a, b)
    print(f"  N=40 2회 동일: 배송 {a['delivered']} · p95 {a['p95']}")
    charts(rows, ab)
    print("실험 완료 — 스케일·ablation·결정론·차트")


if __name__ == "__main__":
    demo()
