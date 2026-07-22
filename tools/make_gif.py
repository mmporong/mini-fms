# -*- coding: utf-8 -*-
"""README 히어로 GIF 렌더 — 40대 연속 물류를 headless로 돌려 애니메이션으로 굽는다.
코어(sim/coordinator/gridmap)는 읽기만 한다(코드 불변 원칙). run.py의 월드 구성을 재현하되
서버·배터리 없이 순수 시뮬만 구동, on_tick 콜백으로 프레임을 수집한다.

    python tools/make_gif.py          # → assets/demo.gif (약 9초, <4MB 목표)
"""
import sys

sys.path.insert(0, ".")
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import random

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

import coordinator as C
import gridmap as M
import sim

# ---- run.py와 동일한 월드 구성(배터리·견인 제외 — 시각 핵심만) ----
W, DEPOTS = M.warehouse(varied=True)
N = 40
STARTS = M.spread(W, N)
HOMES = {f"r{i}": STARTS[i] for i in range(N)}
PICKUPS, DROPOFFS = M.stations(W, n_pick=40)
SPAWN = C.package_spawner(PICKUPS, DROPOFFS, every=1, per=1, avoid_busy=True)
_BR = random.Random(7)
_bp = _BR.sample(PICKUPS, min(N, len(PICKUPS)))
_bd = _BR.sample(DROPOFFS, min(N, len(DROPOFFS)))
BURST = {1: [C.Task(f"B{i}", _bp[i % len(_bp)], _bd[i % len(_bd)]) for i in range(min(len(_bp), len(_bd)))]}

TICKS = 360          # 수집 구간
EVERY = 2            # 2tick당 1프레임 → 180프레임
FPS = 20

frames = []          # [(tick, [(x, y, loaded, down)...], delivered)]
_done_ids = set()    # 연속 모드는 완료 태스크를 pruning하므로 id 집합으로 누적 집계


def on_tick(tick, telem, world, tasks, log):
    _done_ids.update(t.id for t in tasks if t.stage == "done")
    if tick % EVERY:
        return
    snap = [(m["metrics"]["x"], m["metrics"]["y"],
             bool(m["metrics"].get("task")), m["metrics"]["status"] == "down")
            for m in telem if m["robot_id"] != "tow"]
    frames.append((tick, snap, len(_done_ids)))


robots = tuple(sim.Robot(id=f"r{i}", pos=STARTS[i], goal=STARTS[i], priority=i) for i in range(N))
C.run_dynamic(sim.World(wmap=W, robots=robots), task_stream=BURST, spawn=SPAWN,
              homes=HOMES, max_ticks=TICKS, on_tick=on_tick)
print(f"수집: {len(frames)}프레임 (tick {TICKS})")

# ---- 렌더 ----
walls = [(x, y) for y in range(W.height) for x in range(W.width) if not W.is_free((x, y))]
fig, ax = plt.subplots(figsize=(7.6, 5.4), dpi=60)
fig.patch.set_facecolor("#111418")
ax.set_facecolor("#111418")
ax.set_xlim(-0.7, W.width - 0.3)
ax.set_ylim(W.height - 0.3, -0.7)          # y 뒤집기(격자 좌표계)
ax.set_xticks([])
ax.set_yticks([])
for s in ax.spines.values():
    s.set_visible(False)

ax.scatter([x for x, _ in walls], [y for _, y in walls], marker="s", s=52, c="#2a3138", lw=0)
ax.scatter([x for x, _ in PICKUPS], [y for _, y in PICKUPS], marker="s", s=40, c="#8a6d1f", lw=0, alpha=.9)
ax.scatter([x for x, _ in DROPOFFS], [y for _, y in DROPOFFS], marker="s", s=40, c="#1f6d8a", lw=0, alpha=.9)

empty_sc = ax.scatter([], [], s=46, c="#9aa7b1", zorder=3)
load_sc = ax.scatter([], [], s=58, c="#57d977", zorder=4)
title = ax.set_title("", color="#dfe6ec", fontsize=11, loc="left", pad=8)


def draw(i):
    tick, snap, delivered = frames[i]
    ex = [(x, y) for x, y, loaded, down in snap if not loaded]
    lx = [(x, y) for x, y, loaded, down in snap if loaded]
    empty_sc.set_offsets(ex or [(-5, -5)])
    load_sc.set_offsets(lx or [(-5, -5)])
    title.set_text(f"mini FMS — 40 robots · endless logistics · collisions 0   "
                   f"(tick {tick}, delivered {delivered})")
    return empty_sc, load_sc, title


anim = FuncAnimation(fig, draw, frames=len(frames), blit=False)
out = "assets/demo.gif"
anim.save(out, writer=PillowWriter(fps=FPS))
import os

print(f"저장: {out} ({os.path.getsize(out) / 1e6:.1f} MB, {len(frames)}프레임 @ {FPS}fps)")
