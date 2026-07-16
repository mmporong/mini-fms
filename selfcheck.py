# -*- coding: utf-8 -*-
"""M1 셀프체크 — A* 경로 유효성 + 순수함수 결정론 + 로봇 목표 도달 assert.
실행: py selfcheck.py  → "M1 셀프체크 통과" 또는 AssertionError."""
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")  # Windows cp949 콘솔 print 깨짐 방지

import gridmap as M
import planner
import sim

# 외곽 border 자유 → 연결성 보장. P=픽업, D=배송, #=선반.
ROWS = [
    "..........",
    ".#.####.#.",
    ".#.P..#.#.",
    ".#.##.#.#.",
    ".....#....",
    ".####.##.#",
    "......#..D",
]


def demo():
    wmap = M.from_ascii(ROWS)
    start, goal = (0, 0), wmap.dropoffs[0]

    # 1) A* 경로 유효성
    path = planner.astar(wmap, start, goal)
    assert path is not None, "경로 없음(맵 비연결?)"
    assert path[0] == start and path[-1] == goal, ("끝점 불일치", path[0], path[-1])
    for c in path:
        assert wmap.is_free(c), ("장애물 통과", c)
    for a, b in zip(path, path[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1, ("비인접 이동", a, b)

    # 2) 결정론: 같은 입력 → 같은 경로
    assert planner.astar(wmap, start, goal) == path, "A* 비결정론"

    # 3) 시뮬: 로봇이 경로 따라 목표 도달 + step 순수성(입력 world 불변)
    r = sim.plan_robot(wmap, sim.Robot(id="r1", pos=start, goal=goal))
    world = sim.World(wmap=wmap, robots=(r,))
    _, telem, _ = sim.step(world)
    assert world.tick == 0 and world.robots[0].pos == start, "step이 입력 world를 변경(순수성 위반)"

    final, frames = sim.run(world)
    fr = final.robots[0]
    assert fr.pos == goal, ("목표 미도달", fr.pos)
    assert fr.status == "arrived", fr.status

    # 4) 결정론: 재실행 동일 tick 수·최종 위치
    r2 = sim.plan_robot(wmap, sim.Robot(id="r1", pos=start, goal=goal))
    final2, frames2 = sim.run(sim.World(wmap=wmap, robots=(r2,)))
    assert final2.robots[0].pos == goal and len(frames2) == len(frames), "시뮬 비결정론"

    print(f"M1 셀프체크 통과 — A* 경로 {len(path)}칸(장애물 회피·4연결), "
          f"step 순수성 OK, 로봇 {len(frames)}tick만에 목표 도달, 결정론 OK")


if __name__ == "__main__":
    demo()
