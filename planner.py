# -*- coding: utf-8 -*-
"""A* 경로계획 — 4-연결 격자, 맨해튼 휴리스틱, 결정론 tie-break.
replan()이 로컬 재계획·교착후 재계획의 단일 진입점(발산 구현 2개 금지 — 계획 SHOULD-7)."""
import heapq


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(wmap, start, goal, extra_blocked=frozenset(), cost=None):
    """start→goal 최소비용 경로(셀 리스트, start·goal 포함). 없으면 None.
    extra_blocked = 동적 장애물(다른 로봇 점유 셀). cost={cell:추가비용} = 혼잡 페널티(밀집 셀 우회 → 부하분산).
    heap tie-break=(f,g,counter,cell)로 결정론. 휴리스틱(맨해튼)은 비용≥1이라 admissible 유지."""
    if start == goal:
        return [start]
    counter = 0
    open_heap = [(manhattan(start, goal), 0, counter, start)]
    came_from = {start: None}
    gscore = {start: 0}
    while open_heap:
        _, g, _, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = came_from[cur]
            return path[::-1]
        for nb in sorted(wmap.neighbors(cur)):      # 정렬 순회 → 결정론
            if nb in extra_blocked:
                continue
            ng = g + 1 + (cost.get(nb, 0) if cost else 0)   # 혼잡 비용 = 밀집 셀 회피(부하분산)
            if nb not in gscore or ng < gscore[nb]:
                gscore[nb] = ng
                came_from[nb] = cur
                counter += 1
                heapq.heappush(open_heap, (ng + manhattan(nb, goal), ng, counter, nb))
    return None


def replan(wmap, start, goal, extra_blocked=frozenset(), cost=None):
    """단일 재계획 진입점 — planner(로컬)·coordinator(교착후) 공용. cost=혼잡 페널티(선택)."""
    return astar(wmap, start, goal, extra_blocked, cost)
