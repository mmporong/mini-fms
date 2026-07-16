# -*- coding: utf-8 -*-
"""2D 격자 창고 맵 — 선반(장애물)·픽업/배송 지점. 순수 데이터 + 조회 헬퍼.
(모듈명 map은 파이썬 내장 map과 혼동되므로 gridmap.)"""
from dataclasses import dataclass


@dataclass(frozen=True)
class WarehouseMap:
    width: int
    height: int
    obstacles: frozenset      # {(x,y)} 선반
    pickups: tuple = ()        # ((x,y),...)
    dropoffs: tuple = ()

    def in_bounds(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell):
        return self.in_bounds(cell) and cell not in self.obstacles

    def neighbors(self, cell):
        """4-연결 자유 이웃 (결정론 위해 호출부에서 정렬)."""
        x, y = cell
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (x + dx, y + dy)
            if self.is_free(n):
                yield n


def from_ascii(rows):
    """ASCII 맵 파싱: '#'=선반, 'P'=픽업, 'D'=배송, '.'=자유. rows=문자열 리스트."""
    obstacles, pickups, dropoffs = set(), [], []
    h = len(rows)
    w = max(len(r) for r in rows)
    for y, row in enumerate(rows):
        for x, ch in enumerate(row):
            c = (x, y)
            if ch == "#":
                obstacles.add(c)
            elif ch == "P":
                pickups.append(c)
            elif ch == "D":
                dropoffs.append(c)
    return WarehouseMap(w, h, frozenset(obstacles), tuple(pickups), tuple(dropoffs))


def warehouse(blocks_x=6, blocks_y=5, block_w=4, block_h=3, aisle=2, margin=2):
    """블록형 창고 격자 생성(현업 규모) — 선반 블록 배열 + 통로 + 좌측 depot(staging).
    반환: (WarehouseMap, depots). 기본 ≈38x27(≈1000셀, 소형 대비 10배)."""
    content_x = blocks_x * block_w + (blocks_x - 1) * aisle
    content_y = blocks_y * block_h + (blocks_y - 1) * aisle
    width, height = content_x + 2 * margin, content_y + 2 * margin
    obstacles = set()
    for bx in range(blocks_x):
        ox = margin + bx * (block_w + aisle)
        for by in range(blocks_y):
            oy = margin + by * (block_h + aisle)
            for dx in range(block_w):
                for dy in range(block_h):
                    obstacles.add((ox + dx, oy + dy))
    wmap = WarehouseMap(width, height, frozenset(obstacles), (), ())
    depots = [(x, y) for x in range(margin) for y in range(height) if (x, y) not in obstacles]
    return wmap, depots


def stations(wmap, n_pick=14):
    """물류 지점 — 픽업=선반 접근 rack(분산 n_pick개), 배송=우측 dock 열. 고정 위치라 적재/하역이 눈에 보인다."""
    racks = sorted({(x + dx, y + dy) for (x, y) in wmap.obstacles
                    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                    if wmap.is_free((x + dx, y + dy)) and x + dx >= 2})   # depot 열은 제외
    step = max(1, len(racks) // n_pick)
    pickups = racks[::step][:n_pick]
    dropoffs = [(wmap.width - 1, y) for y in range(wmap.height) if wmap.is_free((wmap.width - 1, y))]
    return pickups, dropoffs


def spread(wmap, n, reserved=frozenset()):
    """자유 셀을 균등 분산해 n개 선택(로봇 초기·home 배치 → 시작부터 뭉치지 않음)."""
    free = [(x, y) for y in range(wmap.height) for x in range(wmap.width)
            if wmap.is_free((x, y)) and (x, y) not in reserved]
    step = max(1, len(free) // n)
    picked = free[::step][:n]
    for c in free:                       # 부족분 채움
        if len(picked) >= n:
            break
        if c not in picked:
            picked.append(c)
    return picked[:n]
