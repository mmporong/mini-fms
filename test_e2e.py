# -*- coding: utf-8 -*-
"""HTTP 통합 테스트 — 발행→수집→저장→조회 왕복(TestClient) + /map + 대시보드.
런타임은 fastapi·uvicorn만, httpx는 test 전용(requirements-dev). 실행: py test_e2e.py"""
import sys
import tempfile
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from fastapi.testclient import TestClient

import store
import app as app_module


def test_roundtrip():
    store.DB_PATH = Path(tempfile.mkdtemp()) / "t.db"          # 격리
    c = TestClient(app_module.app)
    r = c.post("/ingest", json={"robot_id": "r0", "metrics": {"x": 3, "y": 2, "status": "moving", "task": "t1"}})
    assert r.status_code == 200 and r.json()["ok"] is True, r.text
    rows = c.get("/recent?seconds=10").json()["rows"]
    assert len(rows) == 1 and rows[-1]["metrics"]["x"] == 3, rows
    assert c.get("/map").json()["width"] > 0, "map 응답 없음"
    h = c.get("/")
    assert h.status_code == 200 and "FMS Console" in h.text, "대시보드 응답 이상"
    # M3 드릴다운: /recent?robot_id= 는 해당 로봇만·ts 오름차순·타 로봇 미포함
    c.post("/ingest", json={"robot_id": "r0", "metrics": {"x": 4, "y": 2, "status": "moving"}})
    c.post("/ingest", json={"robot_id": "r1", "metrics": {"x": 9, "y": 9, "status": "idle"}})
    d = c.get("/recent?seconds=10&robot_id=r0").json()["rows"]
    assert d and all(row["robot_id"] == "r0" for row in d), ("드릴다운 타 로봇 혼입", d)
    assert [row["ts"] for row in d] == sorted(row["ts"] for row in d), "드릴다운 ts 비오름차순"
    print("test_e2e 통과 — /ingest→/recent 왕복 + /map + 대시보드 + 드릴다운(robot_id) OK")


if __name__ == "__main__":
    test_roundtrip()
