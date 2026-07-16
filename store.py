# -*- coding: utf-8 -*-
"""시계열 텔레메트리 저장 — SQLite(파일 1개) 스토어.
로봇이 보낸 상태 신호를 시간순으로 쌓고, "최근 N초"를 꺼낸다.
ROS2/외부DB 없이 표준 라이브러리 sqlite3만 사용.
ponytail: 규모가 커져 SQLite가 병목이면 그때 시계열DB로 승격. 지금은 파일 하나면 충분.
"""
import json
import sqlite3
import time
from pathlib import Path

DB_PATH = Path(__file__).parent / "telemetry.db"


def _conn(db_path=None):
    if db_path is None:            # 호출 시점에 모듈 전역을 해석 → 테스트에서 store.DB_PATH 교체 가능
        db_path = DB_PATH
    conn = sqlite3.connect(str(db_path))
    conn.execute(
        "CREATE TABLE IF NOT EXISTS telemetry ("
        "  ts REAL NOT NULL,"       # 유닉스 시각(초)
        "  robot_id TEXT NOT NULL," # 어느 로봇/Pi가 보냈나
        "  metrics TEXT NOT NULL"   # 센서값 묶음(JSON 문자열)
        ")"
    )
    conn.execute("CREATE INDEX IF NOT EXISTS idx_ts ON telemetry(ts)")
    return conn


def insert(robot_id, metrics, ts=None, db_path=None):
    """텔레메트리 한 건 저장. ts 없으면 지금 시각으로. 저장한 ts를 돌려준다."""
    ts = time.time() if ts is None else float(ts)
    with _conn(db_path) as conn:
        conn.execute(
            "INSERT INTO telemetry (ts, robot_id, metrics) VALUES (?, ?, ?)",
            (ts, robot_id, json.dumps(metrics)),
        )
    return ts


def recent(seconds=10.0, robot_id=None, db_path=None):
    """최근 `seconds`초 데이터를 시간순(오래된→최신)으로 반환."""
    cutoff = time.time() - float(seconds)
    q = "SELECT ts, robot_id, metrics FROM telemetry WHERE ts >= ?"
    args = [cutoff]
    if robot_id:
        q += " AND robot_id = ?"
        args.append(robot_id)
    q += " ORDER BY ts ASC"
    with _conn(db_path) as conn:
        rows = conn.execute(q, args).fetchall()
    return [{"ts": r[0], "robot_id": r[1], "metrics": json.loads(r[2])} for r in rows]
