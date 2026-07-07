#!/usr/bin/env python3
"""
上位机 (PC 端) - 铁脉医者 · 轨道巡检机器人可视化管控平台
=============================================
技术栈: Flask + SQLite + Chart.js + WebSocket 实时推送
功能:
  1. 实时数据监控仪表盘
  2. 巡检任务下发
  3. 病害记录查询
  4. 历史数据可视化
  5. 远程遥控

运行: python app.py → 浏览器打开 http://localhost:5000
数据库: SQLite (首次运行自动建表)
"""

import os
import json
import sqlite3
import threading
import time
import random
import math
from datetime import datetime
from dataclasses import dataclass, asdict
from flask import Flask, render_template_string, jsonify, request
from flask_sock import Sock

app = Flask(__name__)
sock = Sock(app)

# ============================================================
# 数据库
# ============================================================
DB_PATH = os.environ.get("DB_PATH", "rail_inspect.db")

_conn_cache = None

def _get_conn():
    global _conn_cache
    if _conn_cache is None:
        try:
            _conn_cache = sqlite3.connect(DB_PATH)
            _conn_cache.row_factory = sqlite3.Row
            init_db()
        except sqlite3.OperationalError:
            # fallback to in-memory database
            _conn_cache = sqlite3.connect(":memory:")
            _conn_cache.row_factory = sqlite3.Row
            init_db()
    return _conn_cache

def init_db():
    conn = _get_conn()
    c = conn.cursor()
    c.execute("""
        CREATE TABLE IF NOT EXISTS inspect_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT DEFAULT (datetime('now','localtime')),
            frame_id INTEGER,
            battery_v REAL,
            battery_soc INTEGER,
            pose_x REAL, pose_y REAL, pose_z REAL,
            roll REAL, pitch REAL, yaw REAL,
            rail_temp REAL,
            speed REAL,
            state TEXT,
            alert_flags INTEGER
        )
    """)
    c.execute("""
        CREATE TABLE IF NOT EXISTS defect_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT DEFAULT (datetime('now','localtime')),
            defect_type TEXT,
            confidence REAL,
            gps_lat REAL, gps_lon REAL,
            size_l REAL, size_w REAL, size_d REAL,
            alert_level INTEGER,
            image_path TEXT
        )
    """)
    c.execute("""
        CREATE TABLE IF NOT EXISTS task_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT DEFAULT (datetime('now','localtime')),
            task_type TEXT,
            start_lat REAL, start_lon REAL,
            end_lat REAL, end_lon REAL,
            status TEXT
        )
    """)
    conn.commit()
    conn.close()

# 替换所有 sqlite3.connect(DB_PATH) 为 _get_conn()
# 批量替换 insert_inspect/insert_defect/get_recent_logs/get_defects/get_stats 中的直接连接

def _get_simple_conn():
    """获取简单连接(非缓存, 用于写入)"""
    try:
        return sqlite3.connect(DB_PATH)
    except:
        return sqlite3.connect(":memory:")

def insert_inspect(data: dict):
    conn = _get_simple_conn()
    c = conn.cursor()
    c.execute("""INSERT INTO inspect_log
        (frame_id, battery_v, battery_soc, pose_x, pose_y, pose_z, roll, pitch, yaw, rail_temp, speed, state, alert_flags)
        VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)""",
        (data.get("frame_id",0), data.get("battery_v",0), data.get("battery_soc",0),
         data.get("pose_x",0), data.get("pose_y",0), data.get("pose_z",0),
         data.get("roll",0), data.get("pitch",0), data.get("yaw",0),
         data.get("rail_temp",0), data.get("speed",0),
         data.get("state",""), data.get("alert_flags",0)))
    conn.commit()
    conn.close()

def insert_defect(data: dict):
    conn = _get_simple_conn()
    c = conn.cursor()
    c.execute("""INSERT INTO defect_log
        (defect_type, confidence, gps_lat, gps_lon, size_l, size_w, size_d, alert_level, image_path)
        VALUES (?,?,?,?,?,?,?,?,?)""",
        (data.get("type",""), data.get("confidence",0),
         data.get("gps",[0,0])[0], data.get("gps",[0,0])[1],
         data.get("size_mm",[0,0,0])[0], data.get("size_mm",[0,0,0])[1], data.get("size_mm",[0,0,0])[2],
         data.get("alert",0), data.get("image_path","")))
    conn.commit()
    conn.close()

def get_recent_logs(limit=100):
    conn = _get_simple_conn()
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    c.execute("SELECT * FROM inspect_log ORDER BY id DESC LIMIT ?", (limit,))
    rows = [dict(r) for r in c.fetchall()]
    conn.close()
    return rows

def get_defects(limit=50):
    conn = _get_simple_conn()
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    c.execute("SELECT * FROM defect_log ORDER BY id DESC LIMIT ?", (limit,))
    rows = [dict(r) for r in c.fetchall()]
    conn.close()
    return rows

def get_stats():
    conn = _get_simple_conn()
    c = conn.cursor()
    c.execute("SELECT COUNT(*) FROM inspect_log")
    total_frames = c.fetchone()[0]
    c.execute("SELECT COUNT(*) FROM defect_log WHERE alert_level >= 2")
    serious_defects = c.fetchone()[0]
    c.execute("SELECT COUNT(*) FROM defect_log")
    total_defects = c.fetchone()[0]
    c.execute("SELECT AVG(battery_v) FROM inspect_log WHERE id > (SELECT MAX(id)-20 FROM inspect_log)")
    row = c.fetchone()
    avg_batt = row[0] or 0
    c.execute("SELECT MAX(speed) FROM inspect_log WHERE id > (SELECT MAX(id)-20 FROM inspect_log)")
    row = c.fetchone()
    max_speed = row[0] or 0
    conn.close()
    return {
        "total_frames": total_frames,
        "total_defects": total_defects,
        "serious_defects": serious_defects,
        "avg_batt_v": round(avg_batt, 1),
        "max_speed": round(max_speed, 2),
    }

# ============================================================
# 模拟下位机数据生成器 (线程)
# ============================================================
simulated_state = {
    "frame_id": 0,
    "battery_v": 46.5,
    "battery_soc": 87,
    "pose_x": 0.0,
    "pose_y": 0.0,
    "pose_z": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "rail_temp": 28.0,
    "speed": 0.45,
    "state": "CRUISING",
    "alert_flags": 0,
    "defects": [],
}

websocket_clients = []

def sim_loop():
    """模拟下位机数据生成 (100Hz → 缩放到1Hz入库)"""
    global simulated_state
    tick = 0
    while True:
        time.sleep(1.0)
        tick += 1

        s = simulated_state
        s["frame_id"] += 1
        s["battery_v"] = 46.5 - tick * 0.001 + random.gauss(0, 0.1)
        s["battery_soc"] = int((s["battery_v"] - 36.0) / 12.0 * 100)
        s["yaw"] += random.gauss(0, 0.02)
        s["pose_x"] += 0.45 * math.cos(s["yaw"])
        s["pose_y"] += 0.45 * math.sin(s["yaw"])
        s["rail_temp"] = 28.0 + random.uniform(-1, 3)
        s["speed"] = max(0, 0.45 + random.gauss(0, 0.05))

        # 模拟病害检测
        if random.random() < 0.15:
            types = ["裂纹", "剥离", "压溃", "波磨", "扣件松动"]
            defect = {
                "type": random.choice(types),
                "type_id": random.choice([0, 1, 2, 3, 4]),
                "confidence": round(random.uniform(0.55, 0.98), 3),
                "gps": [round(31.23 + s["pose_x"] * 0.00001, 6),
                        round(121.47 + s["pose_y"] * 0.00001, 6)],
                "size_mm": [round(random.uniform(2, 80), 1),
                            round(random.uniform(1, 30), 1),
                            round(random.uniform(0, 5), 1)],
                "alert": random.choice([1, 1, 2, 2, 3]),
                "timestamp": datetime.now().isoformat(),
            }
            insert_defect(defect)
            s["defects"] = [defect]  # 只推送最新

            if defect["alert"] >= 3:
                s["alert_flags"] |= 0x08
                s["state"] = "ALERT"
        else:
            s["defects"] = []

        # 入库
        insert_inspect(s)

        # WebSocket 推送
        msg = json.dumps(s, ensure_ascii=False)
        dead = []
        for ws in websocket_clients:
            try:
                ws.send(msg)
            except:
                dead.append(ws)
        for d in dead:
            websocket_clients.remove(d)

        if s["alert_flags"] & 0x08:
            s["alert_flags"] &= ~0x08
            if s["state"] == "ALERT":
                s["state"] = "CRUISING"

# ============================================================
# WebSocket
# ============================================================
@sock.route("/ws")
def websocket_handler(ws):
    websocket_clients.append(ws)
    try:
        while True:
            data = ws.receive()
            if data:
                try:
                    cmd = json.loads(data)
                    print(f"[CMD] {cmd}")
                except:
                    pass
    except:
        pass
    finally:
        if ws in websocket_clients:
            websocket_clients.remove(ws)

# ============================================================
# REST API
# ============================================================
@app.route("/api/ingest", methods=["POST"])
def api_ingest():
    """Receive data from NUC engine"""
    data = request.json
    if not data:
        return jsonify({"error": "empty"}), 400
    # store sensor telemetry
    telemetry = {
        "frame_id": data.get("frame_id", 0),
        "battery_v": data.get("battery", {}).get("v", 0),
        "battery_soc": data.get("battery", {}).get("soc", 0),
        "pose_x": data.get("pose", [0]*6)[0],
        "pose_y": data.get("pose", [0]*6)[1],
        "pose_z": data.get("pose", [0]*6)[2],
        "roll": data.get("pose", [0]*6)[3],
        "pitch": data.get("pose", [0]*6)[4],
        "yaw": data.get("pose", [0]*6)[5],
        "rail_temp": data.get("rail_temp", 0),
        "speed": data.get("speed", 0),
        "state": str(data.get("state", "")),
        "alert_flags": data.get("alert_flags", 0),
    }
    insert_inspect(telemetry)
    # store defects
    for d in data.get("defects", []):
        defect = {
            "type": d.get("label", ""),
            "confidence": d.get("confidence", 0),
            "gps": [d.get("gps_lat", 0), d.get("gps_lon", 0)],
            "size_mm": d.get("size_mm", [0, 0, 0]),
            "alert": d.get("alert_level", 0),
        }
        insert_defect(defect)
    return jsonify({"ok": True})

@app.route("/api/status")
def api_status():
    return jsonify(simulated_state)

@app.route("/api/logs")
def api_logs():
    limit = request.args.get("limit", 100, type=int)
    return jsonify(get_recent_logs(limit))

@app.route("/api/defects")
def api_defects():
    limit = request.args.get("limit", 50, type=int)
    return jsonify(get_defects(limit))

@app.route("/api/stats")
def api_stats():
    return jsonify(get_stats())

@app.route("/api/command", methods=["POST"])
def api_command():
    cmd = request.json
    print(f"[CMD下发] {cmd}")
    # 实际: 通过串口/CAN发送给STM32
    return jsonify({"result": "ok", "cmd": cmd})

# ============================================================
# 页面模板
# ============================================================
INDEX_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>铁脉医者</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.5.0/dist/chart.umd.js"
 integrity="sha384-iU8HYtnGQ8Cy4zl7gbNMOhsDTTKX02BTXptVP/vqAWIaTfM7isw76iyZCsjL2eVi"
 crossorigin="anonymous"></script>
<style>
:root{color-scheme:light}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:"Microsoft YaHei","PingFang SC",sans-serif;background:#f0f2f5;color:#1a1a2e;display:flex;height:100vh;overflow:hidden}
/* sidebar */
.sidebar{width:200px;background:linear-gradient(180deg,#0f2027 0%,#203a43 50%,#2c5364 100%);color:#fff;flex-shrink:0;padding:20px 0}
.sidebar h2{padding:0 16px 16px;font-size:16px;border-bottom:1px solid rgba(255,255,255,.15)}
.sidebar a{display:block;padding:10px 20px;color:rgba(255,255,255,.8);text-decoration:none;font-size:13px;border-left:3px solid transparent}
.sidebar a:hover,.sidebar a.active{background:rgba(255,255,255,.1);color:#fff;border-left-color:#00e676}
/* main */
.main{flex:1;overflow-y:auto;padding:20px}
.topbar{background:#fff;border-radius:8px;padding:14px 20px;margin-bottom:16px;display:flex;justify-content:space-between;align-items:center;box-shadow:0 1px 3px rgba(0,0,0,.06)}
.topbar h1{font-size:18px}
.topbar .time{font-size:12px;color:#999}
/* stat cards */
.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:14px;margin-bottom:16px}
.card{background:#fff;border-radius:8px;padding:16px;box-shadow:0 1px 3px rgba(0,0,0,.06);text-align:center}
.card .val{font-size:28px;font-weight:700}
.card .lbl{font-size:12px;color:#888;margin-top:4px}
.card .g{color:#2e7d32}.card .b{color:#1565c0}.card .o{color:#e65100}.card .p{color:#6a1b9a}
/* panels */
.panels{display:grid;grid-template-columns:2fr 1fr;gap:14px}
.panel{background:#fff;border-radius:8px;padding:16px;box-shadow:0 1px 3px rgba(0,0,0,.06)}
.panel h3{font-size:14px;margin-bottom:12px;color:#333}
.chart-wrap{position:relative;height:250px}
/* defect table */
.tbl{width:100%;font-size:12px;border-collapse:collapse;margin-top:8px}
.tbl th{background:#f5f5f5;text-align:left;padding:6px 8px;font-weight:600;border-bottom:2px solid #ddd}
.tbl td{padding:6px 8px;border-bottom:1px solid #eee}
.alert-l1{color:#1565c0}.alert-l2{color:#e65100;font-weight:600}.alert-l3{color:#c62828;font-weight:700}
/* buttons */
.btn{padding:6px 16px;border:none;border-radius:6px;cursor:pointer;font-size:12px;font-weight:500;margin:2px}
.btn-green{background:#43a047;color:#fff}.btn-red{background:#e53935;color:#fff}
.btn-blue{background:#1e88e5;color:#fff}.btn-orange{background:#fb8c00;color:#fff}
#defect-badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:11px;font-weight:600;margin-left:8px}
.badge-ok{background:#e8f5e9;color:#2e7d32}.badge-warn{background:#fff3e0;color:#e65100}.badge-crit{background:#ffebee;color:#c62828}
</style>
</head>
<body>

<nav class="sidebar">
  <h2>🛤 RailInspect</h2>
  <a href="#" class="active">📊 仪表盘</a>
  <a href="#">📍 实时轨迹</a>
  <a href="#">🔍 病害记录</a>
  <a href="#">📋 巡检任务</a>
  <a href="#">📈 趋势分析</a>
  <a href="#">⚙ 系统设置</a>
  <a href="#" style="margin-top:20px;font-size:11px;color:rgba(255,255,255,.4)">STM32F407 + Flask<br>芯片设计大赛</a>
</nav>

<main class="main">
  <div class="topbar">
    <h1>铁脉医者 <span id="defect-badge" class="badge-ok">● 正常巡检</span></h1>
    <div class="time" id="clock">--</div>
  </div>

  <div class="cards">
    <div class="card"><div class="val g" id="val-batt">46.5V</div><div class="lbl">电池电压</div></div>
    <div class="card"><div class="val b" id="val-speed">0.45</div><div class="lbl">当前速度 (m/s)</div></div>
    <div class="card"><div class="val o" id="val-defects">0</div><div class="lbl">累计病害</div></div>
    <div class="card"><div class="val p" id="val-state">巡航中</div><div class="lbl">机器人状态</div></div>
  </div>

  <div class="panels">
    <div class="panel">
      <h3>📈 实时传感器数据 <small style="color:#999">(1 Hz 刷新)</small></h3>
      <div class="chart-wrap"><canvas id="chartPose"></canvas></div>
    </div>
    <div class="panel">
      <h3>⚠ 最近病害检测 <button class="btn btn-blue" onclick="fetchDefects()">刷新</button></h3>
      <div id="defect-list" style="max-height:250px;overflow-y:auto">
        <table class="tbl"><thead><tr><th>类型</th><th>置信度</th><th>尺寸(mm)</th><th>等级</th><th>时间</th></tr></thead>
        <tbody id="defect-tbody"><tr><td colspan="5" style="color:#999">加载中...</td></tr></tbody></table>
      </div>
    </div>
  </div>

  <div class="panels" style="margin-top:14px;grid-template-columns:1fr 1fr">
    <div class="panel">
      <h3>🎮 远程遥控
        <button class="btn btn-green" onclick="sendCmd('start')">▶ 启动巡检</button>
        <button class="btn btn-orange" onclick="sendCmd('pause')">⏸ 暂停</button>
        <button class="btn btn-red" onclick="sendCmd('stop')">■ 紧急制动</button>
      </h3>
      <div style="font-size:12px;color:#888;margin-top:8px">
        速度控制: <input type="range" min="0" max="80" value="45" style="width:120px" id="speed-slider">
        <span id="speed-val">0.45</span> m/s<br>
        指令记录: <span id="cmd-log">--</span>
      </div>
    </div>
    <div class="panel">
      <h3>📋 系统信息</h3>
      <div style="font-size:12px;line-height:1.8">
        <table class="tbl">
          <tr><td>下位机</td><td><b>STM32F407IGT6</b> @ 168MHz</td></tr>
          <tr><td>RTOS</td><td>FreeRTOS v10 · 6任务</td></tr>
          <tr><td>通信</td><td>CAN 1Mbps + UART 5G</td></tr>
          <tr><td>传感器</td><td>MPU6050 IMU + 8ch超声 + RTK</td></tr>
          <tr><td>电机</td><td>4×直流伺服 + PID闭环</td></tr>
          <tr><td>数据库</td><td>SQLite · 帧数: <span id="sys-frames">0</span></td></tr>
          <tr><td>运行时间</td><td><span id="sys-uptime">--</span></td></tr>
        </table>
      </div>
    </div>
  </div>
</main>

<script>
// state
var chartPose = null;
var poseHistory = [];
var defectCount = 0;
var startTime = new Date();

// clock
setInterval(function(){ document.getElementById('clock').textContent = new Date().toLocaleString(); }, 1000);

// speed slider
var slider = document.getElementById('speed-slider');
var speedVal = document.getElementById('speed-val');
slider.oninput = function(){ speedVal.textContent = (this.value / 100).toFixed(2); };

// init chart
var ctxPose = document.getElementById('chartPose').getContext('2d');
chartPose = new Chart(ctxPose, {
  type: 'line',
  data: {
    labels: [],
    datasets: [
      {label:'Yaw (rad)', data:[], borderColor:'#1565c0', tension:0.3, pointRadius:0},
      {label:'速度 (m/s)', data:[], borderColor:'#43a047', tension:0.3, pointRadius:0},
    ]
  },
  options: {
    responsive:true, maintainAspectRatio:false,
    animation:false,
    scales:{ x:{display:true}, y:{display:true} },
    plugins:{legend:{position:'top',labels:{font:{size:10}}}}
  }
});

// WebSocket
var ws = null;
function connectWS(){
  var proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  ws = new WebSocket(proto + '//' + location.host + '/ws');
  ws.onmessage = function(e){
    var d = JSON.parse(e.data);
    updateUI(d);
  };
  ws.onclose = function(){ setTimeout(connectWS, 2000); };
}

function updateUI(d){
  document.getElementById('val-batt').textContent = d.battery_v.toFixed(1) + 'V';
  document.getElementById('val-speed').textContent = d.speed.toFixed(2);
  document.getElementById('val-defects').textContent = defectCount;
  var states = {'IDLE':'待机','READY':'就绪','CRUISING':'巡航中','INSPECTING':'检测中','ALERT':'⚠告警','EMERGENCY_STOP':'■急停','CHARGING':'充电','ERROR':'故障'};
  document.getElementById('val-state').textContent = states[d.state] || d.state;

  // badge
  var badge = document.getElementById('defect-badge');
  if(d.alert_flags & 8){ badge.className='badge-crit'; badge.textContent='● 严重告警'; }
  else if(d.alert_flags & 2){ badge.className='badge-warn'; badge.textContent='● 障碍物预警'; }
  else{ badge.className='badge-ok'; b