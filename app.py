# -*- coding: utf-8 -*-
"""FMS 관제 대시보드 — 2D 격자 맵에 로봇 fleet 실시간.
pi_fleet_console 재사용: store(복사)·/ingest·/recent·폴링 스캐폴드. drawViz만 격자 맵 렌더로 교체.
로봇 경로선=주행 궤적(위치 누적). 경보: 고장(down). run.py가 MAP을 설정(GET /map)."""
import json

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, PlainTextResponse
from pydantic import BaseModel

import store

app = FastAPI(title="FMS Console")

# run.py가 시나리오 맵으로 덮어쓴다.
MAP = {"width": 8, "height": 8, "obstacles": [], "pickups": [], "dropoffs": []}


class Telemetry(BaseModel):
    robot_id: str
    metrics: dict
    ts: float | None = None


@app.post("/ingest")
def ingest(t: Telemetry):
    ts = store.insert(t.robot_id, t.metrics, t.ts)
    return {"ok": True, "ts": ts}


@app.post("/ingest_batch")
def ingest_batch(batch: dict):
    """한 tick의 fleet 텔레메트리를 1회 발행(대규모 40+대에서 tick당 40 POST 방지)."""
    for row in batch.get("rows", []):
        store.insert(row["robot_id"], row["metrics"], row.get("ts"))
    return {"ok": True, "n": len(batch.get("rows", []))}


@app.get("/recent")
def recent(seconds: float = 10.0, robot_id: str | None = None):
    return {"rows": store.recent(seconds, robot_id)}


@app.get("/map")
def get_map():
    return MAP


# run.py가 매 tick 갱신: 로봇 스냅샷·동적 폐쇄·통로 방향·집계 지표·최근 이벤트
STATE = {"robots": [], "dyn_blocked": [], "oneway": [], "blocked_queue": [], "nav_trace": [], "metrics": {}, "events": []}


SPEED = 1.0   # 실행 속도 배율(기본 1배) — run.py on_tick이 읽어 sleep 조절


@app.get("/speed")
def set_speed(mult: float = None):
    global SPEED
    if mult is not None:
        SPEED = max(0.25, min(8.0, mult))
    return {"speed": SPEED}


@app.get("/state")
def get_state():
    return STATE


@app.get("/trace.jsonl", response_class=PlainTextResponse)
def get_trace():
    """자동주행 결정 트레이스 flight recorder(ASPIRE식 구조화 JSON lines) — 재경로·양보·교착해소를 tick별 기록.
    다운로드/분석·replay용. 각 줄: {tick, kind, robot, cause}."""
    return "\n".join(json.dumps(r, ensure_ascii=False) for r in STATE.get("nav_trace", []))


@app.get("/", response_class=HTMLResponse)
def dashboard():
    return DASHBOARD_HTML


DASHBOARD_HTML = """<!doctype html>
<meta charset="utf-8">
<title>FMS Console</title>
<style>
 body{font-family:ui-monospace,Menlo,Consolas,monospace;background:#0e1116;color:#e6edf3;margin:0;padding:24px;font-size:14px}
 h1{font-size:21px;margin:0 0 4px} .sub{color:#8b949e;font-size:13px;margin-bottom:14px}
 .dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#3fb950;margin-right:6px}
 .dot.off{background:#f85149}
 .alert{background:#3d1416;border:1px solid #f85149;color:#ffb3b3;border-radius:6px;padding:8px 14px;margin-bottom:12px;font-size:14px;display:none}
 .wrap{display:flex;gap:20px;align-items:flex-start;flex-wrap:wrap}
 #viz{width:48%;flex:0 0 48%;height:auto;display:block}
 canvas{background:#0b0e13;border:1px solid #21262d;border-radius:8px;max-width:100%}
 .panel{flex:1;min-width:320px;margin-top:0}
 .logs{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:14px 24px;margin-top:14px;align-items:start}
 .ctrl{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:12px;font-size:13px;color:#adbac7}
 .ctrl button{background:#161b22;border:1px solid #30363d;color:#e6edf3;border-radius:5px;padding:5px 11px;cursor:pointer;font:inherit}
 .ctrl button.on{background:#1f6feb;border-color:#1f6feb}
 .kpi{display:flex;gap:12px;flex-wrap:wrap;margin-bottom:14px}
 .kpi .k{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px 15px;min-width:84px}
 .kpi .v{font-size:27px;font-variant-numeric:tabular-nums;line-height:1.1;font-weight:600} .kpi .l{color:#adbac7;font-size:12px;margin-top:3px}
 .bar{display:flex;height:22px;border-radius:5px;overflow:hidden;margin:6px 0 12px;background:#161b22}
 .bar span{display:block} .legend{color:#c9d1d9;font-size:13px;display:flex;gap:16px;flex-wrap:wrap;margin-bottom:12px}
 .sw{display:inline-block;width:11px;height:11px;border-radius:2px;margin-right:5px;vertical-align:middle}
 .ttl{font-size:13px;color:#8b949e;margin:0 0 4px;font-weight:600}
 .count{color:#c9d1d9;font-size:13.5px;line-height:1.7} .events{color:#c9d1d9;font-size:13.5px;line-height:1.75;white-space:pre-line;max-height:320px;overflow:auto;
   border-top:1px solid #21262d;padding-top:10px;margin-top:10px}
</style>
<h1>🤖 FMS Console — 다중 로봇 관제 (현업 규모·연속 물류)</h1>
<div class="sub"><span id="dot" class="dot off"></span><span id="status">연결 대기…</span> · 폴링 250ms · DB <span id="db">0</span>행</div>
<div class="alert" id="alert"></div>
<div class="ctrl">
 <span>속도</span><span id="spd">1×</span>
 <button class="spd" data-m="0.5" onclick="setSpeed(0.5)">0.5×</button>
 <button class="spd" data-m="1" onclick="setSpeed(1)">1×</button>
 <button class="spd" data-m="2" onclick="setSpeed(2)">2×</button>
 <button class="spd" data-m="4" onclick="setSpeed(4)">4×</button>
 <button class="spd" data-m="8" onclick="setSpeed(8)">8×</button>
 <span style="margin-left:12px">라벨</span>
 <button id="lm0" onclick="setLabel(0)">번호</button>
 <button id="lm1" onclick="setLabel(1)">번호+우선순위</button>
 <button id="lm2" onclick="setLabel(2)">끄기</button>
 <span style="margin-left:12px">로봇 클릭 → 상세(우선순위 포함)</span>
</div>
<div class="wrap">
 <canvas id="viz" width="1140" height="810"></canvas>
 <div class="panel">
  <div class="kpi" id="kpi"></div>
  <div class="bar" id="bar"></div>
  <div class="legend">
   <span>로봇 색 = 배송 지연:</span>
   <span><i class="sw" style="background:#ffffff;border:1px solid #444c56"></i>방금</span>
   <span><i class="sw" style="background:#f7c8c3"></i>지연</span>
   <span><i class="sw" style="background:#f85149"></i>오래 지연</span>
   <span><i class="sw" style="background:#6e7681"></i>유휴</span>
   <span><i class="sw" style="background:#484f58"></i>고장(X)</span>
   <span><i class="sw" style="background:#f0d060"></i>적재(금색 링)</span>
   <span><i class="sw" style="background:#1f6feb"></i>픽업 rack</span><span><i class="sw" style="background:#8a6d1f"></i>배송 dock</span>
   <span style="color:#8b949e">상태(이동/대기/…)는 위 막대·요약 참고</span>
  </div>
  <div class="logs">
   <div>
    <div class="ttl">📊 처리 흐름</div>
    <div class="count" id="count"></div>
    <div class="count" id="oneway" style="margin-top:6px;color:#a5d6ff"></div>
    <canvas id="spark" width="320" height="88" style="margin-top:10px;display:block;width:100%;background:#0b0e13;border:1px solid #21262d;border-radius:6px"></canvas>
    <div class="count" id="sparklbl" style="margin-top:4px"></div>
   </div>
   <div>
    <div class="ttl">⚠ 개입 필요(도달불가)</div>
    <div class="count" id="blocked" style="white-space:pre-line;color:#f0883e;max-height:220px;overflow:auto"></div>
   </div>
   <div>
    <div class="ttl">🧭 자동주행 결정</div>
    <div class="count" id="drill" style="white-space:pre-line;color:#79c0ff;margin-bottom:6px"></div>
    <div class="count" id="nav" style="white-space:pre-line;color:#a5d6ff;max-height:200px;overflow:auto"></div>
    <div class="count" style="margin-top:6px"><a href="/trace.jsonl" style="color:#8b949e">주행 트레이스 다운로드(.jsonl)</a></div>
   </div>
   <div>
    <div class="ttl">📜 이벤트</div>
    <div class="events" id="events" style="border-top:none;margin-top:0;padding-top:0"></div>
   </div>
  </div>
 </div>
</div>
<script>
const COLOR = {moving:"#3fb950", waiting:"#e3b341", arrived:"#58a6ff", down:"#f85149", idle:"#7d8590", blocked:"#f0883e"};
const EVKO = {task_spawn:"임무 발생", fault_derived:"고장 파생", recovered:"로봇 회복",
  aisle_close:"통로 폐쇄", aisle_open:"통로 개방", task_reassign:"임무 재배분", task_blocked:"임무 차단(개입)",
  reroute:"혼잡 재경로", yield_idle:"유휴 양보", deadlock:"교착 해소", oneway:"통로 방향잠금"};
let MAP=null, STATE={robots:[],dyn_blocked:[],oneway:[],blocked_queue:[],nav_trace:[],metrics:{},events:[]};
let SPARK=[], SEL=null, LABELMODE=0;   // 시계열 · 선택 로봇 · 라벨(0=번호 1=번호+우선순위 2=끄기)

function sizeCanvas(){                  // 고해상도(크리스프) — 표시 폭×DPR을 버퍼로
 if(!MAP) return; const c=document.getElementById('viz'), dpr=Math.min(2,window.devicePixelRatio||1);
 const w=Math.round((c.clientWidth||900)*dpr); c.width=w; c.height=Math.round(w*MAP.height/MAP.width);
}
async function setSpeed(m){ try{ const s=await (await fetch('/speed?mult='+m)).json();
 document.getElementById('spd').textContent=s.speed+'×';
 document.querySelectorAll('.spd').forEach(b=>b.className=(parseFloat(b.dataset.m)===s.speed?'spd on':'spd')); }catch(e){} }
function setLabel(m){ LABELMODE=m; for(let i=0;i<3;i++) document.getElementById('lm'+i).className=(i===m?'on':''); }

async function loadMap(){ MAP = await (await fetch('/map')).json(); sizeCanvas(); setLabel(0);
 try{ const s=await (await fetch('/speed')).json(); document.getElementById('spd').textContent=s.speed+'×';   // 현재 속도 하이라이트(리셋 없이)
  document.querySelectorAll('.spd').forEach(b=>b.className=(parseFloat(b.dataset.m)===s.speed?'spd on':'spd')); }catch(e){} }
window.addEventListener('resize', sizeCanvas);

async function poll(){
 try{
  try{ STATE = await (await fetch('/state')).json(); syncAnim(); }catch(e){}   // 보간 목표 갱신(부드러운 렌더)
  let dbn=0; try{ dbn=(await (await fetch('/recent?seconds=6')).json()).rows.length; }catch(e){}  // 파이프라인 생존 지표
  const R=STATE.robots||[], mt=STATE.metrics||{};
  document.getElementById('db').textContent=dbn;
  document.getElementById('dot').className='dot'+(R.length?'':' off');
  document.getElementById('status').textContent=R.length?'수신 중':'데이터 없음 (시뮬 실행?)';
  // KPI
  document.getElementById('kpi').innerHTML = [
    ['배송완료', mt.delivered??0], ['적재중', mt.carrying??0], ['대기물류', mt.active??0], ['처리량', (mt.throughput??0)+'/t'],
    ['고장', mt.faults??0], ['회복', mt.recovered??0], ['차단', mt.blocked??0], ['tick', mt.ticks??0],
  ].map(([l,v])=>`<div class="k"><div class="v">${v}</div><div class="l">${l}</div></div>`).join('');
  // 상태 요약 바
  const cnt={}; for(const r of R) cnt[r.status]=(cnt[r.status]||0)+1;
  const order=['moving','waiting','arrived','idle','down','blocked'], tot=R.length||1;
  document.getElementById('bar').innerHTML = order.filter(s=>cnt[s]).map(s=>
    `<span style="width:${cnt[s]/tot*100}%;background:${COLOR[s]}" title="${s} ${cnt[s]}"></span>`).join('');
  document.getElementById('count').textContent =
    order.filter(s=>cnt[s]).map(s=>`${s} ${cnt[s]}`).join(' · ') + ` · 총 ${R.length}대`;
  // 통로 방향잠금 표시
  const ow=STATE.oneway||[];
  document.getElementById('oneway').textContent = ow.length? `↔ 통로 방향잠금(one-way) ${ow.length}곳 활성` : '';
  // 차단(개입 필요) 큐 — 도달불가 태스크
  const bq=STATE.blocked_queue||[];
  document.getElementById('blocked').textContent = bq.length?
    `${bq.length}건\\n`+bq.slice(-8).map(q=>`${q.id}  픽업${q.pickup}→배송${q.dropoff}`).join('\\n') : '없음';
  // 처리량/백로그 시계열 축적 + 스파크라인
  if(mt.ticks!=null){ SPARK.push([mt.throughput??0, mt.active??0]); if(SPARK.length>120) SPARK.shift(); }
  drawSpark();
  document.getElementById('sparklbl').innerHTML =
    `<span style="color:#3fb950">▬ 처리량 ${mt.throughput??0}/t</span>&nbsp;&nbsp;<span style="color:#e3b341">▬ 대기물류 ${mt.active??0}</span>`;
  // 로봇 드릴다운(선택 로봇 상세 — 우선순위 포함)
  if(SEL){ const sr=R.find(r=>r.id===SEL);
    let head = sr? `🔍 ${SEL} · ${sr.status}${sr.task?' · 임무 '+sr.task+' · 배송경과 '+sr.age+'t':' · 유휴'}`
                   +` · 우선순위 p${sr.pr}(유효 ${sr.eff}, 대기 ${sr.stuck}t)`
                   +`${sr.wait_reason&&sr.wait_reason!=='none'?' · '+sr.wait_reason:''}` : `🔍 ${SEL}`;
    try{ const {rows}=await (await fetch('/recent?seconds=30&robot_id='+SEL)).json();
      head += `\\n최근 ${rows.length}: `+rows.slice(-6).map(r=>`(${r.metrics.x},${r.metrics.y})`).join(' '); }catch(e){}
    document.getElementById('drill').textContent = head;
  } else document.getElementById('drill').textContent='';
  // 자동주행 결정 트레이스(ASPIRE식 — 재경로/양보/교착)
  const NAVKO={reroute:"혼잡 재경로", yield_idle:"유휴 양보", deadlock:"교착 해소"};
  const nv=STATE.nav_trace||[];
  document.getElementById('nav').textContent = nv.length?
    nv.slice(-10).reverse().map(r=>`t${r.tick} ${NAVKO[r.kind]||r.kind} ${r.robot||''}${r.cause?' ('+r.cause+')':''}`).join('\\n') : '없음';
  // 이벤트(한글)
  document.getElementById('events').textContent =
    (STATE.events||[]).slice(-14).reverse().map(e=>`t${e.tick}  ${EVKO[e.type]||e.type}${e.robot?' '+e.robot:''}${e.task?' '+e.task:''}`).join('\\n');
  // 경보
  const downs=R.filter(r=>r.status==='down').map(r=>r.id), al=document.getElementById('alert');
  if(downs.length){ al.style.display='block';
    al.textContent='🔴 로봇 고장: '+downs.join(', ')+' — coordinator 재배분 후 회복(towed) 진행'; }
  else al.style.display='none';
  // 그리기는 animate()의 requestAnimationFrame 루프가 담당(부드러운 보간)
 }catch(e){ document.getElementById('dot').className='dot off'; document.getElementById('status').textContent='서버 끊김'; }
}

let ANIM={}, lastPollT=0;                  // 로봇별 보간(from→to, 시간기반 등속 — 감속·정지 없이 매끄럽게)
function syncAnim(){                        // 새 STATE 도착 시 보간 목표 갱신(현재 렌더 위치에서 출발)
 const now=performance.now(), dur=lastPollT?Math.max(80,Math.min(450,now-lastPollT)):180; lastPollT=now;
 const ids=new Set();
 for(const r of (STATE.robots||[])){ ids.add(r.id); const a=ANIM[r.id];
  const fx=a?a.rx:r.x, fy=a?a.ry:r.y;
  ANIM[r.id]={fx,fy,tx:r.x,ty:r.y,t0:now,dur,rx:fx,ry:fy}; }
 for(const id in ANIM) if(!ids.has(id)) delete ANIM[id];
}
function animate(){                        // 60fps — poll 간격 동안 목표 셀로 '등속' 선형 보간(등속=멀미 없음)
 const now=performance.now(), cur={};
 for(const id in ANIM){ const a=ANIM[id], p=a.dur?Math.min(1,(now-a.t0)/a.dur):1;
  a.rx=a.fx+(a.tx-a.fx)*p; a.ry=a.fy+(a.ty-a.fy)*p; cur[id]={x:a.rx,y:a.ry}; }
 draw(STATE.robots||[], cur);
 requestAnimationFrame(animate);
}

function draw(R, cur){
 if(!MAP) return;
 const c=document.getElementById('viz'), g=c.getContext('2d'), W=c.width, H=c.height;
 const cs=Math.max(4,Math.floor(Math.min(W/MAP.width, H/MAP.height)));
 const px=x=>x*cs+cs/2, py=y=>y*cs+cs/2;
 g.clearRect(0,0,W,H);   // 로봇 색 = 배송 경과(age): 흰=방금·유휴 회색 → 빨강=오래 지연(반납 시 리셋)
 g.fillStyle='#30363d'; for(const [x,y] of MAP.obstacles) g.fillRect(x*cs,y*cs,cs-1,cs-1);           // 선반
 g.fillStyle='#1f6feb'; for(const [x,y] of (MAP.pickups||[])) g.fillRect(x*cs+cs*0.12,y*cs+cs*0.12,cs*0.76,cs*0.76); // 픽업 rack
 g.fillStyle='#8a6d1f'; for(const [x,y] of (MAP.dropoffs||[])) g.fillRect(x*cs,y*cs,cs-1,cs-1);       // 배송 dock
 g.fillStyle='rgba(248,81,73,.5)'; for(const [x,y] of (STATE.dyn_blocked||[])) g.fillRect(x*cs,y*cs,cs-1,cs-1);  // 동적 폐쇄
 for(const lk of (STATE.oneway||[])){                                                                  // 통로 방향(one-way) 화살표
  const [dx,dy]=lk.dir; for(const [x,y] of lk.cells){ const cx2=px(x),cy2=py(y),a=cs*0.32;
   g.strokeStyle='#a5d6ff'; g.lineWidth=Math.max(1,cs*0.10); g.beginPath();
   g.moveTo(cx2-dx*a,cy2-dy*a); g.lineTo(cx2+dx*a,cy2+dy*a);                                            // 몸통
   g.moveTo(cx2+dx*a,cy2+dy*a); g.lineTo(cx2+dx*a-(dx+dy)*a*0.5,cy2+dy*a-(dy-dx)*a*0.5);                // 촉1
   g.moveTo(cx2+dx*a,cy2+dy*a); g.lineTo(cx2+dx*a-(dx-dy)*a*0.5,cy2+dy*a-(dy+dx)*a*0.5); g.stroke(); }  // 촉2
 }
 for(const r of R){                                                                                    // 로봇 dot(색=우선순위) + 라벨
  const p=(cur&&cur[r.id])||r, X=px(p.x), Y=py(p.y), rad=Math.max(2,cs*0.46);
  let col;
  if(r.status==='down') col='#484f58';                                                                 // 고장=회색
  else if(r.task==null) col='#6e7681';                                                                 // 유휴(임무 없음)=회색
  else { const t=Math.min(1,(r.age||0)/60);                                                            // 0=방금(흰) .. 1=오래지연(빨강)
    col=`rgb(${255-Math.round(t*7)},${255-Math.round(t*174)},${255-Math.round(t*182)})`; }
  g.fillStyle=col; g.beginPath(); g.arc(X,Y,rad,0,7); g.fill();
  g.strokeStyle='#0b0e13'; g.lineWidth=1; g.stroke();                                                  // 경계선(인접 로봇 구분)
  if(r.carrying){ g.strokeStyle='#f0d060'; g.lineWidth=Math.max(2,cs*0.16); g.beginPath();             // 적재 중 = 굵은 금색 링(잘 보이게)
   g.arc(X,Y,rad+cs*0.12,0,7); g.stroke();
   g.fillStyle='#f0d060'; g.fillRect(X+rad*0.4,Y-rad*1.15,rad*0.62,rad*0.62); }                        // + 우상단 박스
  if(r.id===SEL){ g.strokeStyle='#58a6ff'; g.lineWidth=Math.max(2,cs*0.10); g.beginPath();             // 선택 로봇 강조 링
   g.arc(X,Y,rad+cs*0.26,0,7); g.stroke(); }
  if(LABELMODE!==2 && cs>=11){
   g.textAlign='center'; g.fillStyle='#0b0e13';
   if(LABELMODE===0){                                                                                  // 번호만(크게·가독)
    g.textBaseline='middle'; g.font=`bold ${Math.max(9,Math.floor(cs*0.64))}px ui-monospace,monospace`;
    g.fillText(r.id.replace(/\\D/g,''), X, Y);
   }else{                                                                                               // 번호+우선순위
    g.textBaseline='middle'; g.font=`bold ${Math.max(8,Math.floor(cs*0.44))}px ui-monospace,monospace`;
    g.fillText(r.id.replace(/\\D/g,''), X, Y-cs*0.11);
    g.textBaseline='alphabetic'; g.font=`${Math.max(7,Math.floor(cs*0.34))}px ui-monospace,monospace`;
    g.fillText('p'+(r.pr??''), X, Y+rad*0.98);
   }
  }
  if(r.status==='down'){ g.strokeStyle='#fff'; g.lineWidth=1.5; const d=rad*0.6; g.beginPath();
   g.moveTo(X-d,Y-d); g.lineTo(X+d,Y+d); g.moveTo(X+d,Y-d); g.lineTo(X-d,Y+d); g.stroke(); }
 }
}
function drawSpark(){                                    // 처리량(초록)·백로그(노랑) 시계열 — 면적 채움
 const c=document.getElementById('spark'), g=c.getContext('2d'), W=c.width, H=c.height, P=5;
 g.clearRect(0,0,W,H); if(SPARK.length<2) return;
 const th=SPARK.map(s=>s[0]), bl=SPARK.map(s=>s[1]), mxT=Math.max(...th,0.01), mxB=Math.max(...bl,1);
 const draw=(arr,mx,col,fill)=>{ const pts=arr.map((v,i)=>[i/(arr.length-1)*W, H-P-(v/mx)*(H-2*P)]);
  g.beginPath(); pts.forEach(([x,y],i)=> i?g.lineTo(x,y):g.moveTo(x,y));
  g.strokeStyle=col; g.lineWidth=2; g.stroke();
  g.lineTo(W,H); g.lineTo(0,H); g.closePath(); g.fillStyle=fill; g.fill(); };
 draw(bl,mxB,'#e3b341','rgba(227,179,65,.10)');       // 백로그
 draw(th,mxT,'#3fb950','rgba(63,185,80,.15)');        // 처리량
}
document.getElementById('viz').addEventListener('click', ev=>{   // 로봇 클릭 → 드릴다운 선택
 if(!MAP) return; const c=ev.currentTarget, rect=c.getBoundingClientRect();
 const cs=Math.max(4,Math.floor(Math.min(c.width/MAP.width, c.height/MAP.height)));
 const mx=(ev.clientX-rect.left)/rect.width*c.width, my=(ev.clientY-rect.top)/rect.height*c.height;
 let best=null,bd=1e9; for(const r of (STATE.robots||[])){ const dx=(r.x*cs+cs/2)-mx,dy=(r.y*cs+cs/2)-my,d=dx*dx+dy*dy; if(d<bd){bd=d;best=r;} }
 SEL = (best && bd < cs*cs*4) ? best.id : null;          // 근처 로봇 선택(멀면 해제)
});
loadMap().then(()=>{ setInterval(poll,180); poll(); animate(); });   // poll=데이터(180ms), animate=부드러운 렌더(60fps)
</script>
"""
