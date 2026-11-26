// web_ui_balance_turn.h  -  The Ultra Robot: WiFi tuning page for the self-balancing robot
// USE_AP 1 : the ESP32 makes its own WiFi. Phone joins "UltraRobot", then open http://192.168.4.1
// USE_AP 0 : the ESP32 joins your home WiFi. The IP is printed in Serial Monitor at 115200 baud.

#pragma once
#include <WiFi.h>
#include <WebServer.h>

#define USE_AP 1

const char* AP_SSID  = "UltraRobot";
const char* AP_PASS  = "12345678";        // at least 8 characters
const char* STA_SSID = "YOUR_WIFI_NAME";  // only used when USE_AP is 0
const char* STA_PASS = "YOUR_WIFI_PASSWORD";

WebServer server(80);

// ---- small command queue: the web task pushes, handleSerial() pops ----
volatile char    wq[16];
volatile uint8_t wqH = 0, wqT = 0;
bool webCmdAvailable() { return wqH != wqT; }
char webCmdPop() { char c = wq[wqT]; wqT = (wqT + 1) % 16; return c; }
void webCmdPush(char c) {
  uint8_t n = (wqH + 1) % 16;
  if (n == wqT) return;                   // queue full -> drop
  wq[wqH] = c; wqH = n;
}

// ---- page (red + black, phone friendly, no internet needed) ----
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>The Ultra Robot</title>
<style>
:root{--red:#e10600;--hot:#ff3b30;--panel:#140709;--line:#3a0f13;--dim:#a88a8c}
*{box-sizing:border-box}
body{margin:0;background:#000;color:#f4eaea;font-family:'Arial Narrow',Arial,sans-serif;padding:0 0 40px}
header{padding:20px 16px 14px;border-bottom:2px solid var(--red);display:flex;justify-content:space-between;align-items:center;gap:10px}
h1{margin:0;font-size:2rem;font-style:italic;line-height:1}h1 span{color:var(--red)}
.dot{width:12px;height:12px;border-radius:50%;background:#555}.dot.on{background:var(--hot);box-shadow:0 0 10px var(--hot)}
main{padding:14px 16px;display:grid;gap:14px;max-width:720px;margin:auto}
.card{background:var(--panel);border:1px solid var(--line);border-left:5px solid var(--red);padding:14px}
.card h2{margin:0 0 4px;font-size:1.2rem}
.top{display:flex;justify-content:space-between;align-items:baseline}
.live{font-size:2rem;font-weight:700;color:var(--hot)}
.dim{color:var(--dim);font-size:.85rem}
input[type=range]{width:100%;accent-color:var(--red);height:36px}
.val{font-size:1.1rem}
button{font:inherit;font-weight:700;color:#fff;background:transparent;border:2px solid var(--red);padding:12px 14px;border-radius:0}
button:active{background:var(--red)}button.solid{background:var(--red);width:100%;font-size:1.1rem}
button:disabled{opacity:.45}
#runBtn{font-size:1.5rem;padding:18px}
#runBtn.stop{background:transparent}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;text-align:center}
.stats b{display:block;font-size:1.5rem}
#fwdBtn,#backBtn,#leftBtn,#rightBtn{touch-action:none;user-select:none;-webkit-user-select:none;padding:22px;font-size:1.3rem}
canvas{width:100%;height:150px;background:#000;border:1px solid var(--line);display:block}
</style></head><body>
<header><h1>The <span>Ultra</span> Robot</h1><div class="dot" id="dot"></div></header>
<main>
 <div class="card">
  <button class="solid" id="runBtn" onclick="toggleRun()">Start robot</button>
  <div class="dim" id="runMsg" style="margin-top:8px">Connecting...</div>
 </div>

 <div class="card"><div class="stats">
  <div><b id="sA">-</b><span class="dim">Angle</span></div>
  <div><b id="sO">-</b><span class="dim">Output</span></div>
  <div><b id="sR">-</b><span class="dim">Motors</span></div></div>
  <canvas id="ch" width="600" height="150" style="margin-top:10px"></canvas>
  <div class="dim" id="why" style="margin-top:8px"></div></div>

 <div class="card"><div class="top"><h2>Kp</h2><div class="live" id="lKp">-</div></div>
  <div class="dim">Proportional: push-back against tilt</div>
  <input type="range" id="kp" min="0" max="100" step="1"><div class="val">Target: <span id="vKp"></span> &nbsp;<span class="dim">In use: <span id="eKp">-</span></span></div></div>
 <div class="card"><div class="top"><h2>Ki</h2><div class="live" id="lKi">-</div></div>
  <div class="dim">Integral: removes steady lean</div>
  <input type="range" id="ki" min="0" max="10" step="0.05"><div class="val">Target: <span id="vKi"></span> &nbsp;<span class="dim">In use: <span id="eKi">-</span></span></div></div>
 <div class="card"><div class="top"><h2>Kd</h2><div class="live" id="lKd">-</div></div>
  <div class="dim">Derivative: damps wobble</div>
  <input type="range" id="kd" min="0" max="5" step="0.01"><div class="val">Target: <span id="vKd"></span> &nbsp;<span class="dim">In use: <span id="eKd">-</span></span></div></div>
 <button class="solid" onclick="sendGains()">Send Kp, Ki, Kd to robot</button>

 <div class="card"><h2>Drive</h2><div class="dim" id="drvMsg">Start the robot first</div>
  <div class="grid" style="margin-top:8px">
   <button id="fwdBtn" class="solid" disabled>Forward</button>
   <button id="backBtn" class="solid" disabled>Back</button></div>
  <div class="grid" style="margin-top:8px">
   <button id="leftBtn" class="solid" disabled>&larr; Left</button>
   <button id="rightBtn" class="solid" disabled>Right &rarr;</button></div>
  <div class="val" style="margin-top:10px">Lean when driving: <span id="vLean"></span>&deg;</div>
  <input type="range" id="lean" min="0" max="8" step="0.5">
  <div class="val">Turn strength: <span id="vTstr"></span></div>
  <input type="range" id="tstr" min="0" max="120" step="5">
  <div class="val">Stop driving if FORWARD tilt passes: <span id="vAf"></span>&deg;</div>
  <input type="range" id="af" min="5" max="34" step="1">
  <div class="val">Stop driving if BACKWARD tilt passes: <span id="vAb"></span>&deg;</div>
  <input type="range" id="ab" min="5" max="34" step="1">
  <button class="solid" style="margin-top:8px" onclick="sendDrive()">Send drive settings</button></div>

 <div class="card"><div class="top"><h2>Fuzzy PID</h2><button id="fzBtn" onclick="toggleFz()" style="padding:8px 16px">-</button></div>
  <div class="dim">Auto-adjusts Kp, Ki, Kd around your base values while balancing</div>
  <div class="val" style="margin-top:8px">Kp range &plusmn;<span id="vFkp"></span></div>
  <input type="range" id="fkp" min="0" max="20" step="0.5">
  <div class="val">Ki range &plusmn;<span id="vFki"></span></div>
  <input type="range" id="fki" min="0" max="2" step="0.05">
  <div class="val">Kd range &plusmn;<span id="vFkd"></span></div>
  <input type="range" id="fkd" min="0" max="1" step="0.01">
  <button class="solid" style="margin-top:8px" onclick="sendFz()">Send fuzzy ranges</button></div>

 <div class="card"><h2>Setpoint <span class="live" id="lSet" style="font-size:1.3rem">-</span></h2>
  <div class="grid"><button onclick="cmd('f')">-0.1</button><button onclick="cmd('r')">+0.1</button></div></div>
 <div class="card"><h2>minPwm <span class="live" id="lMin" style="font-size:1.3rem">-</span></h2>
  <div class="grid"><button onclick="cmd('v')">-5</button><button onclick="cmd('c')">+5</button></div></div>
 <div class="card"><h2>Other</h2><div class="grid">
  <button onclick="cmd('z')">Set pose as 0&deg;</button><button onclick="cmd('o')">Auto-balance: <span id="lH">-</span></button>
  <button onclick="cmd('n')">K_SPD -0.1</button><button onclick="cmd('h')">K_SPD +0.1</button>
  <button onclick="cmd('m')">K_POS -0.05</button><button onclick="cmd('b')">K_POS +0.05</button>
  <button onclick="cmd('t')">Ramp limit -10</button><button onclick="cmd('y')">Ramp limit +10</button></div>
  <div class="dim" style="margin-top:8px">K_SPD <span id="lKs">-</span> &nbsp; K_POS <span id="lKpos">-</span> &nbsp; Ramp <span id="lSt">-</span></div></div>
</main>
<script>
const $=i=>document.getElementById(i);let first=true,st=0,fz=1;const H=[];
['fkp','fki','fkd'].forEach(k=>{$(k).oninput=()=>{$('v'+k[0].toUpperCase()+k.slice(1)).textContent=$(k).value}});
['lean','tstr','af','ab'].forEach(k=>{$(k).oninput=()=>{$('v'+k[0].toUpperCase()+k.slice(1)).textContent=$(k).value}});
let drv=0,drvTimer=null;
function setDrive(d){drv=d;fetch('/drive?d='+d)}
function bindDrive(id,d){const b=$(id);
 const down=e=>{e.preventDefault();if(b.disabled)return;setDrive(d);clearInterval(drvTimer);drvTimer=setInterval(()=>fetch('/drive?d='+d),150)};
 const up=()=>{clearInterval(drvTimer);if(drv!==0)setDrive(0)};
 b.addEventListener('pointerdown',down);
 ['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,up));
 b.addEventListener('contextmenu',e=>e.preventDefault())}
bindDrive('fwdBtn',1);bindDrive('backBtn',-1);
let trn=0,trnTimer=null;
function setTurn(d){trn=d;fetch('/turn?d='+d)}
function bindTurn(id,d){const b=$(id);
 const down=e=>{e.preventDefault();if(b.disabled)return;setTurn(d);clearInterval(trnTimer);trnTimer=setInterval(()=>fetch('/turn?d='+d),150)};
 const up=()=>{clearInterval(trnTimer);if(trn!==0)setTurn(0)};
 b.addEventListener('pointerdown',down);
 ['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,up));
 b.addEventListener('contextmenu',e=>e.preventDefault())}
bindTurn('leftBtn',-1);bindTurn('rightBtn',1);
['kp','ki','kd'].forEach(k=>{$(k).oninput=()=>{$('v'+k[0].toUpperCase()+k[1]).textContent=$(k).value}});
function draw(){const c=$('ch'),g=c.getContext('2d'),w=c.width,h=c.height;g.clearRect(0,0,w,h);
 g.strokeStyle='#3a0f13';g.beginPath();g.moveTo(0,h/2);g.lineTo(w,h/2);g.stroke();
 g.strokeStyle='#ff3b30';g.lineWidth=2.5;g.beginPath();
 H.forEach((v,i)=>{const x=i/119*w,y=h/2-Math.max(-1,Math.min(1,v/35))*(h/2-4);i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke()}
function showState(){
 const b=$('runBtn'),m=$('runMsg');
 if(st===0){b.textContent='Start robot';b.className='solid';b.disabled=false;
  m.textContent='Idle. Hold the robot upright, press Start, and keep holding it still for a few seconds.'}
 else if(st===1){b.textContent='Finding upright angle...';b.disabled=true;
  m.textContent='Hold the robot upright and still. This angle becomes 0 degrees.'}
 else{b.textContent='Stop robot';b.className='solid stop';b.disabled=false;
  m.textContent='Release the robot once it pushes back. When Auto-balance shows ON, Forward and Back unlock.'}}
async function poll(){try{
 const s=await (await fetch('/state')).json();
 $('dot').className='dot on';
 if(s.state!==st&&s.state>0)first=true;
 st=s.state;showState();
 $('lKp').textContent=s.kp.toFixed(1);$('lKi').textContent=s.ki.toFixed(2);$('lKd').textContent=s.kd.toFixed(2);
 $('lSet').textContent=s.set.toFixed(1);$('lMin').textContent=s.minpwm;$('lSt').textContent=s.step;
 $('lKs').textContent=s.kspd.toFixed(2);$('lKpos').textContent=s.kpos.toFixed(2);
 $('lH').textContent=s.hold;
 fz=s.fz;$('fzBtn').textContent=fz?'ON':'OFF';
 const can=(s.state===2&&s.hold==='ON');
 $('fwdBtn').disabled=!can;$('backBtn').disabled=!can;$('leftBtn').disabled=!can;$('rightBtn').disabled=!can;
 if(!can&&trn!==0){clearInterval(trnTimer);setTurn(0)}
 if(!can&&drv!==0){clearInterval(drvTimer);setDrive(0)}
 $('drvMsg').textContent=can?'Hold a button to drive or turn. Release to stop.':(s.state!==2?'Start the robot first':(s.hold==='OFF'?'Turn Auto-balance ON to drive.':'Balancing first. Forward and Back unlock when Auto-balance shows ON (robot steady).'));
 $('eKp').textContent=s.kpe.toFixed(1);$('eKi').textContent=s.kie.toFixed(2);$('eKd').textContent=s.kde.toFixed(2);
 $('sA').textContent=s.angle.toFixed(1);$('sO').textContent=Math.round(s.out);$('sR').textContent=s.run?'Run':'Stop';
 const W=['Motors running','Press Start robot','Motors off: waiting for the robot to be upright (within \u00b1'+s.rs+'\u00b0)','Motors off: tilted past '+s.fl+'\u00b0. Stand it up within \u00b1'+s.rs+'\u00b0','Motors off: sensor error (check wiring and power)'];
 $('why').textContent=W[s.why]||'';
 H.push(s.angle);if(H.length>120)H.shift();draw();
 if(first){first=false;$('kp').value=s.kp;$('ki').value=s.ki;$('kd').value=s.kd;
  $('vKp').textContent=s.kp.toFixed(1);$('vKi').textContent=s.ki.toFixed(2);$('vKd').textContent=s.kd.toFixed(2);
  $('tstr').value=s.ts;$('vTstr').textContent=s.ts.toFixed(0);$('lean').value=s.lean;$('af').value=s.af;$('ab').value=s.ab;$('vLean').textContent=s.lean.toFixed(1);$('vAf').textContent=s.af.toFixed(0);$('vAb').textContent=s.ab.toFixed(0);
  $('fkp').value=s.fkp;$('fki').value=s.fki;$('fkd').value=s.fkd;
  $('vFkp').textContent=s.fkp.toFixed(1);$('vFki').textContent=s.fki.toFixed(2);$('vFkd').textContent=s.fkd.toFixed(2)}
}catch(e){$('dot').className='dot';$('runMsg').textContent='Lost connection to the robot.'}}
async function toggleRun(){await fetch(st===2?'/stop':'/start')}
async function sendGains(){await fetch('/set?kp='+$('kp').value+'&ki='+$('ki').value+'&kd='+$('kd').value)}
async function toggleFz(){await fetch('/fz?on='+(fz?0:1))}
async function sendFz(){await fetch('/fz?kp='+$('fkp').value+'&ki='+$('fki').value+'&kd='+$('fkd').value)}
async function sendDrive(){await fetch('/drive?lean='+$('lean').value+'&af='+$('af').value+'&ab='+$('ab').value+'&ts='+$('tstr').value)}
async function cmd(c){await fetch('/cmd?c='+c)}
setInterval(poll,300);poll();
</script></body></html>
)rawliteral";

// ---- handlers ----
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleState() {
  char b[1000];
  snprintf(b, sizeof(b),
    "{\"kp\":%.2f,\"ki\":%.3f,\"kd\":%.3f,\"set\":%.2f,\"minpwm\":%d,\"step\":%d,"
    "\"kspd\":%.2f,\"kpos\":%.2f,\"angle\":%.2f,\"gyro\":%.1f,\"out\":%.0f,\"run\":%d,"
    "\"state\":%d,\"fz\":%d,\"kpe\":%.2f,\"kie\":%.3f,\"kde\":%.3f,\"fkp\":%.1f,\"fki\":%.2f,\"fkd\":%.2f,\"lean\":%.1f,\"af\":%.0f,\"ab\":%.0f,\"drv\":%.1f,\"why\":%d,\"rs\":%.0f,\"fl\":%.0f,\"ts\":%.0f,\"hold\":\"%s\"}",
    Kp, Ki, Kd, setpoint, minPwm, pwmStepLimit, K_SPD, K_POS,
    angle, gyroRate, lastOutput, active ? 1 : 0,
    calibrating ? 1 : (armed ? 2 : 0),
    fuzzyOn ? 1 : 0, KpE, KiE, KdE, fzKpSpan, fzKiSpan, fzKdSpan,
    driveLean, driveLimFwd, driveLimBack, driveShiftF,
    (int)stopWhy, RESUME_ANGLE, FALL_ANGLE, turnStrength,
    holdWanted ? (holdOn ? "ON" : "wait") : "OFF");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

void handleSet() {
  if (server.hasArg("kp")) Kp = constrain(server.arg("kp").toFloat(), 0.0f, 200.0f);
  if (server.hasArg("ki")) Ki = constrain(server.arg("ki").toFloat(), 0.0f, 20.0f);
  if (server.hasArg("kd")) Kd = constrain(server.arg("kd").toFloat(), 0.0f, 10.0f);
  Serial.printf("WEB: Kp=%.2f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
  server.send(200, "text/plain", "ok");
}

void handleCmd() {
  if (server.hasArg("c") && server.arg("c").length() == 1) {
    char c = server.arg("c")[0];
    if (strchr("rfcvonhbmtyzu", c)) webCmdPush(c);   // only safe commands
  }
  server.send(200, "text/plain", "ok");
}

void handleFz() {
  if (server.hasArg("on")) fuzzyOn = server.arg("on").toInt() != 0;
  if (server.hasArg("kp")) fzKpSpan = constrain(server.arg("kp").toFloat(), 0.0f, 30.0f);
  if (server.hasArg("ki")) fzKiSpan = constrain(server.arg("ki").toFloat(), 0.0f, 3.0f);
  if (server.hasArg("kd")) fzKdSpan = constrain(server.arg("kd").toFloat(), 0.0f, 2.0f);
  Serial.printf("WEB: fuzzy=%d spanKp=%.1f spanKi=%.2f spanKd=%.2f\n", fuzzyOn ? 1 : 0, fzKpSpan, fzKiSpan, fzKdSpan);
  server.send(200, "text/plain", "ok");
}

void handleDrive() {
  if (server.hasArg("ts"))   turnStrength = constrain(server.arg("ts").toFloat(), 0.0f, 120.0f);
  if (server.hasArg("lean")) driveLean = constrain(server.arg("lean").toFloat(), 0.0f, 8.0f);
  if (server.hasArg("af"))   driveLimFwd  = constrain(server.arg("af").toFloat(), 5.0f, 34.0f);
  if (server.hasArg("ab"))   driveLimBack = constrain(server.arg("ab").toFloat(), 5.0f, 34.0f);
  if (server.hasArg("d")) {
    int d = server.arg("d").toInt();
    driveCmd = d > 0 ? 1 : (d < 0 ? -1 : 0);
    if (d == 0) driveLocked = false;     // tha nut -> mo khoa
    driveLastMs = millis();
  }
  server.send(200, "text/plain", "ok");
}

void handleTurn() {
  if (server.hasArg("d")) {
    int d = server.arg("d").toInt();
    turnCmd = d > 0 ? 1 : (d < 0 ? -1 : 0);
    turnLastMs = millis();
  }
  server.send(200, "text/plain", "ok");
}

void handleStart() {
  if (!armed && !calibrating) startReq = true;
  server.send(200, "text/plain", "ok");
}

void handleStop() {
  stopReq = true;
  server.send(200, "text/plain", "ok");
}

// ---- runs on core 0 so WiFi never slows the 200 Hz balance loop on core 1 ----
void webTask(void*) {
#if USE_AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("WiFi: join '")); Serial.print(AP_SSID);
  Serial.print(F("' then open http://")); Serial.println(WiFi.softAPIP());
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  while (WiFi.status() != WL_CONNECTED) vTaskDelay(pdMS_TO_TICKS(500));
  Serial.print(F("WiFi OK. Open http://")); Serial.println(WiFi.localIP());
#endif
  server.on("/", handleRoot);
  server.on("/state", handleState);
  server.on("/set", handleSet);
  server.on("/cmd", handleCmd);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/fz", handleFz);
  server.on("/drive", handleDrive);
  server.on("/turn", handleTurn);
  server.begin();
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void webSetup() {
  xTaskCreatePinnedToCore(webTask, "web", 8192, NULL, 1, NULL, 0);
}