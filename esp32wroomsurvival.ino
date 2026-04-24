/*
  RescueBot ESP32 - Single-file program
  Features:
   - AP hotspot + web UI (manual "hold-to-move", speed slider)
   - Auto mode (ultrasonic obstacle avoid)
   - Auto-Finder (uses MQ-2 smoke, MQ-135, IR flame, ultrasonic)
   - Separate ENA & ENB PWM (two LEDC channels)
   - Live sensors JSON endpoint for AJAX updates
   - Pins chosen to avoid changing motor wiring
   - All used GPIOs <= 35
*/

/* ----------------- PIN MAP (final) -----------------
  Motors (unchanged):
   - ENA -> GPIO25 (PWM channel 0)
   - ENB -> GPIO12 (PWM channel 1)
   - IN1 -> GPIO26
   - IN2 -> GPIO27
   - IN3 -> GPIO14
   - IN4 -> GPIO13

  Ultrasonic (moved off ADC pins):
   - TRIG -> GPIO19
   - ECHO -> GPIO18

  ADC sensors:
   - MQ-2 (smoke) -> GPIO32 (ADC)
   - MQ-135 (air quality) -> GPIO34 (ADC)
   - MIC (analog) -> GPIO35 (ADC)

  IR flame (digital):
   - FLAME_DO -> GPIO23

  I2C MLX90614:
   - SDA -> GPIO21
   - SCL -> GPIO22

  Status LED:
   - LED -> GPIO2
----------------------------------------------------*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <ArduinoJson.h>
#include <math.h>

// ===== CONFIG =====
// If your IR flame module outputs LOW when flame is detected set this true.
// If it outputs HIGH on detection set false. Tweak if detection logic seems inverted.
#define FLAME_ACTIVE_LOW true

// ===== USER CONFIG =====
const char* AP_SSID = "Surveillance RescueBot_AP";
const char* AP_PASS = "12345678";

const float SI_ALERT_THRESHOLD = 70.0;     // Survivability Index percent
const float GAS_ALERT_V = 2.0;             // MQ-135 threshold (voltage)
const float MQ2_ALERT_V = 1.6;             // MQ-2 threshold for smoke (0..3.3V) -- tune to your sensor
const float MIC_ALERT_PERCENT = 70.0;
const float TEMP_ALERT_C = 45.0;
const float AVOID_DISTANCE_CM = 20.0;      // ultrasonic avoid threshold (cm)
const unsigned long SENSOR_INTERVAL_MS = 1000UL; // update sensors every 1s

// ===== PIN MAP =====
// Motors
#define PIN_ENA    25   // PWM channel 0
#define PIN_ENB    12   // PWM channel 1
#define PIN_IN1    26
#define PIN_IN2    27
#define PIN_IN3    14
#define PIN_IN4    13

// Ultrasonic
#define PIN_TRIG   19
#define PIN_ECHO   18

// Analog sensors (ADC)
#define PIN_MQ2    32  // MQ-2 smoke (ADC1)
#define PIN_MQ135  34  // MQ-135 (ADC1)
#define PIN_MIC    35  // Microphone analog (ADC1)

// IR flame digital
#define PIN_FLAME  23  // IR flame digital output (DO)

// I2C MLX
#define PIN_SDA    21
#define PIN_SCL    22

#define PIN_LED    2   // status LED

// ===== Globals =====
WebServer server(80);
Adafruit_MLX90614 mlx;

unsigned long lastSensorMillis = 0;
float lastDistance = 999.0;
float lastMQ2V = 0.0;
float lastMQ135V = 0.0;
float lastTempC = -1000.0;
float lastMicLevel = 0.0;
float lastSI = 0.0;
bool autoMode = false;
bool autoFinderMode = false;
bool alerted = false;
bool flameDetected = false;

// motor PWM (LEDC)
const int LEDC_CHANNEL_A = 0;
const int LEDC_CHANNEL_B = 1;
const int LEDC_FREQ = 20000;
const int LEDC_RES = 8; // 8-bit => 0..255
int pwmA = 180; // 0..255
int pwmB = 180;

// Forward declarations
void handleRoot();
void handleCmd();
void handleSpeed();
void handleMode();
void handleModeFinder();
void handleSensors();
void startAP();

// motor helpers
void stopMotors();
void setMotorsForward();
void setMotorsBackward();
void setMotorsLeft();
void setMotorsRight();

// sensors
float readUltrasonicCM();
float readMQ2Voltage();
float readMQ135Voltage();
float readMicRMS();
float readMLXTempC();
float computeSI(float gasV, float micLevel, float distCm, float tempC);
bool readFlameDO();

// auto-finder helpers
void autoFinderBehavior();

void setup(){
  Serial.begin(115200);
  delay(50);
  Serial.println("\n--- RescueBot ESP32 (AP mode) starting ---");

  // motor pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // ultrasonic
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // flame
  pinMode(PIN_FLAME, INPUT);

  // LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // ADC attenuation (make 0..3.3V range)
  analogSetPinAttenuation(PIN_MQ2, ADC_11db);
  analogSetPinAttenuation(PIN_MQ135, ADC_11db);
  analogSetPinAttenuation(PIN_MIC, ADC_11db);

  // setup PWM LEDC for two channels
  ledcSetup(LEDC_CHANNEL_A, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PIN_ENA, LEDC_CHANNEL_A);
  ledcWrite(LEDC_CHANNEL_A, pwmA);

  ledcSetup(LEDC_CHANNEL_B, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PIN_ENB, LEDC_CHANNEL_B);
  ledcWrite(LEDC_CHANNEL_B, pwmB);

  // I2C for MLX90614
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!mlx.begin()) {
    Serial.println("Warning: MLX90614 not found. Check wiring.");
  } else {
    Serial.println("MLX90614 OK");
  }

  // Start AP (onboard hotspot)
  startAP();

  // Web routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCmd);       // ?act=forward/backward/left/right/stop
  server.on("/speed", HTTP_GET, handleSpeed);   // ?value=0..255 (applies to both ENA & ENB)
  server.on("/mode", HTTP_GET, handleMode);     // toggles auto/manual
  server.on("/finder", HTTP_GET, handleModeFinder); // toggles auto-finder mode
  server.on("/sensors", HTTP_GET, handleSensors);// returns JSON of sensors

  server.begin();
  Serial.println("HTTP server started");

  // initial motor outputs
  stopMotors();
}

// AP startup
void startAP(){
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP started. Connect to SSID: ");
  Serial.print(AP_SSID);
  Serial.print("  Password: ");
  Serial.println(AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(ip.toString());
}

void loop(){
  server.handleClient();

  unsigned long now = millis();
  if (now - lastSensorMillis >= SENSOR_INTERVAL_MS) {
    lastSensorMillis = now;

    lastDistance = readUltrasonicCM();
    lastMQ2V = readMQ2Voltage();
    lastMQ135V = readMQ135Voltage();
    lastTempC = readMLXTempC();
    lastMicLevel = readMicRMS();
    lastSI = computeSI(lastMQ135V, lastMicLevel, lastDistance, lastTempC);
    flameDetected = readFlameDO();

    Serial.printf("SI: %.1f  Dist: %.1f cm  MQ2: %.2fV  MQ135: %.2fV  Temp: %.2f  Mic: %.1f  Flame:%d\n",
                  lastSI, lastDistance, lastMQ2V, lastMQ135V, lastTempC, lastMicLevel, flameDetected ? 1 : 0);

    // flame immediate stop & alert
    if (flameDetected) {
      stopMotors();
      autoMode = false;
      autoFinderMode = false;
      alerted = true;
      digitalWrite(PIN_LED, HIGH);
      Serial.println("FLAME DETECTED: STOPPING");
    } else {
      // alert logic (combine SI, gas, mic, distance, MQ2)
      bool currentAlert = (lastSI >= SI_ALERT_THRESHOLD ||
                           lastDistance < AVOID_DISTANCE_CM ||
                           lastMQ135V > GAS_ALERT_V ||
                           lastMicLevel >= MIC_ALERT_PERCENT ||
                           lastTempC >= TEMP_ALERT_C ||
                           lastMQ2V >= MQ2_ALERT_V);
      if (currentAlert && !alerted) {
        alerted = true;
        digitalWrite(PIN_LED, HIGH);
        Serial.println("ALERT ON");
      } else if (!currentAlert && alerted) {
        alerted = false;
        digitalWrite(PIN_LED, LOW);
        Serial.println("ALERT OFF");
      }
    }

    // auto mode behavior: obstacle avoid
    if (autoMode && !flameDetected) {
      if (lastDistance < AVOID_DISTANCE_CM) {
        // avoid: stop, back up, turn, resume
        stopMotors(); delay(120);
        setMotorsBackward(); delay(400);
        setMotorsRight(); delay(350);
        stopMotors(); delay(120);
      } else {
        setMotorsForward();
      }
    }

    // auto-finder (smoke search & move) - only if enabled and no flame
    if (autoFinderMode && !flameDetected) {
      autoFinderBehavior();
    }
  }
}

// Web handlers
void handleRoot(){
  String page = R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Surveillance RescueBot</title>
<style>
:root{--bg:#071021;--card:#0f1720;--accent:#19d3da;--muted:#94a3b8}
body{margin:0;font-family:Inter,Arial;background:linear-gradient(180deg,#021021,#071021);color:#e6f0f3;padding:12px}
.header{display:flex;align-items:center;justify-content:space-between}
.h1{font-size:20px;font-weight:700}
.card{background:var(--card);padding:12px;border-radius:10px;margin-top:12px;box-shadow:0 6px 20px rgba(2,6,23,0.6)}
.grid{display:grid;grid-template-columns:1fr 320px;gap:12px}
.controls button{padding:10px 12px;border-radius:8px;border:none;background:#0b2530;color:#fff;margin:6px;cursor:pointer}
.joy{display:grid;grid-template-columns:60px 60px 60px;grid-template-rows:60px 60px 60px;gap:6px;justify-content:center}
.joy button{width:60px;height:60px;border-radius:8px;border:none;background:#07202b;color:#fff;font-size:18px}
.small{font-size:13px;color:var(--muted)}
.row{display:flex;justify-content:space-between;align-items:center;padding:6px 0}
.bar{height:12px;background:#07202b;border-radius:8px;overflow:hidden}
.fill{height:100%;width:0;background:linear-gradient(90deg,var(--accent),#2dd4bf)}
.badge{padding:6px 8px;border-radius:8px;background:#051a23;color:#9ff}
.switch{padding:8px 10px;border-radius:8px;background:#02151a;display:inline-block;cursor:pointer}
.footer{font-size:12px;color:var(--muted);margin-top:10px;text-align:center}
</style>
</head>
<body>
<div class="header">
  <div class="h1">Surveillance RescueBot</div>
  <div class="small">AP: <strong>)rawliteral";
  page += WiFi.softAPIP().toString();
  page += R"rawliteral(</strong></div>
</div>

<div class="grid">
  <div>
    <div class="card">
      <div style="display:flex;justify-content:space-between;align-items:center">
        <div>
          <div class="small">Mode</div>
          <div id="modeTxt"><strong>MANUAL</strong></div>
        </div>
        <div>
          <button id="toggleMode" class="switch">Toggle Auto</button>
          <button id="toggleFinder" class="switch">Toggle Finder</button>
        </div>
      </div>
    </div>

    <div class="card">
      <h3 style="margin:0 0 8px 0">Manual control (hold to move)</h3>
      <div class="joy">
        <div></div><button id="fwd">F</button><div></div>
        <button id="left">L</button><button id="stop">S</button><button id="right">R</button>
        <div></div><button id="back">B</button><div></div>
      </div>
      <div style="text-align:center;margin-top:8px">
        Speed: <span id="speedVal">180</span><br>
        <input id="speed" type="range" min="0" max="255" value="180" style="width:90%">
      </div>
    </div>

    <div class="card">
      <h3 style="margin:0 0 8px 0">Auto-Finder status</h3>
      <div class="row"><div>Finder</div><div id="finderState">OFF</div></div>
      <div class="row"><div>Flame</div><div id="flame">NO</div></div>
      <div class="row"><div>Smoke (MQ-2)</div><div id="mq2v">-</div></div>
      <div class="bar" style="margin-top:6px"><div id="mq2fill" class="fill"></div></div>
    </div>
  </div>

  <div>
    <div class="card">
      <h3 style="margin:0 0 8px 0">Live sensors</h3>
      <div class="row"><div>Survivability Index</div><div id="si">0</div></div>
      <div class="row"><div>Distance (cm)</div><div id="distance">-</div></div>
      <div class="row"><div>MQ-135 (V)</div><div id="mq135">-</div></div>
      <div class="row"><div>Temp (°C)</div><div id="temp">-</div></div>
      <div class="row"><div>Mic level</div><div id="mic">-</div></div>
      <div style="margin-top:8px" class="small">Alerts lighted when thresholds reached</div>
    </div>

    <div class="card">
      <h3 style="margin:0 0 8px 0">Quick actions</h3>
      <div style="display:flex;gap:8px;flex-wrap:wrap">
        <button onclick="fetch('/cmd?act=stop')">Stop</button>
        <button onclick="fetch('/cmd?act=forward')">Forward</button>
        <button onclick="fetch('/cmd?act=backward')">Backward</button>
        <button onclick="fetch('/cmd?act=left')">Left</button>
        <button onclick="fetch('/cmd?act=right')">Right</button>
      </div>
    </div>

    <div class="card" style="text-align:center">
      <div class="small">Status LED</div>
      <div id="ledBadge" class="badge">OK</div>
    </div>
  </div>
</div>

<div class="footer">Surveillance RescueBot    Auto / Finder / Manual controls</div>

<script>
const sendCmd = (c)=> fetch('/cmd?act='+c).catch(()=>{});
const holdButton = (id, cmd) => {
  const el = document.getElementById(id);
  function start(){ sendCmd(cmd); }
  function stop(){ sendCmd('stop'); }
  el.addEventListener('mousedown', start);
  el.addEventListener('touchstart', (e)=>{ e.preventDefault(); start(); });
  el.addEventListener('mouseup', stop);
  el.addEventListener('mouseleave', stop);
  el.addEventListener('touchend', stop);
};
holdButton('fwd','forward');
holdButton('back','backward');
holdButton('left','left');
holdButton('right','right');
document.getElementById('stop').addEventListener('click', ()=> sendCmd('stop'));

document.getElementById('toggleMode').addEventListener('click', async ()=>{
  const r = await fetch('/mode'); const text = await r.text();
  document.getElementById('modeTxt').innerHTML = '<strong>'+text+'</strong>';
});
document.getElementById('toggleFinder').addEventListener('click', async ()=>{
  const r = await fetch('/finder'); const text = await r.text();
  document.getElementById('finderState').innerText = text;
});

const speed = document.getElementById('speed');
const speedVal = document.getElementById('speedVal');
speed.addEventListener('input', async (e)=>{
  const v = e.target.value;
  speedVal.innerText = v;
  await fetch('/speed?value='+v).catch(()=>{});
});

async function updateSensors(){
  try {
    const r = await fetch('/sensors');
    const j = await r.json();
    document.getElementById('si').innerText = j.si.toFixed(1);
    document.getElementById('distance').innerText = j.distance.toFixed(1);
    document.getElementById('mq135').innerText = j.gasV.toFixed(2);
    document.getElementById('temp').innerText = j.temp.toFixed(2);
    document.getElementById('mic').innerText = j.mic.toFixed(1);
    document.getElementById('mq2v').innerText = j.mq2.toFixed(2) + ' V';
    document.getElementById('mq2fill').style.width = Math.min(100, (j.mq2/3.3*100)).toFixed(1) + '%';
    document.getElementById('flame').innerText = j.flame==1 ? 'YES' : 'NO';
    document.getElementById('finderState').innerText = j.finder==1 ? 'ON' : 'OFF';
    document.getElementById('modeTxt').innerHTML = '<strong>' + (j.auto==1 ? 'AUTO' : 'MANUAL') + '</strong>';
    document.getElementById('ledBadge').innerText = j.alert==1 ? 'ALERT' : 'OK';
    if (j.alert==1) document.getElementById('ledBadge').style.background = '#7a0';
    else document.getElementById('ledBadge').style.background = '#051a23';
  } catch(e){}
  setTimeout(updateSensors, 1000);
}
updateSensors();
</script>

</body>
</html>
)rawliteral";

  server.send(200, "text/html", page);
}

// /cmd?act=forward/backward/left/right/stop
void handleCmd(){
  if (!server.hasArg("act")) { server.send(400, "text/plain", "Missing act"); return; }
  String act = server.arg("act");
  if (act == "forward") setMotorsForward();
  else if (act == "backward") setMotorsBackward();
  else if (act == "left") setMotorsLeft();
  else if (act == "right") setMotorsRight();
  else stopMotors();
  server.send(200, "text/plain", "OK");
}

// /speed?value=0..255
void handleSpeed(){
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  int v = server.arg("value").toInt();
  if (v < 0) v = 0; if (v > 255) v = 255;
  pwmA = v; pwmB = v;
  ledcWrite(LEDC_CHANNEL_A, pwmA);
  ledcWrite(LEDC_CHANNEL_B, pwmB);
  server.send(200, "text/plain", "OK");
}

// toggles auto/manual and returns new mode text
void handleMode(){
  autoMode = !autoMode;
  if (autoMode) autoFinderMode = false; // don't run finder and normal auto together
  server.send(200, "text/plain", autoMode ? "AUTO" : "MANUAL");
}

// toggles auto-finder mode
void handleModeFinder(){
  autoFinderMode = !autoFinderMode;
  if (autoFinderMode) autoMode = false; // exclusive modes
  server.send(200, "text/plain", autoFinderMode ? "ON" : "OFF");
}

// /sensors -> JSON
void handleSensors(){
  StaticJsonDocument<384> doc;
  doc["si"] = lastSI;
  doc["distance"] = lastDistance;
  doc["mq2"] = lastMQ2V;
  doc["gasV"] = lastMQ135V;
  doc["temp"] = lastTempC;
  doc["mic"] = lastMicLevel;
  doc["auto"] = autoMode ? 1 : 0;
  doc["finder"] = autoFinderMode ? 1 : 0;
  doc["alert"] = alerted ? 1 : 0;
  doc["flame"] = flameDetected ? 1 : 0;
  String s; serializeJson(doc, s);
  server.send(200, "application/json", s);
}

// Motor implementations
void setMotorsForward(){
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  ledcWrite(LEDC_CHANNEL_A, pwmA);
  ledcWrite(LEDC_CHANNEL_B, pwmB);
}
void setMotorsBackward(){
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  ledcWrite(LEDC_CHANNEL_A, pwmA);
  ledcWrite(LEDC_CHANNEL_B, pwmB);
}
void setMotorsLeft(){
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  ledcWrite(LEDC_CHANNEL_A, pwmA);
  ledcWrite(LEDC_CHANNEL_B, pwmB);
}
void setMotorsRight(){
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  ledcWrite(LEDC_CHANNEL_A, pwmA);
  ledcWrite(LEDC_CHANNEL_B, pwmB);
}
void stopMotors(){
  ledcWrite(LEDC_CHANNEL_A, 0);
  ledcWrite(LEDC_CHANNEL_B, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

// Sensors
float readUltrasonicCM(){
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 30000);
  if (duration == 0) return 999.0;
  float dist = (duration * 0.0343) / 2.0;
  return dist;
}

float readMQ2Voltage(){
  int raw = analogRead(PIN_MQ2); // ADC1 0..4095
  float v = (raw / 4095.0) * 3.3;
  return v;
}

float readMQ135Voltage(){
  int raw = analogRead(PIN_MQ135);
  float v = (raw / 4095.0) * 3.3;
  return v;
}

float readMicRMS(){
  unsigned long start = millis();
  unsigned long endt = start + 30;
  uint32_t sumSq = 0;
  uint32_t sum = 0;
  uint32_t cnt = 0;
  while (millis() < endt) {
    int a = analogRead(PIN_MIC);
    sum += a;
    sumSq += (uint32_t)a * (uint32_t)a;
    cnt++;
    delayMicroseconds(200);
  }
  if (cnt == 0) return 0;
  float mean = (float)sum / cnt;
  float meanSq = (float)sumSq / cnt;
  float var = meanSq - mean * mean;
  if (var < 0) var = 0;
  float rms = sqrt(var);
  float normalized = (rms / 4095.0) * 100.0 * 2.0;
  if (normalized > 100.0) normalized = 100.0;
  return normalized;
}

float readMLXTempC(){
  float t = mlx.readObjectTempC();
  if (isnan(t)) return -1000.0;
  return t;
}

bool readFlameDO(){
  int v = digitalRead(PIN_FLAME);
#if FLAME_ACTIVE_LOW
  return (v == LOW);
#else
  return (v == HIGH);
#endif
}

float computeSI(float gasV, float micLevel, float distCm, float tempC){
  float gasNorm = constrain(1.0 - (gasV / 3.3), 0.0, 1.0);
  float micNorm = constrain(micLevel / 100.0, 0.0, 1.0);
  float distNorm = distCm < 100.0 ? constrain((100.0 - distCm) / 100.0, 0.0, 1.0) : 0.0;
  float tempNorm = (tempC < -100.0) ? 0.0 : constrain((tempC - 20.0) / 30.0, 0.0, 1.0);

  const float w_gas = 0.25;
  const float w_mic = 0.35;
  const float w_dist = 0.30;
  const float w_temp = 0.10;

  float idx = (w_gas * gasNorm + w_mic * micNorm + w_dist * distNorm + w_temp * tempNorm) * 100.0;
  return idx;
}

/* Simple auto-finder behavior:
   - If MQ-2 reading above threshold, robot will try to find direction of stronger smoke:
     rotate left while sampling; if reading increases, face that direction and move forward.
   - Basic and conservative: avoids obstacles using ultrasonic and stops on flame.
*/
void autoFinderBehavior(){
  // if no significant smoke, just forward slowly (and avoid)
  if (lastMQ2V < MQ2_ALERT_V) {
    if (lastDistance > AVOID_DISTANCE_CM) setMotorsForward();
    else { stopMotors(); setMotorsRight(); delay(300); stopMotors(); }
    return;
  }

  // smoky: search for stronger signal by rotating and sampling
  Serial.println("Finder: smoke detected -> searching");
  stopMotors();
  delay(80);

  float base = lastMQ2V;
  float bestVal = base;
  int bestDir = 0; // 0: none, 1: left, 2: right

  // rotate left in small steps and sample
  for (int i = 0; i < 5; ++i) {
    setMotorsLeft();
    ledcWrite(LEDC_CHANNEL_A, 120);
    ledcWrite(LEDC_CHANNEL_B, 120);
    delay(200); // rotate a bit
    stopMotors(); delay(80);
    float v = readMQ2Voltage();
    Serial.printf("Finder left sample %d -> %.2f\n", i, v);
    if (v > bestVal + 0.02) { bestVal = v; bestDir = 1; break; }
  }

  if (bestDir == 0) {
    // rotate right a bit and sample
    for (int i = 0; i < 6; ++i) {
      setMotorsRight();
      ledcWrite(LEDC_CHANNEL_A, 120);
      ledcWrite(LEDC_CHANNEL_B, 120);
      delay(200);
      stopMotors(); delay(80);
      float v = readMQ2Voltage();
      Serial.printf("Finder right sample %d -> %.2f\n", i, v);
      if (v > bestVal + 0.02) { bestVal = v; bestDir = 2; break; }
    }
  }

  // Move toward best direction
  if (bestDir == 1) {
    Serial.println("Finder: moving left-forward toward smoke");
    setMotorsLeft(); ledcWrite(LEDC_CHANNEL_A, 160); ledcWrite(LEDC_CHANNEL_B, 160);
    delay(450);
    stopMotors();
  } else if (bestDir == 2) {
    Serial.println("Finder: moving right-forward toward smoke");
    setMotorsRight(); ledcWrite(LEDC_CHANNEL_A, 160); ledcWrite(LEDC_CHANNEL_B, 160);
    delay(450);
    stopMotors();
  } else {
    // No direction found: move forward slowly (will re-evaluate)
    Serial.println("Finder: no direction stronger -> forward probe");
    setMotorsForward(); ledcWrite(LEDC_CHANNEL_A, 140); ledcWrite(LEDC_CHANNEL_B, 140);
    delay(600);
    stopMotors();
  }
}
