/*
  Self-balancing robot: ESP32 + MPU6050 + L293D
  Arduino-ESP32 core 3.x (uses ledcAttach / ledcWrite(pin, duty))

  Live tuning over Serial Monitor (115200 baud, "Newline" line ending):
    p18      -> set Kp = 18
    i0.5     -> set Ki = 0.5
    d0.6     -> set Kd = 0.6
    s1.5     -> set balance setpoint (deg) = 1.5
    m40      -> set minimum motor PWM (deadband) = 40
    t        -> toggle telemetry (angle, output)
    ?        -> print current values

  WiFi tuning: put the laptop on the same 2.4 GHz WiFi, open the Serial Monitor,
  read the printed IP address (e.g. 192.168.1.50) and open http://<that IP> in a browser.
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ---------------- WIFI (2.4 GHz only - ESP32 does not support 5 GHz) ----------------
const char* WIFI_SSID = "WIFI_NAME";
const char* WIFI_PASS = "WIFI_PASSWORD";

// ---------------- PINS (change to match your wiring) ----------------
#define SDA_PIN 21
#define SCL_PIN 22

// Left motor = M1 (OUT1/OUT2), right motor = M2 (OUT3/OUT4)
#define M1_IN1 25   // IN1
#define M1_IN2 26   // IN2
#define M1_EN  33   // ENA (PWM)

#define M2_IN1 27   // IN3
#define M2_IN2 14   // IN4
#define M2_EN  32   // ENB (PWM)

// ---------------- CONFIG ----------------
#define MPU_ADDR      0x68
#define PWM_FREQ      10000
#define PWM_BITS      8
#define LOOP_US       5000      // 200 Hz control loop

#define ANGLE_SIGN    1.0f      // flip to -1 if angle increases when robot leans backward
#define GYRO_SIGN     1.0f      // flip to -1 if the angle lags/jumps back after a quick tilt (gyro sign wrong)
#define MOTOR_SIGN    1.0f      // flip to -1 if wheels drive the wrong way when leaning
#define M1_REVERSE    false     // flip individual motor wiring direction
#define M2_REVERSE    false

#define FALL_ANGLE    30.0f     // cut motors beyond this tilt
#define START_ANGLE   2.0f      // robot arms itself when within this of upright

// ---------------- PID (starting values - tune these) ----------------
float Kp = 18.0f;
float Ki = 0.0f;
float Kd = 0.6f;
float setpoint = 0.0f;          // deg, fine-tune to true balance point
float KpB = 18.0f;              // Kp used while leaning backward (angle < setpoint)
float KdB = 0.6f;               // Kd used while leaning backward
int   minPWM = 40;              // overcomes motor/L293D deadband
float integralLimit = 150.0f;

// ---------------- STATE ----------------
float gyroBias = 0.0f;
float angle = 0.0f;
float integral = 0.0f;
bool  armed = false;
bool  telemetry = false;
unsigned long lastMicros = 0;
unsigned long lastPrint = 0;
bool  enabled = true;           // Run/Stop from the web page
float lastOutput = 0;
WebServer server(80);
Preferences prefs;

// ---------------- MPU6050 ----------------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWrite(0x6B, 0x00);   // wake up
  mpuWrite(0x1A, 0x03);   // DLPF ~44 Hz
  mpuWrite(0x1B, 0x08);   // gyro +-500 dps  (65.5 LSB/dps)
  mpuWrite(0x1C, 0x00);   // accel +-2 g     (16384 LSB/g)
}

void mpuRead(float &accAngle, float &gyroRate) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();                       // temperature (skip)
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  (void)ay; (void)gx; (void)gz;

  // MPU mounted flat with its X axis pointing to the FRONT of the robot,
  // so forward/back tilt is rotation about Y: use ax/az for angle and gy for rate.
  // Leaning forward should give a POSITIVE angle (flip ANGLE_SIGN if not).
  accAngle = ANGLE_SIGN * atan2(-(float)ax, (float)az) * RAD_TO_DEG;
  gyroRate = GYRO_SIGN * (gy / 65.5f) - gyroBias;
}

void calibrateGyro() {
  long sum = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x45);   // GYRO_YOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(3);
  }
  gyroBias = GYRO_SIGN * (sum / (float)N) / 65.5f;
}

// ---------------- MOTORS ----------------
void driveMotor(int in1, int in2, int en, int speed, bool reverse) {
  if (reverse) speed = -speed;
  if (speed > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
  ledcWrite(en, abs(speed));
}

void setMotors(float out) {
  out *= MOTOR_SIGN;
  int pwm = 0;
  if (fabs(out) > 1.0f) {
    pwm = (int)constrain(fabs(out), 0, 255);
    pwm = map(pwm, 0, 255, minPWM, 255);            // skip the dead zone
    if (out < 0) pwm = -pwm;
  }
  driveMotor(M1_IN1, M1_IN2, M1_EN, pwm, M1_REVERSE);
  driveMotor(M2_IN1, M2_IN2, M2_EN, pwm, M2_REVERSE);
}

void stopMotors() { setMotors(0); }

// ---------------- SERIAL TUNING ----------------
void printParams() {
  Serial.printf("Kp=%.2f Ki=%.3f Kd=%.3f setpoint=%.2f minPWM=%d\n", Kp, Ki, Kd, setpoint, minPWM);
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'p': Kp = Serial.parseFloat(); break;
    case 'i': Ki = Serial.parseFloat(); integral = 0; break;
    case 'd': Kd = Serial.parseFloat(); break;
    case 's': setpoint = Serial.parseFloat(); break;
    case 'm': minPWM = (int)Serial.parseFloat(); break;
    case 't': telemetry = !telemetry; break;
    case '?': break;
    default: return;
  }
  printParams();
}

// ---------------- WEB TUNING PAGE ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ATOM PID</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;max-width:480px;margin:auto;padding:12px}
label{display:block;margin-top:14px}
input[type=number]{width:90px;font-size:18px;margin-left:6px}
input[type=range]{width:100%}
button{font-size:16px;padding:10px 14px;margin:12px 6px 0 0}
canvas{width:100%;height:120px;background:#000;margin-top:8px}
</style></head><body>
<h2>ATOM balance tuning</h2>
<div>Angle: <b id="a">-</b>&deg; &nbsp; Output: <b id="o">-</b> &nbsp; <b id="st">-</b></div>
<canvas id="c" width="480" height="120"></canvas>
<div id="ctl"></div>
<button onclick="apply()">Apply</button>
<button onclick="save()">Save to flash</button>
<button onclick="toggle()">Run / Stop</button>
<script>
const P=[['kp','Kp',0,60,0.1],['ki','Ki',0,10,0.01],['kd','Kd',0,5,0.01],['sp','Setpoint (deg)',-10,10,0.1],['min','Min PWM',0,120,1],['kpb','Back Kp',0,60,0.1],['kdb','Back Kd',0,5,0.01]];
let init=false,hist=[];
const $=id=>document.getElementById(id);
function build(d){
  $('ctl').innerHTML=P.map(([k,n,lo,hi,s])=>
   `<label>${n}: <input type="number" id="n_${k}" step="${s}" value="${d[k]}" oninput="$('r_${k}').value=this.value">
    <input type="range" id="r_${k}" min="${lo}" max="${hi}" step="${s}" value="${d[k]}"
     oninput="$('n_${k}').value=this.value" onchange="apply()"></label>`).join('');
}
function apply(){fetch('/set?'+P.map(([k])=>k+'='+$('n_'+k).value).join('&'));}
function save(){fetch('/save');}
function toggle(){fetch('/toggle');}
function draw(){
  const c=$('c'),g=c.getContext('2d');g.clearRect(0,0,480,120);
  g.strokeStyle='#444';g.beginPath();g.moveTo(0,60);g.lineTo(480,60);g.stroke();
  g.strokeStyle='#4af';g.beginPath();
  hist.forEach((v,i)=>{const x=i*4,y=60-Math.max(-45,Math.min(45,v))*(60/45);i?g.lineTo(x,y):g.moveTo(x,y)});
  g.stroke();
}
async function tick(){
  try{
    const d=await (await fetch('/status')).json();
    if(!init){build(d);init=true;}
    $('a').textContent=d.angle.toFixed(1);
    $('o').textContent=d.out.toFixed(0);
    $('st').textContent=!d.enabled?'STOPPED':d.armed?'BALANCING':'WAITING (hold upright)';
    hist.push(d.angle);if(hist.length>120)hist.shift();draw();
  }catch(e){}
  setTimeout(tick,150);
}
tick();
</script></body></html>
)rawliteral";

void handleStatus() {
  char buf[300];
  snprintf(buf, sizeof(buf),
    "{\"angle\":%.2f,\"out\":%.1f,\"armed\":%d,\"enabled\":%d,\"kp\":%.2f,\"ki\":%.3f,\"kd\":%.3f,\"sp\":%.2f,\"min\":%d,\"kpb\":%.2f,\"kdb\":%.2f}",
    angle, lastOutput, armed ? 1 : 0, enabled ? 1 : 0, Kp, Ki, Kd, setpoint, minPWM, KpB, KdB);
  server.send(200, "application/json", buf);
}

void handleSet() {
  if (server.hasArg("kp"))  Kp       = server.arg("kp").toFloat();
  if (server.hasArg("ki"))  Ki       = server.arg("ki").toFloat();
  if (server.hasArg("kd"))  Kd       = server.arg("kd").toFloat();
  if (server.hasArg("sp"))  setpoint = server.arg("sp").toFloat();
  if (server.hasArg("min")) minPWM   = server.arg("min").toInt();
  if (server.hasArg("kpb")) KpB      = server.arg("kpb").toFloat();
  if (server.hasArg("kdb")) KdB      = server.arg("kdb").toFloat();
  integral = 0;
  server.send(200, "text/plain", "ok");
}

void handleSave() {
  prefs.begin("pid", false);
  prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
  prefs.putFloat("sp", setpoint); prefs.putInt("min", minPWM); prefs.putFloat("bkp", KpB); prefs.putFloat("bkd", KdB);
  prefs.end();
  server.send(200, "text/plain", "saved");
}

void handleToggle() {
  enabled = !enabled;
  server.send(200, "text/plain", enabled ? "running" : "stopped");
}

void loadParams() {
  prefs.begin("pid", true);
  Kp = prefs.getFloat("kp", Kp); Ki = prefs.getFloat("ki", Ki); Kd = prefs.getFloat("kd", Kd);
  setpoint = prefs.getFloat("sp", setpoint); minPWM = prefs.getInt("min", minPWM); KpB = prefs.getFloat("bkp", Kp); KdB = prefs.getFloat("bkd", Kd);
  prefs.end();
}

// Web server runs on core 0 so it never delays the balance loop (core 1)
void webTask(void*) {
  for (;;) { server.handleClient(); delay(2); }
}

void startWeb() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(300); Serial.print("."); }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed - continuing with Serial tuning only.");
    return;
  }
  Serial.print("\nOpen http://"); Serial.println(WiFi.localIP());

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/status", handleStatus);
  server.on("/set", handleSet);
  server.on("/save", handleSave);
  server.on("/toggle", handleToggle);
  server.begin();
  xTaskCreatePinnedToCore(webTask, "web", 8192, NULL, 1, NULL, 0);
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  ledcAttach(M1_EN, PWM_FREQ, PWM_BITS);
  ledcAttach(M2_EN, PWM_FREQ, PWM_BITS);
  stopMotors();

  loadParams();
  mpuInit();
  Serial.println("Keep the robot still - calibrating gyro...");
  delay(1000);
  calibrateGyro();

  float acc, gyro;
  mpuRead(acc, gyro);
  angle = acc;

  startWeb();

  Serial.println("Ready. Hold robot upright to arm it.");
  printParams();
  lastMicros = micros();
}

void loop() {
  handleSerial();

  unsigned long now = micros();
  if (now - lastMicros < LOOP_US) return;
  float dt = (now - lastMicros) / 1e6f;
  lastMicros = now;

  float accAngle, gyroRate;
  mpuRead(accAngle, gyroRate);

  // Complementary filter
  angle = 0.98f * (angle + gyroRate * dt) + 0.02f * accAngle;

  // Stopped from the web page
  if (!enabled) {
    armed = false;
    integral = 0;
    lastOutput = 0;
    stopMotors();
    return;
  }

  // Safety: fell over
  if (fabs(angle) > FALL_ANGLE) {
    armed = false;
    integral = 0;
    lastOutput = 0;
    stopMotors();
  }

  // Arm only when held close to upright
  if (!armed) {
    if (fabs(angle - setpoint) < START_ANGLE) {
      armed = true;
      integral = 0;
      Serial.println("ARMED");
    } else {
      stopMotors();
      return;
    }
  }

  // PID (derivative taken from gyro rate - less noisy than differentiating the angle)
  float error = angle - setpoint;
  integral += error * dt;
  integral = constrain(integral, -integralLimit, integralLimit);

  float kp = Kp, kd = Kd;
  if (error < 0) { kp = KpB; kd = KdB; }   // robot leaning backward: use the Back gains
  float output = kp * error + Ki * integral + kd * gyroRate;
  output = constrain(output, -255, 255);
  lastOutput = output;
  setMotors(output);

  if (telemetry && millis() - lastPrint > 50) {
    lastPrint = millis();
    Serial.printf("%.2f,%.1f\n", angle, output);   // Serial Plotter friendly
  }
}
