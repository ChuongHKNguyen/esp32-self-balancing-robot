/*
  Self-balancing robot
  ESP32 Dev Module + MPU6050 (GY-521) + L298N + 2x GB37-520
  (chuyen tu ban Arduino Mega 2560, giu nguyen logic)

  MPU6050 (GY-521):
    VCC -> 3V3, GND -> GND, SDA -> GPIO 21, SCL -> GPIO 22
    Lap GY-521 theo cach nao cung duoc -> chon AXIS_MODE ben duoi cho dung.

  L298N:
    ENA -> GPIO 25, IN1 -> GPIO 26, IN2 -> GPIO 27
    ENB -> GPIO 33, IN3 -> GPIO 18, IN4 -> GPIO 19
    GND chung voi ESP32 va nguon motor

  Serial Monitor: 115200 baud, "Newline"
  Lenh chinh PID luc dang chay:
    q / a : Kp +1 / -1
    w / s : Ki +0.1 / -0.1
    e / d : Kd +0.05 / -0.05
    r / f : setpoint +0.1 / -0.1 do (diem can bang)
    p     : in thong so hien tai
    x     : in gia tri tho cua cam bien (de chon AXIS_MODE)
    z     : dat tu the HIEN TAI (robot dung thang) la 0 do
    c / v : minPwm +5 / -5   (bu vung chet motor: tang neu robot phan ung cham voi goc nho)
    o     : bat/tat tu dong tim diem can bang (tu bat sau 5 giay robot tu dung - BUONG TAY ngay)
    h / n : K_SPD +0.1 / -0.1     b / m : K_POS +0.05 / -0.05
    t / y : gioi han toc do doi PWM -10 / +10 (nho = muot hon, lon = nhanh hon)
    u     : bat/tat Fuzzy PID (tu dong chinh Kp, Ki, Kd)

  DI TOI / LUI (tren web): Start -> robot CAN BANG TRUOC (Auto-balance BAT, doi hien "ON").
    CHI khi Auto-balance = ON moi giu duoc nut Forward / Back.
    Nghieng qua driveLimFwd (toi) hoac driveLimBack (lui) -> ngung di de robot tu lay lai can bang.
  RE TRAI / PHAI (tren web): cung dieu kien Auto-balance = ON. Re bang cach cho 2 banh quay lech nhau,
    chi dung phan PWM con du sau khi can bang -> CAN BANG LUON DUOC UU TIEN.

  WEB (dien thoai): xem tab web_ui_balance_turn.h
    Noi WiFi "UltraRobot" (mat khau 12345678), mo http://192.168.4.1
    Giu robot thang dung, bam "Start robot" -> robot do goc dung roi tu can bang.
*/

#include <Wire.h>
#include <esp_arduino_version.h>

// ---------------- Chan ----------------
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const int ENA = 25;
const int IN1 = 26;
const int IN2 = 27;
const int ENB = 33;
const int IN3 = 18;
const int IN4 = 19;

// ---------------- PWM (LEDC cua ESP32, thay cho analogWrite) ----------------
#define PWM_FREQ 500    // Hz. L298N cham: KHONG dat cao (20 kHz lam xung nho bi meo -> motor khong day o PWM thap, chip nong hon)
#define PWM_BITS 8      // 0..255 giong ban Arduino

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
// ESP32 Arduino core 3.x
void pwmSetup() {
  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);
}
void pwmWrite(int pin, int duty) {
  ledcWrite(pin, duty);
}
#else
// ESP32 Arduino core 2.x
const int CH_A = 0;
const int CH_B = 1;
void pwmSetup() {
  ledcSetup(CH_A, PWM_FREQ, PWM_BITS);
  ledcSetup(CH_B, PWM_FREQ, PWM_BITS);
  ledcAttachPin(ENA, CH_A);
  ledcAttachPin(ENB, CH_B);
}
void pwmWrite(int pin, int duty) {
  ledcWrite(pin == ENA ? CH_A : CH_B, duty);
}
#endif

// ---------------- Cau hinh dau/chieu ----------------
// Neu robot ngã mà banh xe quay NGUOC chieu ngã -> doi MOTOR_SIGN (1 <-> -1)
// Neu 1 banh quay nguoc banh kia -> doi MOTOR_A_DIR hoac MOTOR_B_DIR
const int MOTOR_SIGN  = 1;
const int MOTOR_A_DIR = 1;
const int MOTOR_B_DIR = 1;

// Goc dung thang = 0 do (hieu chuan khi bam Start tren web).
// Nghieng ve 1 phia -> goc DUONG, nghieng phia kia -> goc AM.
// Neu muon dao phia duong/am -> doi ANGLE_SIGN VA GYRO_SIGN VA MOTOR_SIGN cung luc.
// Neu nghieng robot ve phia truoc ma angle giam (am) -> doi ANGLE_SIGN
// Angle va gyroRate phai CUNG dau khi nghieng (xem Serial)
const int ANGLE_SIGN = 1;
const int GYRO_SIGN  = 1;

// ---------------- CHON TRUC (AXIS_MODE) ----------------
// Chon theo cach GY-521 duoc gan len robot (nhin cac mui ten X, Y tren board):
//   1: board nam ngang (Z huong len), robot nghieng truoc/sau quanh truc Y   [mac dinh]
//   2: board nam ngang (Z huong len), robot nghieng truoc/sau quanh truc X
//   3: board dung thang dung, truc X huong len, nghieng quanh truc Y
//   4: board dung thang dung, truc Y huong len, nghieng quanh truc X
//   5: truc Y huong len, nghieng quanh truc Z
//   6: truc X huong len, nghieng quanh truc Z
// Cach nhanh: gan robot dung thang, gui lenh 'x': truc nao doc ~ +-16384 la truc huong len.
// Roi nghieng robot truoc/sau: truc doc thay doi nhieu nhat (ngoai truc huong len) la truc "nghieng".
#define AXIS_MODE 2

// ---------------- MPU6050 ----------------
const uint8_t MPU_ADDR = 0x68;        // AD0 noi GND (mac dinh GY-521)
const float GYRO_SCALE = 65.5;        // +-500 deg/s -> 65.5 LSB/(deg/s)

// ---------------- Bo loc & PID ----------------
const float ALPHA = 0.98;             // complementary filter
const unsigned long LOOP_US = 5000;   // 5 ms = 200 Hz

float Kp = 25;
float Ki = 1.0;
float Kd = 1.54;
float setpoint = 0.0;                 // do, 0 = dung thang luc hieu chuan; chinh nhe de tim diem can bang

const float FALL_ANGLE = 35.0;        // qua goc nay -> tat motor
const float RESUME_ANGLE = 8.0;       // motor chay lai khi robot nam trong +-8 do so voi setpoint (truoc day 3 do, qua hep)
const int   MAX_PWM = 191;           // tran duty cua motor (75%) - CHI gioi han PWM cuoi, KHONG cat output PID
int         minPwm = 30;                 // bu vung chet cua motor (tang neu motor ru ru khong quay), lenh c / v
// --- Lam muot lenh motor (chong giat) ---
const float OUT_DEADBAND = 4.0;      // |output PID| nho hon muc nay -> tat motor (chong rung quanh 0)
const float MIN_RAMP     = 15.0;     // bu vung chet tang dan 0 -> minPwm khi output di tu 0 -> 15 (khong nhay vot)
const float GYRO_LPF     = 0.3;      // loc thong thap cho thanh D (0.1 = muot hon, 1.0 = khong loc)
int         pwmStepLimit = 40;       // PWM chi duoc doi toi da bay nhieu moi vong 5 ms (lenh t / y)
float       pwmNow = 0;              // PWM hien tai (co dau)
float       gyroF  = 0;              // gyro da loc, dung cho thanh D

// ---------------- Bien trang thai ----------------
float angle = 0;
float gyroRate = 0;
float gyroOffset = 0;
float angleOffset = 0;               // goc cua cam bien khi robot dung thang
float iTerm = 0;
bool  active = false;
unsigned long lastUs = 0;
unsigned long lastPrintMs = 0;
float lastOutput = 0;

// ---------------- START / STOP tu web ----------------
bool          armed = false;         // true sau khi bam Start va hieu chuan xong
volatile bool calibrating = false;   // dang do goc dung
volatile bool startReq = false;      // web yeu cau Start
volatile bool stopReq  = false;      // web yeu cau Stop
volatile uint8_t stopWhy = 1;        // ly do motor dang tat: 0 chay, 1 chua Start, 2 cho robot dung thang, 3 nghieng qua FALL_ANGLE, 4 loi cam bien

// ---------------- FUZZY PID (tu dong chinh Kp, Ki, Kd) ----------------
// Kp, Ki, Kd tren web = gia tri GOC. Fuzzy cong/tru them quanh gia tri goc dua tren
// sai so goc (error) va toc do nghieng (gyro). KpE/KiE/KdE = gia tri THUC SU dang dung.
#include "fuzzy_pid.h"
bool  fuzzyOn  = true;
float fzKpSpan = 8.0;            // Kp duoc phep lech +-8 quanh gia tri goc
float fzKiSpan = 0.3;            // Ki duoc phep lech +-0.3
float fzKdSpan = 0.3;            // Kd duoc phep lech +-0.3
const float FZ_SMOOTH = 0.05;    // lam muot gain (~0.1 s) de gain khong nhay dot ngot
float KpE = 25, KiE = 1.0, KdE = 1.54;

// ---------------- DI TOI / LUI (chi hoat dong khi Auto-balance dang ON va on dinh) ----------------
// Robot di bang cach NGHIENG goc muc tieu mot chut: PID se keo banh xe chay theo de dua robot ve goc do.
const int   DRIVE_SIGN = 1;              // neu bam Forward ma robot chay LUI -> doi thanh -1
volatile int driveCmd = 0;               // +1 = toi, -1 = lui, 0 = dung (web gui lien tuc khi dang giu nut)
volatile unsigned long driveLastMs = 0;  // lan cuoi nhan lenh di tu web
float driveLean = 3.0;                   // do nghieng muc tieu khi di (do): lon hon = nhanh hon
const float DRIVE_SLEW = 12.0;           // do/giay: goc muc tieu nghieng dan dan, khong giat
const unsigned long DRIVE_TIMEOUT_MS = 400;   // mat tin hieu web > 0.4 s -> tu dung (an toan)
const float DRIVE_FZ_BOOST = 1.5;        // khi dang di: fuzzy luon bat va duoc lech gain gap 1.5 lan
float driveLimFwd  = 12.0;               // do: dang di toi ma nghieng qua muc nay -> ngung di (phai < FALL_ANGLE)
float driveLimBack = 10.0;               // do: dang di lui ma nghieng qua muc nay -> ngung di (phai < FALL_ANGLE)
volatile bool driveLocked = false;       // da ngung do nghieng qua gioi han: phai THA NUT roi moi di lai
unsigned long driveEndMs = 0;            // luc vua tha nut
const unsigned long DRIVE_TRIM_FREEZE_MS = 1000;   // sau khi tha nut 1 s: khong hoc trim (robot con dang troi)

// ---------------- RE TRAI / PHAI (chi hoat dong khi Auto-balance dang ON) ----------------
// Banh A nhan them +turn, banh B nhan them -turn (robot quay tai cho, van can bang).
const int   TURN_SIGN = 1;               // neu bam Right ma robot quay TRAI -> doi thanh -1 (hoac doi day 2 motor)
volatile int turnCmd = 0;                // +1 = phai, -1 = trai (web gui lien tuc khi dang giu nut)
volatile unsigned long turnLastMs = 0;
float turnStrength = 60.0;               // PWM chenh lech 2 banh khi re (0..120); tang neu banh khong quay, giam neu re qua manh
const float TURN_SLEW = 400.0;           // PWM/giay: luc re tang/giam dan, khong giat
const float TURN_TILT_LIMIT = 10.0;      // do: nghieng qua muc nay -> ngung re de uu tien can bang
float turnNow = 0;                       // luc re dang ap dung (da lam muot)
// Giu nut lau: goc nghieng giam dan (nghieng giu nguyen = robot TANG TOC mai) de toc do khong vot len
const unsigned long DRIVE_TAPER_MS = 1500;   // sau ~1.5 s giu nut, goc nghieng giam xuong DRIVE_MIN_FRAC
const float DRIVE_MIN_FRAC = 0.3;            // giu 30% goc nghieng ban dau de van chay deu
int           driveDirPrev  = 0;
unsigned long driveHeldStart = 0;
float driveShiftF = 0;                   // goc nghieng muc tieu dang ap dung (da lam muot)

// ---------------- Tu dong tim diem can bang (chong "nga roi chay luon") ----------------
// Robot khong co encoder -> dung output PID trung binh de biet robot dang bi keo di ve phia nao,
// roi tu ha/nang goc muc tieu cho den khi output trung binh = 0 (robot dung yen tai cho).
const int   HOLD_SIGN = 1;      // neu bat 'o' ma robot chay NHANH HON -> doi thanh -1
const float AVG_TAU   = 0.5;    // s, hang so loc output
const float MAX_TRIM  = 5.0;    // do, gioi han do lech tu dong
float K_SPD = 0.2;              // phanh ngay khi bat dau troi (do / don vi output trung binh)
float K_POS = 0.1;              // keo ve diem can bang that (do/giay / don vi output trung binh)
bool  holdWanted = true;             // 'o' bat/tat tinh nang nay
bool  holdOn = false;                // dang hoat dong (chi bat sau khi robot da tu can bang HOLD_ARM_MS)
bool  holdTripped = false;           // bao ve da tat tinh nang cho den lan dung len ke tiep
const unsigned long HOLD_ARM_MS = 5000;   // (khong con dung) truoc day: cho co dinh 5 s
// Tu dong tim can bang chi BAT khi robot da THAT SU on dinh: goc nho + gyro nho lien tuc HOLD_STEADY_MS
const float HOLD_STEADY_ANGLE = 3.0;      // do
const float HOLD_STEADY_GYRO  = 15.0;     // do/giay
const unsigned long HOLD_STEADY_MS = 2000;   // ms
unsigned long steadySince = 0;
unsigned long activeSinceMs = 0;
float outAvg = 0;
float trim   = 0;
const float P_CAP        = 3.0;      // do, gioi han phan "phanh" ngay lap tuc
const float TARGET_SLEW  = 20.0;     // do/giay, goc muc tieu chi doi cham, khong nhay dot ngot
float shiftF = 0;                    // do lech goc muc tieu sau khi lam muot
unsigned long trimSatStart = 0;      // thoi diem trim bat dau cham gioi han

void resetHold() {
  trim = 0;
  outAvg = 0;
  shiftF = 0;
}
int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
int  i2cFails = 0;                   // so lan doc loi lien tiep
bool needResync = false;             // vua khoi phuc I2C -> dong bo lai goc

// ---------------- WEB UI (dien thoai) ----------------
// Phai dat SAU cac bien o tren va TRUOC handleSerial()
#include "web_ui_balance_turn.h"

// ---------------- MPU6050 ----------------
int16_t read16() {
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

// Tra ve false neu cam bien KHONG tra loi that (loi I2C, day long, cam bien treo)
bool readMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;
  ax = read16();
  ay = read16();
  az = read16();
  read16();               // bo qua nhiet do
  gx = read16();
  gy = read16();
  gz = read16();
  // 6 gia tri giong het nhau (vd. toan -1 hoac toan 0) = bus I2C chet, khong phai du lieu that
  if (ax == ay && ay == az && az == gx && gx == gy && gy == gz) return false;
  return true;
}

// Giai phong bus I2C neu cam bien giu day SDA o muc thap (9 xung clock + STOP)
void i2cBusClear() {
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  }
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(10);
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

// Tra ve false neu khong thay MPU6050 tren bus
bool initMPU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;

  writeReg(0x6B, 0x00);   // danh thuc MPU6050
  delay(100);
  writeReg(0x1A, 0x03);   // low-pass filter ~44 Hz
  writeReg(0x1B, 0x08);   // gyro +-500 deg/s
  writeReg(0x1C, 0x00);   // accel +-2g
  return true;
}

bool sensorOK() {
  int16_t a, b, c, d, e, f;
  return readMPU(a, b, c, d, e, f);
}

// Dua goc ve khoang -180..180
float wrap180(float a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// Goc nghieng tu accelerometer, theo AXIS_MODE
float accelAngle(int16_t ax, int16_t ay, int16_t az) {
  float a;
#if AXIS_MODE == 1
  a = atan2((float)ax, (float)az);
#elif AXIS_MODE == 2
  a = atan2((float)ay, (float)az);
#elif AXIS_MODE == 3
  a = atan2((float)az, (float)ax);
#elif AXIS_MODE == 4
  a = atan2((float)az, (float)ay);
#elif AXIS_MODE == 5
  a = atan2((float)ax, (float)ay);
#elif AXIS_MODE == 6
  a = atan2((float)ay, (float)ax);
#else
#error "AXIS_MODE phai tu 1 den 6"
#endif
  return ANGLE_SIGN * a * RAD_TO_DEG;
}

// Gia tri gyro tho cua truc dang nghieng, theo AXIS_MODE
float gyroRaw(int16_t gx, int16_t gy, int16_t gz) {
#if AXIS_MODE == 1 || AXIS_MODE == 3
  return (float)gy;
#elif AXIS_MODE == 2 || AXIS_MODE == 4
  return (float)gx;
#else
  return (float)gz;
#endif
}

const int WAIT_SECONDS = 3;   // sau khi bam Start: thoi gian de GIU robot thang dung truoc khi do goc (giay)

bool calibrateGyro() {
  Serial.println(F("Hieu chuan: dat robot dung o goc mong muon va GIU YEN."));
  for (int t = WAIT_SECONDS; t > 0; t--) {
    Serial.print(F("Bat dau do goc sau "));
    Serial.print(t);
    Serial.println(F(" giay..."));
    delay(1000);
  }
  Serial.println(F("Dang do goc ban dau - GIU YEN..."));
  long sumG = 0, sumAx = 0, sumAy = 0, sumAz = 0;
  const int N = 500;
  int ok = 0;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sumG  += (long)gyroRaw(gx, gy, gz);
      sumAx += ax;
      sumAy += ay;
      sumAz += az;
      ok++;
    }
    delay(3);
  }
  if (ok < N / 2) return false;          // cam bien khong tra loi du lieu that
  gyroOffset  = (float)sumG / ok;
  // Goc luc dung thang -> tru di de goc dung thang = 0 do
  angleOffset = accelAngle((int16_t)(sumAx / ok), (int16_t)(sumAy / ok), (int16_t)(sumAz / ok));
  Serial.println(F("Hieu chuan xong: goc dung thang = 0 do."));
  return true;
}

// ---------------- Dieu khien motor ----------------
// speed: -255..255 (duong = tien)
void driveMotor(int en, int inA, int inB, int speed) {
  if (speed > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else if (speed < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  }
  pwmWrite(en, abs(speed));
}

void stopMotors() {
  pwmNow = 0;
  turnNow = 0;
  driveMotor(ENA, IN1, IN2, 0);
  driveMotor(ENB, IN3, IN4, 0);
}

void setMotors(float out) {
  out *= MOTOR_SIGN;
  out = constrain(out, -255, 255);          // output PID giu thang 255 nhu code goc

  // PWM mong muon (co dau). Bu vung chet TANG DAN thay vi nhay vot tu 0 len minPwm.
  float want = 0;
  float a = fabs(out);
  if (a > OUT_DEADBAND) {
    float r = a / MIN_RAMP;
    if (r > 1.0) r = 1.0;
    float p = minPwm * r + a * (MAX_PWM - minPwm) / 255.0;
    want = (out < 0) ? -p : p;
  }

  // Gioi han toc do thay doi PWM moi vong -> motor khong giat dot ngot
  pwmNow += constrain(want - pwmNow, -(float)pwmStepLimit, (float)pwmStepLimit);
  int pwm = constrain((int)pwmNow, -MAX_PWM, MAX_PWM);

  // Re: banh A + turn, banh B - turn. Chi dung phan PWM con du (headroom) -> can bang luon duoc uu tien
  int turn = (int)turnNow;
  int headroom = MAX_PWM - abs(pwm);
  turn = constrain(turn, -headroom, headroom);
  int pwmA = constrain(pwm + turn, -MAX_PWM, MAX_PWM);
  int pwmB = constrain(pwm - turn, -MAX_PWM, MAX_PWM);
  driveMotor(ENA, IN1, IN2, pwmA * MOTOR_A_DIR);
  driveMotor(ENB, IN3, IN4, pwmB * MOTOR_B_DIR);
}

// ---------------- Start / Stop robot ----------------
void startRobot() {
  calibrating = true;
  Kp = 25; Ki = 1.0; Kd = 1.54;   // mac dinh khi bam Start
  KpE = Kp; KiE = Ki; KdE = Kd;
  driveCmd = 0; driveShiftF = 0; driveLocked = false; turnCmd = 0;
  setpoint = 0.0;        // goc 0 vua do lai luc Start -> khong mang setpoint cu sang (tranh motor khong chay lai)
  holdWanted = true;     // Start -> Auto-balance BAT truoc; on dinh roi moi tat de dieu khien
  stopMotors();
  Serial.println(F("START: dang tim goc dung - hay GIU robot thang dung va dung yen"));
  if (!calibrateGyro()) {
    calibrating = false;
    Serial.println(F("LOI: khong hieu chuan duoc (kiem tra MPU6050). Bam Start lai."));
    return;
  }
  angle = 0;
  gyroF = 0;
  iTerm = 0;
  pwmNow = 0;
  active = false;
  holdOn = false;
  holdTripped = false;
  resetHold();
  armed = true;
  calibrating = false;
  stopWhy = 2;
  lastUs = micros();
  Serial.println(F("Robot dang CAN BANG theo Kp, Ki, Kd cua ban."));
}

void stopRobot() {
  armed = false;
  stopWhy = 1;
  driveCmd = 0;
  driveLocked = false;
  turnCmd = 0;
  driveShiftF = 0;
  active = false;
  iTerm = 0;
  holdOn = false;
  resetHold();
  stopMotors();
  Serial.println(F("STOP: da dung robot"));
}

// ---------------- Serial tuning ----------------
void printParams() {
  Serial.print(F("Kp=")); Serial.print(Kp);
  Serial.print(F(" Ki=")); Serial.print(Ki);
  Serial.print(F(" Kd=")); Serial.print(Kd);
  Serial.print(F(" setpoint=")); Serial.print(setpoint);
  Serial.print(F(" trim=")); Serial.print(trim, 2);
  Serial.print(F(" minPwm=")); Serial.print(minPwm);
  Serial.print(F(" step=")); Serial.print(pwmStepLimit);
  Serial.print(F(" hold=")); Serial.println(holdWanted ? (holdOn ? F("ON") : F("cho")) : F("OFF"));
}

void printRaw() {
  Serial.print(F("RAW  ax=")); Serial.print(rawAx);
  Serial.print(F(" ay=")); Serial.print(rawAy);
  Serial.print(F(" az=")); Serial.print(rawAz);
  Serial.print(F(" | gx=")); Serial.print(rawGx);
  Serial.print(F(" gy=")); Serial.print(rawGy);
  Serial.print(F(" gz=")); Serial.println(rawGz);
}

void handleSerial() {
  while (Serial.available() || webCmdAvailable()) {
    char c = webCmdAvailable() ? webCmdPop() : Serial.read();
    switch (c) {
      case 'q': Kp += 1.0;  printParams(); break;
      case 'a': Kp -= 1.0;  if (Kp < 0) Kp = 0; printParams(); break;
      case 'w': Ki += 0.1;  printParams(); break;
      case 's': Ki -= 0.1;  if (Ki < 0) Ki = 0; printParams(); break;
      case 'e': Kd += 0.05; printParams(); break;
      case 'd': Kd -= 0.05; if (Kd < 0) Kd = 0; printParams(); break;
      case 'r': setpoint += 0.1; printParams(); break;
      case 'f': setpoint -= 0.1; printParams(); break;
      case 'c': minPwm = min(minPwm + 5, 120); printParams(); break;
      case 'v': minPwm = max(minPwm - 5, 0);   printParams(); break;
      case 'o':
        holdWanted = !holdWanted;
        if (!holdWanted && holdOn) {         // tat Auto-balance: GIU LAI diem can bang da hoc (khong mat lech)
          setpoint = constrain(setpoint + trim, -10.0f, 10.0f);
          Serial.print(F("Giu diem can bang da hoc: setpoint=")); Serial.println(setpoint, 2);
        }
        holdOn = false;
        holdTripped = false;
        resetHold();
        Serial.println(holdWanted ? F("Tu dong tim can bang: CHO BAT (tu bat sau 5 giay robot tu dung, KHONG cam tay)")
                                  : F("Tu dong tim can bang: TAT"));
        printParams();
        break;
      case 't': pwmStepLimit = max(pwmStepLimit - 10, 10);  printParams(); break;
      case 'y': pwmStepLimit = min(pwmStepLimit + 10, 255); printParams(); break;
      case 'h': K_SPD += 0.1; printParams(); break;
      case 'n': K_SPD -= 0.1; if (K_SPD < 0) K_SPD = 0; printParams(); break;
      case 'b': K_POS += 0.05; printParams(); break;
      case 'm': K_POS -= 0.05; if (K_POS < 0) K_POS = 0; printParams(); break;
      case 'u': fuzzyOn = !fuzzyOn; Serial.println(fuzzyOn ? F("Fuzzy PID: BAT") : F("Fuzzy PID: TAT")); break;
      case 'p': printParams(); break;
      case 'x': printRaw(); break;
      case 'z':
        if (!armed) { Serial.println(F("Bam Start truoc")); break; }
        angleOffset = accelAngle(rawAx, rawAy, rawAz);
        angle = 0;
        iTerm = 0;
        resetHold();
        Serial.println(F("Da dat tu the hien tai = 0 do"));
        break;
      default: break;
    }
  }
}

// ---------------- setup / loop ----------------
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pwmSetup();
  stopMotors();

  Serial.begin(115200);
  delay(300);
  webSetup();                     // bat WiFi + web tren core 0 (khong lam cham vong can bang)

  // Cho den khi MPU6050 tra loi that (neu khong -> motor van tat, khong chay mu)
  while (true) {
    if (initMPU() && sensorOK()) break;
    stopMotors();
    Serial.println(F("LOI: khong doc duoc MPU6050! Kiem tra VCC->3V3, GND, SDA->GPIO21, SCL->GPIO22 (day long / han lanh). Thu lai sau 2 giay..."));
    delay(2000);
    Wire.end();
    i2cBusClear();
  }

  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (readMPU(ax, ay, az, gx, gy, gz)) {
    rawAx = ax; rawAy = ay; rawAz = az;
    rawGx = gx; rawGy = gy; rawGz = gz;
  }
  angle = 0;
  gyroF = 0;

  printParams();
  Serial.println(F("San sang. Robot DANG CHO. Mo web, giu robot thang dung roi bam 'Start robot'."));
  lastUs = micros();
}

void loop() {
  // Lenh Start / Stop tu web
  if (stopReq)  { stopReq = false;  stopRobot(); }
  if (startReq) { startReq = false; if (!armed) startRobot(); }

  unsigned long now = micros();
  if (now - lastUs < LOOP_US) {
    handleSerial();
    return;
  }
  float dt = (now - lastUs) / 1000000.0;
  lastUs = now;

  // Chua bam Start (hoac da Stop) -> motor tat, khong can bang
  if (!armed) {
    stopMotors();
    handleSerial();
    return;
  }

  // 1) Doc cam bien (loi -> TAT MOTOR, thu khoi phuc I2C, khong bao gio chay voi du lieu gia)
  int16_t ax, ay, az, gx, gy, gz;
  if (!readMPU(ax, ay, az, gx, gy, gz)) {
    stopMotors();
    active = false;
    stopWhy = 4;
    iTerm = 0;
    holdOn = false;
    resetHold();
    i2cFails++;
    if (millis() - lastPrintMs >= 500) {
      lastPrintMs = millis();
      Serial.println(F("LOI doc MPU6050 - DA TAT MOTOR (kiem tra day SDA/SCL, VCC 3V3, GND)"));
    }
    if (i2cFails >= 20) {                 // ~0.1 s loi lien tuc -> thu khoi phuc bus I2C
      i2cFails = 0;
      Wire.end();
      i2cBusClear();
      if (initMPU()) needResync = true;
    }
    handleSerial();
    return;
  }
  i2cFails = 0;
  if (needResync) {
    needResync = false;
    angle = wrap180(accelAngle(ax, ay, az) - angleOffset);
    gyroF = 0;
    Serial.println(F("MPU6050 da ket noi lai - dung robot gan thang de chay tiep"));
  }
  rawAx = ax; rawAy = ay; rawAz = az;
  rawGx = gx; rawGy = gy; rawGz = gz;

  gyroRate = GYRO_SIGN * (gyroRaw(gx, gy, gz) - gyroOffset) / GYRO_SCALE;   // do/giay
  gyroF += GYRO_LPF * (gyroRate - gyroF);     // gyro da loc cho thanh D
  float accAngle = wrap180(accelAngle(ax, ay, az) - angleOffset);

  // 2) Complementary filter
  angle = ALPHA * (angle + gyroRate * dt) + (1.0 - ALPHA) * accAngle;

  // Tu dong tim diem can bang: chi BAT sau HOLD_ARM_MS ke tu luc robot bat dau can bang
  // (de co thoi gian buong tay - neu van dang cam thi phan nay se "wind-up")
  bool steady = active && fabs(angle - setpoint) < HOLD_STEADY_ANGLE && fabs(gyroF) < HOLD_STEADY_GYRO;
  if (!steady) steadySince = millis();
  if (active && holdWanted && !holdOn && millis() - steadySince > HOLD_STEADY_MS) {
    holdOn = true;
    resetHold();
    Serial.println(F("Tu dong tim diem can bang: BAT"));
  }

  // ---- Di toi / lui: chi khi Auto-balance = ON (holdOn) va khong bi khoa ----
  float tilt = (angle - setpoint) * DRIVE_SIGN;       // duong = dang nghieng ve phia TOI
  if (millis() - driveLastMs > DRIVE_TIMEOUT_MS) { driveCmd = 0; driveLocked = false; }   // mat tin hieu web -> dung
  int dc = (holdOn && !driveLocked) ? driveCmd : 0;
  if (dc != 0 && (tilt > driveLimFwd || tilt < -driveLimBack)) {
    driveLocked = true;                               // nghieng qua gioi han -> ngung di, de robot tu lay lai can bang
    driveCmd = 0;
    dc = 0;
    Serial.println(F("DI CHUYEN: nghieng qua gioi han -> ngung di (tha nut roi bam lai)"));
  }
  if (dc != driveDirPrev) {
    if (dc == 0) driveEndMs = millis();
    driveDirPrev = dc;
    driveHeldStart = millis();
  }
  float driveTarget = 0;
  if (dc != 0) {
    // dang giu nut: goc nghieng giam dan theo thoi gian giu (chong tang toc mai)
    float held = (float)(millis() - driveHeldStart) / (float)DRIVE_TAPER_MS;
    if (held > 1.0f) held = 1.0f;
    float taper = 1.0f - (1.0f - DRIVE_MIN_FRAC) * held;
    driveTarget = dc * DRIVE_SIGN * driveLean * taper;
  }
  float driveStep = DRIVE_SLEW * dt;
  driveShiftF += constrain(driveTarget - driveShiftF, -driveStep, driveStep);
  bool driving = (dc != 0) || fabs(driveShiftF) > 0.3;

  // ---- Re trai / phai: chi khi Auto-balance = ON; mat tin hieu web -> tu dung ----
  if (millis() - turnLastMs > DRIVE_TIMEOUT_MS) turnCmd = 0;
  int tc = holdOn ? turnCmd : 0;
  if (fabs(angle - setpoint) > TURN_TILT_LIMIT) tc = 0;     // nghieng nhieu -> ngung re, uu tien can bang
  float turnTarget = tc * TURN_SIGN * turnStrength;
  float turnStep = TURN_SLEW * dt;
  turnNow += constrain(turnTarget - turnNow, -turnStep, turnStep);

  // Goc muc tieu = setpoint + do lech tu dong (chong troi)
  float autoShift = 0;
  if (holdOn) {
    // dang giu nut Forward/Back: tam ngung "phanh chong troi" de khong cai nhau voi lenh di; tha nut -> phanh tro lai
    float pTerm = (dc != 0) ? 0.0f : constrain(-HOLD_SIGN * K_SPD * outAvg, -P_CAP, P_CAP);
    autoShift = constrain(trim + pTerm, -MAX_TRIM, MAX_TRIM);
  }
  float stepMax = TARGET_SLEW * dt;           // muc tieu chi doi cham -> khong giat
  shiftF += constrain(autoShift - shiftF, -stepMax, stepMax);


  float target = setpoint + shiftF + driveShiftF;

  // Bao ve: trim ket o gioi han > 0.5 s = robot dang bi cam hoac khong phan hoi -> tat tu dong tim can bang
  if (holdOn && fabs(shiftF) >= MAX_TRIM - 0.05) {
    if (trimSatStart == 0) {
      trimSatStart = millis();
    } else if (millis() - trimSatStart > 500) {
      holdOn = false;
      holdTripped = true;
      resetHold();
      trimSatStart = 0;
      Serial.println(F("CANH BAO: trim cham gioi han (robot dang bi cam / khong phan hoi) -> da TAT tu dong tim can bang"));
    }
  } else {
    trimSatStart = 0;
  }
  float baseError = angle - setpoint;      // dung cho an toan (nga / dung len lai)
  float error = angle - target;            // dung cho PID

  // 3) An toan: ngã thi tat motor, dung len lai thi chay tiep
  if (fabs(baseError) > FALL_ANGLE) {
    if (active) { Serial.print(F("TAT MOTOR: nghieng qua ")); Serial.print(angle, 1); Serial.println(F(" do")); }
    stopWhy = 3;
    active = false;
    iTerm = 0;
    holdOn = false;
    resetHold();
    driveCmd = 0;
    driveShiftF = 0;
    stopMotors();
  } else if (!active && fabs(baseError) < RESUME_ANGLE) {
    active = true;
    stopWhy = 0;
    Serial.println(F("MOTOR BAT: robot da trong vung can bang"));
    iTerm = 0;
    holdOn = false;
    holdTripped = false;
    activeSinceMs = millis();
    resetHold();
  } else if (!active) {
    stopWhy = 2;        // dang cho robot dung thang (trong +-RESUME_ANGLE)
  }

  // 4) PID (thanh D dung truc tiep van toc goc cua gyro -> it nhieu)
  if (active) {
    // Fuzzy PID: gain muc tieu = gain goc + do lech fuzzy, roi lam muot
    float tKp = Kp, tKi = Ki, tKd = Kd;
    if (fuzzyOn || driving) {                      // dang di -> fuzzy luon bat
      float dKp, dKi, dKd;
      fuzzyCompute(error, gyroF, dKp, dKi, dKd);   // moi gia tri nam trong -1..1
      float k = driving ? DRIVE_FZ_BOOST : 1.0f;
      tKp = fmaxf(0.0f, Kp + dKp * fzKpSpan * k);
      tKi = fmaxf(0.0f, Ki + dKi * fzKiSpan * k);
      tKd = fmaxf(0.0f, Kd + dKd * fzKdSpan * k);
    }
    KpE += FZ_SMOOTH * (tKp - KpE);
    KiE += FZ_SMOOTH * (tKi - KiE);
    KdE += FZ_SMOOTH * (tKd - KdE);

    iTerm += KiE * error * dt;
    iTerm = constrain(iTerm, -100, 100);   // chong windup

    float output = KpE * error + iTerm + KdE * gyroF;
    lastOutput = output;

    // Loc output -> biet robot dang bi keo di dau; tich phan cham de keo ve diem can bang that
    float outC = constrain(output, -255, 255);
    outAvg += (outC - outAvg) * (dt / AVG_TAU);       // luon cap nhat: dung de uoc luong toc do (phanh khi di)
    if (holdOn && fabs(angle - setpoint) < 6.0f && !driving && millis() - driveEndMs > DRIVE_TRIM_FREEZE_MS) {   // khong hoc trim khi dang nghieng xa / dang di / vua tha nut
      trim   -= HOLD_SIGN * K_POS * outAvg * dt;
      trim    = constrain(trim, -MAX_TRIM, MAX_TRIM);
    }

    setMotors(output);
  }

  // 5) In debug (5 lan/giay)
  if (millis() - lastPrintMs >= 200) {
    lastPrintMs = millis();
    Serial.print(F("angle=")); Serial.print(angle, 1);
    Serial.print(F(" gyro=")); Serial.print(gyroRate, 1);
    Serial.print(F(" out=")); Serial.print(lastOutput, 0);
    Serial.print(F(" trim=")); Serial.print(trim, 2);
    Serial.print(F(" Kpe=")); Serial.print(KpE, 1);
    Serial.print(F(" Kie=")); Serial.print(KiE, 2);
    Serial.print(F(" Kde=")); Serial.print(KdE, 2);
    Serial.print(F(" lean=")); Serial.print(driveShiftF, 1);
    Serial.println(active ? F(" RUN") : F(" STOP"));
  }

  handleSerial();
}
