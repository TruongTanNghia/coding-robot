/*
 * motor_bridge.ino — Arduino Mega 2560
 * Nhận lệnh tốc độ từ Pi5X qua USB Serial, điều khiển L298N,
 * và gửi số xung encoder về Pi để tính odometry.
 *
 * ĐẤU DÂY (khớp 100% với tài liệu Final1.docx):
 *   Encoder TRÁI :  A=D2,  B=D3
 *   Encoder PHẢI :  A=D18, B=D19
 *   L298N kênh A (motor PHẢI): IN1=D5, IN2=D9,  ENA=D12 (giữ HIGH, tháo jumper)
 *   L298N kênh B (motor TRÁI): IN3=D10, IN4=D6, ENB=D13 (giữ HIGH, tháo jumper)
 *
 * GIAO THỨC SERIAL (115200 baud, kết thúc bằng '\n'):
 *   Pi -> Mega:  "V <pwm_trai> <pwm_phai>"   pwm: -255..255 (âm = lùi)
 *                "S"                          dừng ngay
 *   Mega -> Pi:  "E <tick_trai> <tick_phai>"  gửi mỗi 50ms (tick cộng dồn, kiểu long)
 *
 * AN TOÀN: nếu quá 500ms không nhận được lệnh "V" nào -> tự dừng motor
 * (chống mất kết nối Wi-Fi/USB, khớp yêu cầu "mất tín hiệu thì xe dừng").
 */

// ---------- Chân cắm ----------
const uint8_t ENC_L_A = 2;
const uint8_t ENC_L_B = 3;
const uint8_t ENC_R_A = 18;
const uint8_t ENC_R_B = 19;

const uint8_t IN1 = 5;   // motor PHẢI
const uint8_t IN2 = 9;
const uint8_t ENA = 12;

const uint8_t IN3 = 10;  // motor TRÁI
const uint8_t IN4 = 6;
const uint8_t ENB = 13;

// Nếu bánh quay ngược chiều mong muốn mà lười đảo dây OUT1/OUT2,
// đổi 1 -> -1 ở đây:
const int8_t DIR_LEFT  = 1;
const int8_t DIR_RIGHT = 1;

// ---------- Encoder ----------
volatile long ticksL = 0;
volatile long ticksR = 0;

void isrLeftA() {
  // Đọc kênh B để biết chiều quay (quadrature x1, đủ dùng cho odom)
  if (digitalRead(ENC_L_B) == HIGH) ticksL++;
  else                              ticksL--;
}

void isrRightA() {
  if (digitalRead(ENC_R_B) == HIGH) ticksR++;
  else                              ticksR--;
}

// ---------- Motor ----------
void setMotor(uint8_t pinF, uint8_t pinR, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    analogWrite(pinF, pwm);
    analogWrite(pinR, 0);
  } else {
    analogWrite(pinF, 0);
    analogWrite(pinR, -pwm);
  }
}

void driveLR(int pwmL, int pwmR) {
  setMotor(IN3, IN4, DIR_LEFT  * pwmL);  // kênh B = TRÁI
  setMotor(IN1, IN2, DIR_RIGHT * pwmR);  // kênh A = PHẢI
}

void stopAll() { driveLR(0, 0); }

// ---------- Trạng thái ----------
unsigned long lastCmdMs  = 0;
unsigned long lastEncMs  = 0;
const unsigned long CMD_TIMEOUT_MS = 500;
const unsigned long ENC_PERIOD_MS  = 50;

char lineBuf[32];
uint8_t lineLen = 0;

void handleLine(char *line) {
  if (line[0] == 'V') {
    int l = 0, r = 0;
    if (sscanf(line + 1, "%d %d", &l, &r) == 2) {
      driveLR(l, r);
      lastCmdMs = millis();
    }
  } else if (line[0] == 'S') {
    stopAll();
    lastCmdMs = millis();
  }
}

void setup() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, RISING);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  digitalWrite(ENA, HIGH);   // enable cố định, PWM đánh vào IN
  digitalWrite(ENB, HIGH);

  stopAll();
  Serial.begin(115200);
}

void loop() {
  // Đọc lệnh từ Pi
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }

  unsigned long now = millis();

  // Watchdog: mất lệnh quá 500ms -> dừng
  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    stopAll();
  }

  // Gửi encoder về Pi mỗi 50ms
  if (now - lastEncMs >= ENC_PERIOD_MS) {
    lastEncMs = now;
    long l, r;
    noInterrupts();
    l = ticksL; r = ticksR;
    interrupts();
    Serial.print("E ");
    Serial.print(l);
    Serial.print(' ');
    Serial.println(r);
  }
}
