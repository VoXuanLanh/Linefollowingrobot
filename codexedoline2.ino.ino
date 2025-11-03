// ==========================
// Khai báo chân động cơ
// ==========================
#define ENA 11
#define IN1 6
#define IN2 7
#define ENB 10
#define IN3 9
#define IN4 8

// ==========================
// Khai báo cảm biến (5 sensor)
#define ir1 A1   // Left-most
#define ir2 A2
#define ir3 A3
#define ir4 A4
#define ir5 A5   // Right-most

// ==========================
// Tham số PID
double Kp = 6.5068;   // hệ số tỉ lệ
double Ki = 0.0001;    // tích phân
double Kd = .7199;   // vi phân

int lastError = 0;
int baseSpeed = 70; // tốc độ cơ bản (0-255)

// ==========================
void setup() {
  // Động cơ
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // Cảm biến
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  Serial.begin(9600);
}

// ==========================
// Hàm đọc cảm biến và tính error
// ==========================
int readSensors() {
  int s1 = !digitalRead(ir1);
  int s2 = !digitalRead(ir2);
  int s3 = !digitalRead(ir3);
  int s4 = !digitalRead(ir4);
  int s5 = !digitalRead(ir5);

  // Tính vị trí line (weighted average)
  int sum = s1 + s2 + s3 + s4 + s5;
  if (sum == 0) return lastError; // mất line => giữ lỗi cũ

  if (s2 && s3 && s4) return 0;
  int pos = (s1* -10 + s2* -5 + s3*0 + s4*5 + s5*10) / sum;
  // pos < 0 => line lệch trái, pos > 0 => lệch phải

  return pos;
}

// ==========================
// Hàm điều khiển động cơ
// ==========================
void setMotor(int leftSpeed, int rightSpeed) {
  // Giới hạn tốc độ
  leftSpeed = constrain(leftSpeed, -180, 180);
  rightSpeed = constrain(rightSpeed, -180, 180);

  // Motor trái
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }

  // Motor phải
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightSpeed);
  }
}

// ==========================
void loop() {
  // Đọc sensor
  int s1 = !digitalRead(ir1);
  int s2 = !digitalRead(ir2);
  int s3 = !digitalRead(ir3);
  int s4 = !digitalRead(ir4);
  int s5 = !digitalRead(ir5);

  int sum = s1+s2+s3+s4+s5;

  // ---------------------------
  // NGÃ TƯ: tất cả sensor đen
  // ---------------------------
  if(sum == 5) {
    // Ví dụ rẽ phải tại ngã tư
    setMotor(70, 50);   // trái nhanh, phải chậm
    lastError = 4;       // cập nhật error giả lập
    delay(120);          // giữ một chút để quay xong
    return;
  }


  // ---------------------------
  // PID bình thường
  // ---------------------------
  int error = readSensors();

  int P = error;
  static int I = 0;
  int D = error - lastError;
  I += error;

  double pid = Kp*P + Ki*I + Kd*D;

  int leftMotorSpeed  = baseSpeed - pid;
  int rightMotorSpeed = baseSpeed + pid;

  setMotor(leftMotorSpeed, rightMotorSpeed);

  lastError = error;

  delay(10);
}