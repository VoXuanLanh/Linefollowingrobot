// MAP SỐ 8 - LINE FOLLOWER + NGÃ TƯ
// ==========================
// Khai báo chân động cơ
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
double Kp = 39.9999;
double Ki = 0.0002;
double Kd = 9.9999;

int lastError = 0;
int baseSpeed = 80; 

int I = 0;
int I_max = 100;
double alpha = 0.7;
double D_filtered = 0;

// Timer
unsigned long lastTime = 0;
int sampleTime = 10;

// ==========================
// Biến ngã tư
int demNgaTu = 0;
bool flagQuaNgaTu = false; 

// ==========================
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  Serial.begin(9600);
}

// ==========================
// Đọc cảm biến và tính error
int readSensors() {
  int s1 = !digitalRead(ir1);
  int s2 = !digitalRead(ir2);
  int s3 = !digitalRead(ir3);
  int s4 = !digitalRead(ir4);
  int s5 = !digitalRead(ir5);

  int sum = s1 + s2 + s3 + s4 + s5;
  if (sum == 0) return lastError;

  int pos = (s1* -4 + s2* -2 + s3*0 + s4*2 + s5*4) / sum;
  return pos;
}

// ==========================
// Điều khiển động cơ
void setMotor(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -180, 180);
  rightSpeed = constrain(rightSpeed, -180, 180);

  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }

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
// Hàm xử lý rẽ tại ngã tư
void reTrai() {
  setMotor(-80, 80); // quay trái
  delay(350); // chỉnh cho phù hợp
}

void rePhai() {
  setMotor(80, -80); // quay phải
  delay(350); 
}

void diThang() {
  setMotor(70, 70); // chạy thẳng nhanh hơn
  delay(200); 
}

// ==========================
void loop() {
  unsigned long now = millis();
  if (now - lastTime >= sampleTime) {
    lastTime = now;

    // Đọc sensor
    int s1 = !digitalRead(ir1);
    int s2 = !digitalRead(ir2);
    int s3 = !digitalRead(ir3);
    int s4 = !digitalRead(ir4);
    int s5 = !digitalRead(ir5);

    int sum = s1+s2+s3+s4+s5;

    // =========================
    // Phát hiện ngã tư
    if (sum >= 4 && !flagQuaNgaTu) {
      flagQuaNgaTu = true;
      demNgaTu++;
      Serial.print("Gap nga tu: "); 
      Serial.println(demNgaTu);

      // Tạm ngưng PID, xử lý theo logic
      switch(demNgaTu) {
        case 1: diThang(); break;
        case 2: rePhai(); break;
        case 3: diThang(); break;
        case 4: reTrai(); break;
      }
    }

    // Reset flag sau khi qua ngã tư
    if (sum < 3 && flagQuaNgaTu) {
      flagQuaNgaTu = false;
    }

    // =========================
    // Nếu không phải đang rẽ, thì chạy PID
    if (!flagQuaNgaTu) {
      int error = readSensors();
      if (abs(error) <= 1) error = 0;

      int P = error;
      I += error;
      I = constrain(I, -I_max, I_max);
      double D_raw = error - lastError;
      D_filtered = alpha * D_filtered + (1 - alpha) * D_raw;

      double pid = Kp*P + Ki*I + Kd*D_filtered;

      int leftMotorSpeed  = baseSpeed - pid;
      int rightMotorSpeed = baseSpeed + pid;

      setMotor(leftMotorSpeed, rightMotorSpeed);
      lastError = error;
    }
  }
}
