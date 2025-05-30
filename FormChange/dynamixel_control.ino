#include <DynamixelWorkbench.h>

#define LED_BUILTIN 13

const int enablePin1 = 8;
const int motorPin1A = 9;
const int motorPin1B = 10;

const int enablePin2 = 11;
const int motorPin2A = 12;
const int motorPin2B = 13;

#define DXL_BUS_SERIAL Serial3
const uint8_t DXL1_ID = 1;
const uint8_t DXL2_ID = 2;
const uint8_t DXL3_ID = 3;
const uint8_t DXL4_ID = 4;
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelWorkbench dxl_wb;

char currentCommand = '\0';  // 현재 커맨드 저장

const float INITIAL_ANGLE1 = 277;  // ID 1 모터의 초기 각도
const float INITIAL_ANGLE2 = 235;  // ID 2 모터의 초기 각도

const int32_t CENTER_POSITION1 = (INITIAL_ANGLE1 / 360.0) * 4096;
const int32_t CENTER_POSITION2 = (INITIAL_ANGLE2 / 360.0) * 4096;

const int32_t MIN_POSITION1 = CENTER_POSITION1 - (65.0 / 360.0) * 4096;
const int32_t MAX_POSITION1 = CENTER_POSITION1 + (65.0 / 360.0) * 4096;
const int32_t MIN_POSITION2 = CENTER_POSITION2 - (67.0 / 360.0) * 4096;
const int32_t MAX_POSITION2 = CENTER_POSITION2 + (67.0 / 360.0) * 4096;

int motor3_target_speed = 0;
int motor4_target_speed = 0;
int motor3_current_speed = 0;
int motor4_current_speed = 0;

// Define the desired profile velocity and acceleration
const uint32_t PROFILE_VELOCITY = 50;  // Adjust as needed
const uint32_t PROFILE_ACCELERATION = 10;  // Adjust as needed

bool setupCompleted = false;  // Setup 완료 여부를 추적하는 플래그 변수

void setup() {
  pinMode(motorPin1A, OUTPUT);
  pinMode(motorPin1B, OUTPUT);
  pinMode(enablePin1, OUTPUT);

  pinMode(motorPin2A, OUTPUT);
  pinMode(motorPin2B, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);

  // DynamixelWorkbench 초기화
  if (dxl_wb.init("/dev/ttyACM0", 115200) == false) {
    Serial.println("Failed to initialize DynamixelWorkbench");
    while (1);
  }

  // Dynamixel 설정 및 초기화
  initializeDynamixel();

  Serial.println("Setup completed");
  setupCompleted = true;  // Setup 완료로 플래그 설정
}

void loop() {

  // Dynamixel 토크 켜기
  torqueOnAll();

    // Dynamixel 상태 점검
  if (!checkDynamixelConnections()) {
    setupCompleted = false;  // Setup 상태를 false로 설정하여 다시 초기화하도록 유도
  }

  if (!setupCompleted) {
    // Dynamixel 재연결 및 재설정
    initializeDynamixel();
  }

  if (Serial.available() > 0) {  // Check if there's serial input available from the keyboard
    String data = Serial.readStringUntil('\n');  // 줄바꿈까지 읽기
    data.trim();  // 데이터 앞뒤 공백 제거
    int commaIndex = data.indexOf(',');  // 쉼표의 위치 찾기

    if (commaIndex != -1) {
      // 쉼표가 있는 경우: 숫자 쌍 처리
      String number1Str = data.substring(0, commaIndex);  // 쉼표 앞부분
      String number2Str = data.substring(commaIndex + 1);  // 쉼표 뒷부분
      motor3_target_speed = number1Str.toInt();  // 문자열을 정수로 변환
      motor4_target_speed = number2Str.toInt();  // 문자열을 정수로 변환
      Serial.print("motor3_target_speed: ");
      Serial.print(motor3_target_speed);
      Serial.print("motor4_target_speed: ");
      Serial.println(motor4_target_speed);
    } else if (data.length() == 1) {
      // 단일 문자가 입력된 경우: 명령 처리
      char command = data.charAt(0);
      currentCommand = command;
    } else {
      // 잘못된 입력 처리
      Serial.println("Invalid input received");
    }
  }

  if (currentCommand != '\0') {
    executeCommand(currentCommand);
    currentCommand = '\0';
  }
  
  speed2acceleration();   // 모터 속도 제어 함수 호출
}

bool checkDynamixelConnections() {
  bool allConnected = true;
  for (uint8_t id = DXL1_ID; id <= DXL4_ID; ++id) {
    if (!dxl_wb.ping(id)) {
      Serial.print("Dynamixel ID ");
      Serial.print(id);
      Serial.println(" is not connected");
      allConnected = false;
    }
  }
  return allConnected;
}

void initializeDynamixel() {
  // Dynamixel 테스트
  testDynamixel(DXL1_ID);
  testDynamixel(DXL2_ID);
  testDynamixel(DXL3_ID);
  testDynamixel(DXL4_ID);

  // Dynamixel 운영 모드 설정
  // 모터 1과 2를 3번 모드로 설정
  setOperatingMode(DXL1_ID, 3);
  setOperatingMode(DXL2_ID, 3);

  // 모터 3과 4를 1번 모드로 설정
  setOperatingMode(DXL3_ID, 1);
  setOperatingMode(DXL4_ID, 1);

  // Dynamixel 토크 켜기
  torqueOnAll();

  // Profile velocity와 acceleration 설정
  setProfileVelocityAcceleration(DXL1_ID, PROFILE_VELOCITY, PROFILE_ACCELERATION);
  setProfileVelocityAcceleration(DXL2_ID, PROFILE_VELOCITY, PROFILE_ACCELERATION);

}

void torqueOnAll() {
  if (dxl_wb.torqueOn(DXL1_ID) == false) {
    Serial.println("Failed to turn on torque for Dynamixel ID 1");
  } else {
    Serial.println("Successfully turned on torque for Dynamixel ID 1");
  }

  if (dxl_wb.torqueOn(DXL2_ID) == false) {
    Serial.println("Failed to turn on torque for Dynamixel ID 2");
  } else {
    Serial.println("Successfully turned on torque for Dynamixel ID 2");
  }

  if (dxl_wb.torqueOn(DXL3_ID) == false) {
    Serial.println("Failed to turn on torque for Dynamixel ID 3");
  } else {
    Serial.println("Successfully turned on torque for Dynamixel ID 3");
  }

  if (dxl_wb.torqueOn(DXL4_ID) == false) {
    Serial.println("Failed to turn on torque for Dynamixel ID 4");
  } else {
    Serial.println("Successfully turned on torque for Dynamixel ID 4");
  }
}

void executeCommand(char command) {
  switch (command) {
    case 'E':
      extendMotors();
      delay(9000);
      rotateDynamixel(DXL1_ID, INITIAL_ANGLE1 + 65);  // 30도 증가
      rotateDynamixel(DXL2_ID, INITIAL_ANGLE2 - 67);  // 30도 감소
      delay(3000);
      retractMotors();
      break;
    case 'R':
      extendMotors();
      delay(9000);
      rotateDynamixel(DXL1_ID, INITIAL_ANGLE1);
      rotateDynamixel(DXL2_ID, INITIAL_ANGLE2);
      delay(3000);
      retractMotors();
      break;
    case 'S':
      stopMotors();
      delay(1000);
      extendMotors();
      delay(9000);
      rotateDynamixel(DXL1_ID, INITIAL_ANGLE1);
      rotateDynamixel(DXL2_ID, INITIAL_ANGLE2);
      delay(3000);
      retractMotors();
      break;
    default:
      Serial.println("Unknown command received");
      break;
  }
}

// Dynamixel 테스트 함수
void testDynamixel(uint8_t id) {
  if (dxl_wb.ping(id)) {
    Serial.print("Dynamixel ID ");
    Serial.print(id);
    Serial.println(" is connected");
  } else {
    Serial.print("Failed to connect to Dynamixel ID ");
    Serial.println(id);
  }
}

// Dynamixel 모드 설정 함수
void setOperatingMode(uint8_t id, uint8_t mode) {
  if (dxl_wb.setOperatingMode(id, mode) == false) {
    Serial.print("Failed to set operating mode for Dynamixel ID ");
    Serial.println(id);
  } else {
    Serial.print("Successfully set operating mode for Dynamixel ID ");
    Serial.println(id);
  }
}

// Dynamixel 회전 함수
void rotateDynamixel(uint8_t id, float target_angle) {
  int32_t position = (target_angle / 360.0) * 4096;  // Convert angle to Dynamixel position
  dxl_wb.goalPosition(id, position);
}

// 모터1, 2 전개 함수
void extendMotors() {
  // Motor 1 and Motor 2 forward
  digitalWrite(motorPin1A, HIGH);
  digitalWrite(motorPin1B, LOW);
  digitalWrite(enablePin1, HIGH);
  digitalWrite(motorPin2A, HIGH);
  digitalWrite(motorPin2B, LOW);
  digitalWrite(enablePin2, HIGH);
}

// 모터1, 2 회수 함수
void retractMotors() {
  // Motor 1 and Motor 2 backward
  digitalWrite(motorPin1A, LOW);
  digitalWrite(motorPin1B, HIGH);
  digitalWrite(enablePin1, HIGH);
  digitalWrite(motorPin2A, LOW);
  digitalWrite(motorPin2B, HIGH);
  digitalWrite(enablePin2, HIGH);
}

void stopMotors() {
  digitalWrite(motorPin1A, LOW);
  digitalWrite(motorPin1B, LOW);
  digitalWrite(enablePin1, LOW);

  digitalWrite(motorPin2A, LOW);
  digitalWrite(motorPin2B, LOW);
  digitalWrite(enablePin2, LOW);
}

// Profile velocity와 acceleration 설정 함수
void setProfileVelocityAcceleration(uint8_t id, uint32_t velocity, uint32_t acceleration) {
  if (dxl_wb.itemWrite(id, "Profile_Velocity", velocity) == false) {
    Serial.print("Failed to set profile velocity for Dynamixel ID ");
    Serial.println(id);
  } else {
    Serial.print("Successfully set profile velocity for Dynamixel ID ");
    Serial.println(id);
  }

  if (dxl_wb.itemWrite(id, "Profile_Acceleration", acceleration) == false) {
    Serial.print("Failed to set profile acceleration for Dynamixel ID ");
    Serial.println(id);
  } else {
    Serial.print("Successfully set profile acceleration for Dynamixel ID ");
    Serial.println(id);
  }
}

void speed2acceleration() {
  motor3_current_speed = map(motor3_target_speed, 0, 100, 0, 255);  // Map the target speed to PWM value
  motor4_current_speed = map(motor4_target_speed, 0, 100, 0, 255);  // Map the target speed to PWM value

  analogWrite(enablePin1, motor3_current_speed);  // Set motor 3 speed
  analogWrite(enablePin2, motor4_current_speed);  // Set motor 4 speed

  Serial.print("Motor 3 Current Speed: ");
  Serial.print(motor3_current_speed);
  Serial.print(", Motor 4 Current Speed: ");
  Serial.println(motor4_current_speed);
}
