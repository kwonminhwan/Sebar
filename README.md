# Sebar

**Sebar**는 NVIDIA Jetson Xavier에서 **YOLOv5**를 활용해 가위/바위/보 손 모양을 인식하고, 이를 기반으로 **Arduino Mega + Dynamixel + DC 모터**를 제어하여 로봇 팔의 동작을 수행하는 프로젝트입니다.

---

## 📌 개요

- 🎥 YOLOv5로 실시간 제스처 인식 (가위, 바위, 보)
- 🔁 Jetson Xavier에서 Arduino Mega로 시리얼 명령 전송
- ⚙️ Dynamixel 모터로 팔 회전 제어
- 🌀 DC 모터로 팔 전개 및 회수 구현

---

## 🧠 시스템 구성

```
YOLOv5 (Jetson Xavier)
     ↓ [Serial]
FormChange.py (Python)
     ↓ [USB Serial]
Arduino Mega (Dynamixel 제어)
     ├── Dynamixel 모터 (ID 1, 2)
     └── DC 모터 (팔 전개 / 회수)
```

---

## 🎮 명령어 정의

| 명령 | 설명 |
|------|------|
| `E`  | 팔 전개 후 관절 회전 |
| `R`  | 팔 전개 후 초기 자세 복귀 |
| `S`  | 정지 → 회전 → 회수 |

---

## 📁 폴더 구조

```
Sebar/
├── FormChange/
│   ├── dynamixel_control.ino   # Arduino 코드
│   ├── FormChange.py           # YOLO 인식 결과 → Serial 전송
│
├── Yolo/
│   └── yolov5/                 # YOLOv5 전체 코드
│       ├── detect.py, train.py, val.py
│       ├── models/, utils/, data/
│       └── requirements.txt
│
└── README.md                   # 이 문서
```

---

## 🧩 기술 스택

- YOLOv5 (PyTorch 기반)
- Arduino Mega 2560
- Dynamixel X 시리즈
- DynamixelWorkbench
- Serial 통신 (115200 baud)
- ROS Noetic (옵션)

---

## 🧪 실행 방법

### 1. YOLO로 손 인식

```bash
cd Yolo/yolov5
python detect.py --weights best.pt --source 0
```

### 2. 시리얼 명령 전송

```bash
cd FormChange
python FormChange.py
```

### 3. Arduino 코드 업로드

Arduino IDE에서 `dynamixel_control.ino` 열기 → 보드 선택 → 업로드

---

## ⚒ Windows + MinGW 환경 설정

### 1. MinGW 설치

- [MinGW 다운로드](https://osdn.net/projects/mingw/releases/) → `mingw-get-setup.exe`
- 설치 시 `mingw32-gcc-g++` 선택
- `C:\MinGW\bin` 을 환경변수 PATH에 추가

### 2. VSCode 설정

`.vscode/c_cpp_properties.json` 생성 후 아래 내용 입력

```json
{
  "configurations": [
    {
      "name": "Win32",
      "compilerPath": "C:/MinGW/bin/gcc.exe",
      "intelliSenseMode": "gcc-x64",
      "includePath": ["${workspaceFolder}/**"],
      "cStandard": "c11",
      "cppStandard": "c++17"
    }
  ],
  "version": 4
}
```

---

## 🔧 Arduino 코드 요약 (dynamixel_control.ino)

- Dynamixel ID 1, 2의 초기 각도 설정
- 회전 제한: ±70도
- DC 모터: A/B 방향 핀 제어
- 명령 수신 포트: `Serial3` 또는 `/dev/ttyACM0`

---

## ⚠️ 참고 사항

- Dynamixel은 **12V 이상 외부 전원** 필요
- 시리얼 통신 속도는 **115200**으로 일치 필요
- 각도 및 위치 제한은 `rotateDynamixel()` 함수 내에서 조절 가능
- 포트 경로는 개발 환경에 맞게 조정

---

## 📸 시연 영상 
> https://youtu.be/3PRka_jRMKQ
---

## 🙋‍♀️ 제작자

- 권민환
- 역할: YOLOv5 연동, Dynamixel 제어, 전체 시스템 통합
- 참고 문헌: https://github.com/ultralytics/yolov5 
