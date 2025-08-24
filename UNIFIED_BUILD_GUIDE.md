# 🔄 통합 배터리 히터 시스템 빌드 가이드

## 📁 필요한 파일들 (최종 정리)

### ✅ 사용할 파일들:
```
src/
├── unified_main.cpp                 ✅ 통합 메인 (이것만 사용!)
├── ntc_sensor.h                     ✅ NTC 센서 클래스
├── ntc_sensor.cpp                   ✅ NTC 센서 구현
├── proportional_battery_heater.h    ✅ 비례제어 히터 클래스  
├── proportional_battery_heater.cpp  ✅ 비례제어 히터 구현
├── battery_heater.h                 ✅ 기본 히터 클래스
├── battery_heater.cpp               ✅ 기본 히터 구현
├── esp_now_manager.h                ✅ ESP-NOW 통신 클래스
├── esp_now_manager.cpp              ✅ ESP-NOW 통신 구현
├── sensor_config.h                  ✅ 설정 및 상수 (수정됨)
└── sensor_types.h                   ✅ 데이터 구조체 (수정됨)
```

### ❌ 사용하지 않을 파일들 (백업 또는 삭제):
```
src/
├── main.cpp                         ❌ 기존 메인 (백업)
├── prototype_test_main.cpp          ❌ 프로토타입용 (백업)
├── receiver_main.cpp                ❌ 수신기 전용 (백업)
├── heater_with_espnow.cpp           ❌ 통합 전 버전 (백업)
├── battery_heater_integration_example.cpp ❌ 예시 코드 (백업)
└── dynamic_heater_control_example.cpp     ❌ 예시 코드 (백업)
```

## 🔧 컴파일 방법

### 1. 기존 main.cpp 백업 및 교체
```bash
# 프로젝트 루트에서
cd src/
mv main.cpp main_backup.cpp
cp unified_main.cpp main.cpp
```

### 2. PlatformIO 빌드
```bash
pio run
```

### 3. 업로드 및 모니터링
```bash
pio run --target upload
pio device monitor
```

## 🎯 사용법 (하나의 빌드로 3가지 모드)

### 초기 설정
시스템 부팅 후 모드를 선택해야 합니다:

```bash
# 모드 선택 (필수!)
> mode heater    # 히터 전용 모드
> mode tx        # 송신기 모드 (히터+전송)  
> mode rx        # 수신기 모드 (모니터링)

# 현재 상태 확인
> info           # 모드 및 상태 정보
> help           # 전체 명령어 도움말
```

## 📋 모드별 명령어 정리

### 🔥 **히터 전용 모드 (mode heater)**
현장에서 히터만 사용, 무선 통신 없음
```bash
> start 120         # 2시간 테스트 시작
> status           # 히터 상태 확인
> tune kp=15       # 비례 게인 조정
> tune target=6    # 목표 온도 설정
> log 20           # 최근 20개 데이터 확인
> csv              # CSV 데이터 출력
```

### 📡 **송신기 모드 (mode tx)**  
현장용 히터 + 원격 모니터링 데이터 전송
```bash
> start 1440       # 24시간 자동 운영
> interval 60      # 1분마다 데이터 전송
> sendnow          # 즉시 데이터 전송
> comm             # 통신 통계 확인

# 히터 제어는 동일
> tune kp=20       # 게인 조정
> status           # 상태 확인
```

### 📺 **수신기 모드 (mode rx)**
실내 모니터링 스테이션, 히터 하드웨어 없음  
```bash
> monitor          # 실시간 모니터링 시작
🌡️ 4.8°C → 35% PWM (목표: 5.0°C) [ON]

> send target 7.0  # 원격으로 목표온도 변경
> send kp 25       # 원격으로 게인 변경
> status           # 원격 히터 상태 확인
> comm             # 수신 통계 확인
```

## 🚀 실제 사용 시나리오

### **시나리오 1: 현장 단독 테스트**
```bash
> mode heater      # 히터 전용 모드
> tune target=5    # 목표 5°C 설정
> start 60         # 1시간 테스트
> csv > data.csv   # 결과 저장
```

### **시나리오 2: 원격 모니터링 (2대 ESP32)**

**현장 송신기:**
```bash
> mode tx          # 송신기 모드
> start 1440       # 24시간 운영
> interval 30      # 30초마다 데이터 전송
```

**실내 수신기:**
```bash  
> mode rx          # 수신기 모드
> monitor          # 실시간 모니터링
> send target 6.0  # 원격 제어
```

### **시나리오 3: 파라미터 최적화**
```bash
> mode heater      # 히터 모드에서 시작
> tune kp=10       # 낮은 게인
> start 30         # 30분 테스트
> tune stats       # 성능 확인

> tune kp=20       # 게인 증가
> start 30         # 다시 테스트
> tune stats       # 성능 비교
```

## 💾 **설정 자동 저장**
- 선택한 모드는 NVS에 자동 저장됩니다
- 다음 부팅 시 저장된 모드로 제안됩니다
- `mode` 명령어로 언제든 변경 가능

## ⚡ **전력 최적화**
- **히터 모드**: 최소 전력 (ESP-NOW 없음)
- **송신기 모드**: 배치 전송으로 전력 절약
- **수신기 모드**: 히터 하드웨어 없음 (최소 전력)

## 🔧 **하드웨어 연결 (모드별)**

### 히터 모드 / 송신기 모드:
```
GPIO0: NTC ADC 입력
GPIO1: NTC 전원 제어  
GPIO2: PWM 출력
GPIO3: 스텝업 컨버터 EN
```

### 수신기 모드:
```
ESP-NOW만 사용 (GPIO 연결 불필요)
```

## 🎉 **통합 시스템의 장점**

1. **하나의 빌드**: 소스코드 중복 없음
2. **유연한 운영**: 상황에 따라 모드 전환
3. **설정 저장**: 다음 부팅 시 자동 복원  
4. **원격 제어**: 현장 방문 없이 튜닝 가능
5. **데이터 분석**: CSV 출력으로 Excel 분석

이제 **하나의 펌웨어로 모든 상황에 대응**할 수 있습니다! 🚀