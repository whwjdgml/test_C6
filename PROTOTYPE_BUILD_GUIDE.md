# 배터리 히터 비례제어 프로토타입 빌드 가이드

## 📁 필요한 파일들

### 새로 추가된 파일들:
```
src/
├── ntc_sensor.h                    ✅ NTC 센서 클래스
├── ntc_sensor.cpp                  ✅ NTC 센서 구현
├── battery_heater.h                ✅ 기본 히터 클래스  
├── battery_heater.cpp              ✅ 기본 히터 구현
├── proportional_battery_heater.h   ✅ 비례제어 히터 클래스
├── proportional_battery_heater.cpp ✅ 비례제어 히터 구현
└── prototype_test_main.cpp         ✅ 프로토타입 테스트 메인
```

### 수정된 기존 파일들:
```
src/
├── sensor_config.h    - 배터리 히터 핀 정의 추가
├── sensor_manager.h   - 히터 관련 함수 선언 추가  
└── sensor_types.h     - 배터리 히터 상태 구조체 추가
```

## 🔧 컴파일 방법

### 1. 기존 main.cpp 백업
```bash
cp src/main.cpp src/main_backup.cpp
```

### 2. 프로토타입 메인으로 교체
```bash
cp src/prototype_test_main.cpp src/main.cpp
```

### 3. PlatformIO 빌드
```bash
pio run
```

### 4. 업로드 및 모니터링
```bash
pio run --target upload
pio device monitor
```

## 🔌 하드웨어 연결

### ESP32-C6 핀 연결:
```
GPIO0  - NTC 센서 ADC 입력 (10kΩ 풀업 저항 + NTC to GND)
GPIO1  - NTC 센서 전원 제어 (HIGH = 전원 공급)
GPIO2  - 히터 PWM 출력 (5V 스텝업 컨버터 입력)
GPIO3  - 스텝업 컨버터 EN 핀 (HIGH = 활성화)
```

### NTC 센서 회로:
```
3.3V ----[10kΩ]----+---- GPIO0 (ADC)
                    |
                    [NTC 100kΩ 3950]
                    |
                   GND

GPIO1 ---- [NTC 전원 제어] (선택적)
```

### 5V 스텝업 컨버터:
```
3.7V 배터리 ---- 스텝업 컨버터 ---- 5V 히터
                      |
                   GPIO3 (EN)
                      |  
                   GPIO2 (PWM)
```

## 🧪 테스트 절차

### 1. 기본 동작 확인
```
> status          # 현재 상태 확인
> tune params     # 설정값 확인  
```

### 2. 단기 테스트 (10분)
```
> start 10        # 10분간 연속 테스트
> log 20          # 최근 20개 데이터 확인
```

### 3. 파라미터 튜닝
```
> tune kp=20      # 비례 게인 증가 (더 강한 반응)
> tune kp=10      # 비례 게인 감소 (더 부드러운 반응)
> tune target=6   # 목표 온도 6°C로 변경
> tune dead=0.3   # 데드존 ±0.3°C로 설정
```

### 4. 온도 응답 테스트
```
> shock           # 급격한 목표온도 변경 테스트
```

### 5. 장기 테스트 (2시간)
```
> start 120       # 2시간 연속 테스트
> csv             # 전체 데이터 CSV 출력
```

## 📊 성능 분석

### 좋은 비례 제어 지표:
- **평균 오차 < 1.0°C**: 정밀한 온도 제어
- **최대 오차 < 2.0°C**: 안정적인 제어
- **가열 사이클 적음**: 효율적인 제어
- **전력 소모 일정**: 예측 가능한 동작

### 튜닝 가이드:
```
Kp 값이 너무 작으면 → 반응이 느림, 목표온도 도달 지연
Kp 값이 너무 크면 → 오버슛, 진동 발생

권장 시작값:
- Kp = 15 (기본값)
- Target = 5°C (리튬배터리 최적)  
- Dead Zone = ±0.5°C (노이즈 제거)
```

## 🚨 문제 해결

### 컴파일 에러:
1. `#include "console.h"` 에러 → ESP-IDF 콘솔 컴포넌트 누락
2. ADC 관련 에러 → ESP-IDF 버전 확인 (5.0 이상 필요)

### 런타임 에러:
1. NTC 센서 초기화 실패 → GPIO 핀 연결 확인
2. PWM 출력 없음 → LEDC 드라이버 설정 확인
3. 온도 값 이상 → NTC 캘리브레이션 파라미터 확인

## 🎯 기대 결과

### 성공적인 비례 제어:
- 목표온도 5°C 기준 ±1°C 이내 유지
- 외부온도 변화에 자동 적응
- 전력소모 기존 히스테리시스 대비 20-30% 절약
- 부드러운 온도 변화 곡선

이 프로토타입으로 실제 온도 변화를 측정하고 비례 게인을 최적화하면 
매우 효율적인 배터리 히터 시스템을 구축할 수 있습니다!