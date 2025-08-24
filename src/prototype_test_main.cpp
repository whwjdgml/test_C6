// 배터리 히터 비례 제어 프로토타입 테스트 코드
// 이 코드를 main.cpp에 통합하여 사용

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"

#include "sensor_config.h"
#include "ntc_sensor.h"
#include "proportional_battery_heater.h"

static const char *TAG = "HEATER_PROTOTYPE";

// 전역 객체들
static NTCSensor* battery_ntc = nullptr;
static ProportionalBatteryHeater* prop_heater = nullptr;

// 테스트 설정
static bool continuous_test_running = false;
static uint32_t test_measurement_count = 0;
static uint32_t test_start_time_ms = 0;

// 데이터 로깅용 구조체
struct HeaterTestData {
    uint32_t timestamp_ms;
    float battery_temp;
    uint8_t pwm_duty;
    heater_state_t heater_state;
    float power_consumption;
    bool stepup_active;
    float temperature_error;
};

// 최근 100개 데이터 저장
static HeaterTestData test_data_log[100];
static int log_index = 0;
static bool log_full = false;

// 콘솔 명령어 핸들러들
static int cmd_start_test(int argc, char **argv);
static int cmd_stop_test(int argc, char **argv);
static int cmd_tune_heater(int argc, char **argv);
static int cmd_show_status(int argc, char **argv);
static int cmd_show_log(int argc, char **argv);
static int cmd_export_csv(int argc, char **argv);
static int cmd_temperature_shock(int argc, char **argv);

// 콘솔 명령어 등록
void register_heater_commands() {
    const esp_console_cmd_t start_cmd = {
        .command = "start",
        .help = "연속 테스트 시작 (예: start 30 - 30분간 테스트)",
        .hint = "[duration_minutes]",
        .func = &cmd_start_test,
    };
    esp_console_cmd_register(&start_cmd);
    
    const esp_console_cmd_t stop_cmd = {
        .command = "stop",
        .help = "연속 테스트 중지",
        .hint = NULL,
        .func = &cmd_stop_test,
    };
    esp_console_cmd_register(&stop_cmd);
    
    const esp_console_cmd_t tune_cmd = {
        .command = "tune",
        .help = "히터 파라미터 튜닝 (예: tune kp=15, tune target=6)",
        .hint = "<parameter>=<value>",
        .func = &cmd_tune_heater,
    };
    esp_console_cmd_register(&tune_cmd);
    
    const esp_console_cmd_t status_cmd = {
        .command = "status",
        .help = "현재 히터 상태 출력",
        .hint = NULL,
        .func = &cmd_show_status,
    };
    esp_console_cmd_register(&status_cmd);
    
    const esp_console_cmd_t log_cmd = {
        .command = "log",
        .help = "최근 데이터 로그 출력 (예: log 20 - 최근 20개)",
        .hint = "[count]",
        .func = &cmd_show_log,
    };
    esp_console_cmd_register(&log_cmd);
    
    const esp_console_cmd_t csv_cmd = {
        .command = "csv",
        .help = "CSV 형태로 전체 로그 출력",
        .hint = NULL,
        .func = &cmd_export_csv,
    };
    esp_console_cmd_register(&csv_cmd);
    
    const esp_console_cmd_t shock_cmd = {
        .command = "shock",
        .help = "온도 쇼크 테스트 (급격한 목표온도 변경)",
        .hint = NULL,
        .func = &cmd_temperature_shock,
    };
    esp_console_cmd_register(&shock_cmd);
}

// 테스트 데이터 로깅
void log_test_data() {
    HeaterTestData* data = &test_data_log[log_index];
    
    data->timestamp_ms = esp_timer_get_time() / 1000;
    data->battery_temp = prop_heater->getLastTemperature();
    data->heater_state = prop_heater->getCurrentState();
    data->power_consumption = prop_heater->getCurrentPowerConsumption();
    data->stepup_active = prop_heater->isStepUpEnabled();
    data->temperature_error = prop_heater->getTargetTemperature() - data->battery_temp;
    
    // PWM 듀티 계산 (상태 기반)
    switch (data->heater_state) {
        case HEATER_OFF:  data->pwm_duty = 0; break;
        case HEATER_LOW:  data->pwm_duty = 25; break;  
        case HEATER_MED:  data->pwm_duty = 50; break;
        case HEATER_HIGH: data->pwm_duty = 75; break;
        case HEATER_MAX:  data->pwm_duty = 100; break;
        default: data->pwm_duty = 0; break;
    }
    
    log_index = (log_index + 1) % 100;
    if (log_index == 0) log_full = true;
}

// 연속 테스트 태스크
void continuous_test_task(void *pvParameters) {
    uint32_t test_duration_ms = (uint32_t)pvParameters;
    uint32_t next_measurement_time = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "🚀 연속 테스트 시작 - %lu분간 실행", test_duration_ms / 60000);
    test_start_time_ms = esp_timer_get_time() / 1000;
    test_measurement_count = 0;
    continuous_test_running = true;
    
    while (continuous_test_running) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // 테스트 시간 종료 체크
        if (test_duration_ms > 0 && 
            (current_time - test_start_time_ms) >= test_duration_ms) {
            ESP_LOGI(TAG, "⏰ 테스트 시간 종료 (%lu분)", test_duration_ms / 60000);
            break;
        }
        
        // 10초마다 측정
        if (current_time >= next_measurement_time) {
            test_measurement_count++;
            
            // 히터 업데이트
            if (prop_heater->updateProportionalHeater()) {
                // 데이터 로깅
                log_test_data();
                
                // 실시간 상태 출력 (1분마다)
                if (test_measurement_count % 6 == 1) {
                    float temp = prop_heater->getLastTemperature();
                    float target = prop_heater->getTargetTemperature();
                    float error = target - temp;
                    uint8_t duty = (prop_heater->getCurrentState() == HEATER_OFF) ? 0 : 
                                  (prop_heater->getCurrentState() == HEATER_LOW) ? 25 :
                                  (prop_heater->getCurrentState() == HEATER_MED) ? 50 :
                                  (prop_heater->getCurrentState() == HEATER_HIGH) ? 75 : 100;
                    
                    ESP_LOGI(TAG, "📊 [%lu분] 온도: %.2f°C (목표: %.1f°C, 오차: %+.2f°C) → %d%% PWM", 
                            (current_time - test_start_time_ms) / 60000, 
                            temp, target, error, duty);
                }
                
                // 상세 통계 (10분마다)  
                if (test_measurement_count % 60 == 1) {
                    prop_heater->printPerformanceStatistics();
                }
            } else {
                ESP_LOGW(TAG, "히터 업데이트 실패");
            }
            
            next_measurement_time = current_time + 10000; // 10초 후
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1초 대기
    }
    
    continuous_test_running = false;
    ESP_LOGI(TAG, "🏁 연속 테스트 완료 - 총 %lu회 측정", test_measurement_count);
    prop_heater->printPerformanceStatistics();
    
    vTaskDelete(NULL);
}

// 콘솔 명령어 구현들
static int cmd_start_test(int argc, char **argv) {
    if (continuous_test_running) {
        printf("이미 테스트가 실행 중입니다. 'stop' 명령으로 중지하세요.\n");
        return -1;
    }
    
    uint32_t duration_minutes = 30; // 기본 30분
    if (argc > 1) {
        duration_minutes = atoi(argv[1]);
    }
    
    uint32_t duration_ms = duration_minutes * 60000;
    
    xTaskCreate(continuous_test_task, "heater_test", 4096, 
                (void*)duration_ms, 5, NULL);
    
    printf("연속 테스트 시작: %lu분간 실행\n", duration_minutes);
    return 0;
}

static int cmd_stop_test(int argc, char **argv) {
    if (!continuous_test_running) {
        printf("실행 중인 테스트가 없습니다.\n");
        return -1;
    }
    
    continuous_test_running = false;
    printf("테스트 중지 요청됨\n");
    return 0;
}

static int cmd_tune_heater(int argc, char **argv) {
    if (argc < 2) {
        printf("사용법: tune <parameter>=<value>\n");
        printf("예시: tune kp=15, tune target=6\n");
        return -1;
    }
    
    prop_heater->handleTuningCommand(argv[1]);
    return 0;
}

static int cmd_show_status(int argc, char **argv) {
    prop_heater->printStatus();
    prop_heater->printControlParameters();
    return 0;
}

static int cmd_show_log(int argc, char **argv) {
    int count = 10; // 기본 10개
    if (argc > 1) {
        count = atoi(argv[1]);
    }
    
    if (count > 100) count = 100;
    
    printf("\n=== 최근 %d개 측정 데이터 ===\n", count);
    printf("시간(초)\t온도(°C)\t듀티(%%)\t전력(W)\t오차(°C)\n");
    
    int start_idx = log_full ? (log_index - count + 100) % 100 : 
                              (log_index - count < 0) ? 0 : log_index - count;
    
    for (int i = 0; i < count; i++) {
        int idx = (start_idx + i) % 100;
        if (!log_full && idx >= log_index) break;
        
        HeaterTestData* data = &test_data_log[idx];
        printf("%lu\t\t%.2f\t\t%d\t%.3f\t%+.2f\n",
               (data->timestamp_ms - test_start_time_ms) / 1000,
               data->battery_temp, data->pwm_duty, data->power_consumption,
               data->temperature_error);
    }
    
    return 0;
}

static int cmd_export_csv(int argc, char **argv) {
    printf("# 배터리 히터 테스트 데이터 (CSV)\n");
    printf("Time(s),Temperature(C),PWM_Duty(%%),Power(W),Error(C),StepUp\n");
    
    int total_count = log_full ? 100 : log_index;
    int start_idx = log_full ? log_index : 0;
    
    for (int i = 0; i < total_count; i++) {
        int idx = (start_idx + i) % 100;
        HeaterTestData* data = &test_data_log[idx];
        
        printf("%lu,%.2f,%d,%.3f,%+.2f,%s\n",
               (data->timestamp_ms - test_start_time_ms) / 1000,
               data->battery_temp, data->pwm_duty, data->power_consumption,
               data->temperature_error, data->stepup_active ? "ON" : "OFF");
    }
    
    return 0;
}

static int cmd_temperature_shock(int argc, char **argv) {
    printf("온도 쇼크 테스트 시작...\n");
    
    // 초기 목표온도 저장
    float original_target = prop_heater->getTargetTemperature();
    
    // 1단계: 고온 목표 (10°C)
    prop_heater->setTargetTemperature(10.0f);
    printf("1단계: 목표온도 10°C로 설정 (30초)\n");
    
    for (int i = 0; i < 6; i++) {
        prop_heater->updateProportionalHeater();
        log_test_data();
        printf("  %.2f°C → %s\n", 
               prop_heater->getLastTemperature(),
               (prop_heater->getCurrentState() == HEATER_OFF) ? "OFF" : "ON");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
    // 2단계: 저온 목표 (0°C)  
    prop_heater->setTargetTemperature(0.0f);
    printf("2단계: 목표온도 0°C로 설정 (30초)\n");
    
    for (int i = 0; i < 6; i++) {
        prop_heater->updateProportionalHeater();
        log_test_data();
        printf("  %.2f°C → %s\n", 
               prop_heater->getLastTemperature(),
               (prop_heater->getCurrentState() == HEATER_OFF) ? "OFF" : "ON");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
    // 원래 목표온도 복원
    prop_heater->setTargetTemperature(original_target);
    printf("목표온도 %.1f°C로 복원\n", original_target);
    
    printf("온도 쇼크 테스트 완료\n");
    return 0;
}

// 프로토타입 메인 함수
extern "C" void app_main() {
    ESP_LOGI(TAG, "🔥 배터리 히터 비례제어 프로토타입 시작");
    
    // UART 콘솔 초기화
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    
    esp_console_config_t console_config = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 8,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));
    
    // NTC 센서 초기화
    ESP_LOGI(TAG, "NTC 온도센서 초기화...");
    battery_ntc = new NTCSensor(BATTERY_NTC_ADC_CHANNEL, BATTERY_NTC_POWER_PIN);
    if (!battery_ntc->init()) {
        ESP_LOGE(TAG, "❌ NTC 센서 초기화 실패");
        return;
    }
    battery_ntc->printCalibrationInfo();
    
    // 비례 제어 히터 초기화  
    ESP_LOGI(TAG, "비례 제어 히터 초기화...");
    prop_heater = new ProportionalBatteryHeater(battery_ntc, 
                                               STEPUP_CONVERTER_EN_PIN,
                                               BATTERY_HEATER_PWM_PIN);
    if (!prop_heater->init()) {
        ESP_LOGE(TAG, "❌ 히터 초기화 실패");
        return;
    }
    
    // 초기 설정
    prop_heater->setTargetTemperature(5.0f);     // 목표 5°C
    prop_heater->setProportionalGain(15.0f);     // Kp = 15
    prop_heater->setDeadZone(0.5f);              // ±0.5°C 데드존
    prop_heater->setDutyLimits(0.0f, 100.0f);   // 0~100% 듀티
    
    prop_heater->printControlParameters();
    
    // 콘솔 명령어 등록
    register_heater_commands();
    
    ESP_LOGI(TAG, "✅ 프로토타입 초기화 완료!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📋 사용 가능한 명령어:");
    ESP_LOGI(TAG, "  start [분]     - 연속 테스트 시작");
    ESP_LOGI(TAG, "  stop          - 테스트 중지");  
    ESP_LOGI(TAG, "  status        - 현재 상태 확인");
    ESP_LOGI(TAG, "  tune kp=값    - 비례 게인 조정");
    ESP_LOGI(TAG, "  tune target=값 - 목표 온도 조정");
    ESP_LOGI(TAG, "  log [개수]    - 최근 데이터 확인");
    ESP_LOGI(TAG, "  csv           - CSV 출력");
    ESP_LOGI(TAG, "  shock         - 온도 쇼크 테스트");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "명령어를 입력하세요:");
    
    // 콘솔 루프
    char* line;
    while ((line = esp_console_linenoise("> ")) != NULL) {
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        
        if (err == ESP_ERR_NOT_FOUND) {
            printf("알 수 없는 명령어입니다.\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            printf("잘못된 인수입니다.\n");  
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("명령어 실행 오류\n");
        }
        
        linenoise_history_add(line);
        free(line);
    }
}