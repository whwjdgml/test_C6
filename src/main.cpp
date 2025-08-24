// 통합 배터리 히터 시스템
// 하나의 빌드로 송신기/수신기 모드 전환 가능

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "sensor_config.h"
#include "ntc_sensor.h"
#include "proportional_battery_heater.h"
#include "esp_now_manager.h"

static const char *TAG = "UNIFIED_HEATER";

// 시스템 모드
typedef enum {
    MODE_HEATER_ONLY,     // 히터만 (ESP-NOW 없음)
    MODE_TRANSMITTER,     // 송신기 (히터 + 데이터 전송)
    MODE_RECEIVER,        // 수신기 (데이터 수신 + 모니터링)
    MODE_UNDEFINED        // 미정의 (최초 실행)
} system_mode_t;

// 전역 변수들
static system_mode_t current_mode = MODE_UNDEFINED;
static NTCSensor* battery_ntc = nullptr;
static ProportionalBatteryHeater* prop_heater = nullptr;
static ESPNowManager* espnow_mgr = nullptr;

// 송신기 모드 변수들
static bool continuous_test_running = false;
static uint32_t test_measurement_count = 0;
static uint32_t data_send_interval_ms = 30000;  // 30초
static uint32_t last_data_send_ms = 0;
static uint32_t last_heartbeat_ms = 0;

// 수신기 모드 변수들
static HeaterStatusPacket latest_heater_status = {};
static bool heater_data_received = false;
static uint32_t total_packets_received = 0;
static bool real_time_monitoring = false;

// 데이터 로깅
struct TestDataLog {
    uint32_t timestamp_ms;
    float battery_temp;
    uint8_t pwm_duty;
    float temperature_error;
    bool stepup_active;
};
static TestDataLog test_data_log[100];
static int log_index = 0;
static bool log_full = false;

// NVS에서 모드 저장/로드
void saveCurrentMode() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("heater_config", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_u8(nvs_handle, "system_mode", (uint8_t)current_mode);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "모드 저장: %d", current_mode);
    }
}

system_mode_t loadSavedMode() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("heater_config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        uint8_t saved_mode = 0;
        size_t required_size = sizeof(uint8_t);
        err = nvs_get_u8(nvs_handle, "system_mode", &saved_mode);
        nvs_close(nvs_handle);
        
        if (err == ESP_OK && saved_mode < MODE_UNDEFINED) {
            ESP_LOGI(TAG, "저장된 모드 로드: %d", saved_mode);
            return (system_mode_t)saved_mode;
        }
    }
    return MODE_UNDEFINED;
}

// 모드별 초기화 함수들
bool initHeaterMode();
bool initTransmitterMode();
bool initReceiverMode();
void deinitCurrentMode();

// 콜백 함수들
void onRemoteCommandReceived(const uint8_t *mac, const uint8_t *data, int len);
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len);
void onDataSendComplete(const uint8_t *mac, esp_now_send_status_t status);

// 모드별 태스크들
void heater_only_task(void *pvParameters);
void transmitter_task(void *pvParameters);
void receiver_task(void *pvParameters);

// 공통 유틸리티 함수들
void log_test_data();
void sendHeaterStatusPacket();
const char* getModeString(system_mode_t mode);

// 콘솔 명령어들
static int cmd_set_mode(int argc, char **argv);
static int cmd_show_mode(int argc, char **argv);
static int cmd_start_test(int argc, char **argv);
static int cmd_stop_test(int argc, char **argv);
static int cmd_tune_heater(int argc, char **argv);
static int cmd_show_status(int argc, char **argv);
static int cmd_show_log(int argc, char **argv);
static int cmd_export_csv(int argc, char **argv);
static int cmd_monitor(int argc, char **argv);
static int cmd_send_command(int argc, char **argv);
static int cmd_comm_stats(int argc, char **argv);
static int cmd_set_interval(int argc, char **argv);
static int cmd_send_now(int argc, char **argv);
static int cmd_help(int argc, char **argv);

// 콘솔 명령어 등록
void register_unified_commands() {
    // 시스템 모드 관련
    const esp_console_cmd_t mode_cmd = {
        .command = "mode",
        .help = "시스템 모드 설정 (heater/tx/rx)",
        .hint = "[heater|tx|rx]",
        .func = &cmd_set_mode,
    };
    esp_console_cmd_register(&mode_cmd);
    
    const esp_console_cmd_t info_cmd = {
        .command = "info",
        .help = "현재 모드 및 상태 정보 출력",
        .hint = NULL,
        .func = &cmd_show_mode,
    };
    esp_console_cmd_register(&info_cmd);
    
    // 공통 명령어들
    const esp_console_cmd_t start_cmd = {
        .command = "start",
        .help = "테스트 시작 (분 단위)",
        .hint = "[duration_minutes]",
        .func = &cmd_start_test,
    };
    esp_console_cmd_register(&start_cmd);
    
    const esp_console_cmd_t stop_cmd = {
        .command = "stop",
        .help = "테스트 중지",
        .hint = NULL,
        .func = &cmd_stop_test,
    };
    esp_console_cmd_register(&stop_cmd);
    
    const esp_console_cmd_t status_cmd = {
        .command = "status",
        .help = "현재 상태 출력",
        .hint = NULL,
        .func = &cmd_show_status,
    };
    esp_console_cmd_register(&status_cmd);
    
    const esp_console_cmd_t tune_cmd = {
        .command = "tune",
        .help = "히터 파라미터 튜닝",
        .hint = "<parameter>=<value>",
        .func = &cmd_tune_heater,
    };
    esp_console_cmd_register(&tune_cmd);
    
    const esp_console_cmd_t log_cmd = {
        .command = "log",
        .help = "데이터 로그 출력",
        .hint = "[count]",
        .func = &cmd_show_log,
    };
    esp_console_cmd_register(&log_cmd);
    
    const esp_console_cmd_t csv_cmd = {
        .command = "csv",
        .help = "CSV 형태로 데이터 출력",
        .hint = NULL,
        .func = &cmd_export_csv,
    };
    esp_console_cmd_register(&csv_cmd);
    
    // 수신기 모드 전용
    const esp_console_cmd_t monitor_cmd = {
        .command = "monitor",
        .help = "실시간 모니터링 (수신기 모드)",
        .hint = NULL,
        .func = &cmd_monitor,
    };
    esp_console_cmd_register(&monitor_cmd);
    
    const esp_console_cmd_t send_cmd = {
        .command = "send",
        .help = "원격 명령 전송 (수신기 모드)",
        .hint = "<command> <value>",
        .func = &cmd_send_command,
    };
    esp_console_cmd_register(&send_cmd);
    
    // ESP-NOW 관련
    const esp_console_cmd_t comm_cmd = {
        .command = "comm",
        .help = "통신 통계 출력",
        .hint = NULL,
        .func = &cmd_comm_stats,
    };
    esp_console_cmd_register(&comm_cmd);
    
    const esp_console_cmd_t interval_cmd = {
        .command = "interval",
        .help = "데이터 전송 간격 설정 (송신기 모드)",
        .hint = "<seconds>",
        .func = &cmd_set_interval,
    };
    esp_console_cmd_register(&interval_cmd);
    
    const esp_console_cmd_t sendnow_cmd = {
        .command = "sendnow",
        .help = "즉시 데이터 전송 (송신기 모드)",
        .hint = NULL,
        .func = &cmd_send_now,
    };
    esp_console_cmd_register(&sendnow_cmd);
    
    const esp_console_cmd_t help_cmd = {
        .command = "help",
        .help = "도움말 출력",
        .hint = NULL,
        .func = &cmd_help,
    };
    esp_console_cmd_register(&help_cmd);
}

// 모드 초기화 함수들
bool initHeaterMode() {
    ESP_LOGI(TAG, "히터 전용 모드 초기화...");
    
    // NTC 센서 초기화
    battery_ntc = new NTCSensor(BATTERY_NTC_ADC_CHANNEL, BATTERY_NTC_POWER_PIN);
    if (!battery_ntc->init()) {
        ESP_LOGE(TAG, "NTC 센서 초기화 실패");
        return false;
    }
    
    // 히터 초기화
    prop_heater = new ProportionalBatteryHeater(battery_ntc, STEPUP_CONVERTER_EN_PIN, BATTERY_HEATER_PWM_PIN);
    if (!prop_heater->init()) {
        ESP_LOGE(TAG, "히터 초기화 실패");
        return false;
    }
    
    // 기본 설정
    prop_heater->setTargetTemperature(5.0f);
    prop_heater->setProportionalGain(15.0f);
    
    // 백그라운드 태스크 시작
    xTaskCreate(heater_only_task, "heater_only", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "✅ 히터 전용 모드 초기화 완료");
    return true;
}

bool initTransmitterMode() {
    ESP_LOGI(TAG, "송신기 모드 초기화...");
    
    // 히터 초기화 (히터 모드와 동일)
    if (!initHeaterMode()) {
        return false;
    }
    
    // ESP-NOW 초기화
    espnow_mgr = new ESPNowManager();
    if (!espnow_mgr->init()) {
        ESP_LOGE(TAG, "ESP-NOW 초기화 실패");
        return false;
    }
    
    espnow_mgr->setDataReceivedCallback(onRemoteCommandReceived);
    espnow_mgr->setSendCompleteCallback(onDataSendComplete);
    
    // 송신기 태스크로 전환 (기존 히터 태스크 종료 필요)
    xTaskCreate(transmitter_task, "transmitter", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "✅ 송신기 모드 초기화 완료");
    return true;
}

bool initReceiverMode() {
    ESP_LOGI(TAG, "수신기 모드 초기화...");
    
    // ESP-NOW만 초기화 (히터 없음)
    espnow_mgr = new ESPNowManager();
    if (!espnow_mgr->init()) {
        ESP_LOGE(TAG, "ESP-NOW 초기화 실패");
        return false;
    }
    
    espnow_mgr->setDataReceivedCallback(onDataReceived);
    espnow_mgr->setSendCompleteCallback(onDataSendComplete);
    
    xTaskCreate(receiver_task, "receiver", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "✅ 수신기 모드 초기화 완료");
    return true;
}

void deinitCurrentMode() {
    ESP_LOGI(TAG, "현재 모드 정리 중...");
    
    continuous_test_running = false;
    real_time_monitoring = false;
    
    vTaskDelay(pdMS_TO_TICKS(1000)); // 태스크 종료 대기
    
    if (prop_heater) {
        prop_heater->emergencyShutdown();
        delete prop_heater;
        prop_heater = nullptr;
    }
    
    if (battery_ntc) {
        delete battery_ntc;
        battery_ntc = nullptr;
    }
    
    if (espnow_mgr) {
        espnow_mgr->deinit();
        delete espnow_mgr;
        espnow_mgr = nullptr;
    }
    
    ESP_LOGI(TAG, "모드 정리 완료");
}

// 태스크 함수들
void heater_only_task(void *pvParameters) {
    ESP_LOGI(TAG, "히터 전용 태스크 시작");
    
    while (current_mode == MODE_HEATER_ONLY) {
        if (continuous_test_running && prop_heater) {
            test_measurement_count++;
            
            if (prop_heater->updateProportionalHeater()) {
                log_test_data();
                
                if (test_measurement_count % 6 == 1) { // 1분마다
                    float temp = prop_heater->getLastTemperature();
                    float target = prop_heater->getTargetTemperature();
                    ESP_LOGI(TAG, "🌡️ %.2f°C → %s (목표: %.1f°C)", 
                            temp, (prop_heater->getCurrentState() == HEATER_OFF) ? "OFF" : "ON", target);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10초
    }
    
    ESP_LOGI(TAG, "히터 전용 태스크 종료");
    vTaskDelete(NULL);
}

void transmitter_task(void *pvParameters) {
    ESP_LOGI(TAG, "송신기 태스크 시작");
    
    while (current_mode == MODE_TRANSMITTER) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        if (continuous_test_running && prop_heater) {
            test_measurement_count++;
            
            if (prop_heater->updateProportionalHeater()) {
                log_test_data();
                
                // 주기적 데이터 전송
                if (current_time - last_data_send_ms >= data_send_interval_ms) {
                    sendHeaterStatusPacket();
                    last_data_send_ms = current_time;
                }
                
                // 하트비트 (2분마다)
                if (current_time - last_heartbeat_ms >= 120000) {
                    if (espnow_mgr) espnow_mgr->sendHeartbeat();
                    last_heartbeat_ms = current_time;
                }
                
                if (test_measurement_count % 6 == 1) {
                    float temp = prop_heater->getLastTemperature();
                    ESP_LOGI(TAG, "📡 %.2f°C → 송신 중", temp);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10초
    }
    
    ESP_LOGI(TAG, "송신기 태스크 종료");
    vTaskDelete(NULL);
}

void receiver_task(void *pvParameters) {
    ESP_LOGI(TAG, "수신기 태스크 시작");
    
    while (current_mode == MODE_RECEIVER) {
        // 수신기는 주로 콜백 기반으로 동작
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
    ESP_LOGI(TAG, "수신기 태스크 종료");
    vTaskDelete(NULL);
}

// 콜백 함수들
void onRemoteCommandReceived(const uint8_t *mac, const uint8_t *data, int len) {
    // 송신기 모드에서 원격 명령 수신 처리
    if (len >= sizeof(ESPNowPacketHeader)) {
        ESPNowPacketHeader* header = (ESPNowPacketHeader*)data;
        if (header->packet_type == PACKET_CONTROL_COMMAND) {
            ControlCommandPacket* cmd = (ControlCommandPacket*)data;
            
            switch (cmd->command_type) {
                case 1: // SET_TARGET_TEMP
                    if (prop_heater) {
                        prop_heater->setTargetTemperature(cmd->data.set_target.new_target_temp);
                        ESP_LOGI(TAG, "🎯 원격 목표온도 변경: %.1f°C", cmd->data.set_target.new_target_temp);
                    }
                    break;
                case 2: // SET_GAIN
                    if (prop_heater) {
                        prop_heater->setProportionalGain(cmd->data.set_gain.new_kp);
                        ESP_LOGI(TAG, "⚙️ 원격 게인 변경: %.1f", cmd->data.set_gain.new_kp);
                    }
                    break;
            }
            
            // 즉시 상태 전송
            sendHeaterStatusPacket();
        }
    }
}

void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    // 수신기 모드에서 데이터 수신 처리
    if (len < sizeof(ESPNowPacketHeader)) return;
    
    ESPNowPacketHeader* header = (ESPNowPacketHeader*)data;
    total_packets_received++;
    
    switch (header->packet_type) {
        case PACKET_HEATER_STATUS:
            if (len == sizeof(HeaterStatusPacket)) {
                memcpy(&latest_heater_status, data, sizeof(HeaterStatusPacket));
                heater_data_received = true;
                
                if (real_time_monitoring) {
                    printf("\r🌡️ %.2f°C → %d%% PWM (목표: %.1f°C) [%s]     ",
                           latest_heater_status.battery_temperature,
                           latest_heater_status.pwm_duty_percent,
                           latest_heater_status.target_temperature,
                           latest_heater_status.stepup_enabled ? "ON" : "OFF");
                    fflush(stdout);
                }
            }
            break;
    }
}

void onDataSendComplete(const uint8_t *mac, esp_now_send_status_t status) {
    // 전송 완료 처리
    ESP_LOGV(TAG, "전송 %s", (status == ESP_NOW_SEND_SUCCESS) ? "성공" : "실패");
}

// 유틸리티 함수들
void log_test_data() {
    if (!prop_heater) return;
    
    TestDataLog* data = &test_data_log[log_index];
    data->timestamp_ms = esp_timer_get_time() / 1000;
    data->battery_temp = prop_heater->getLastTemperature();
    data->temperature_error = prop_heater->getTargetTemperature() - data->battery_temp;
    data->stepup_active = prop_heater->isStepUpEnabled();
    
    // PWM 듀티 계산
    switch (prop_heater->getCurrentState()) {
        case HEATER_LOW:  data->pwm_duty = 25; break;
        case HEATER_MED:  data->pwm_duty = 50; break;
        case HEATER_HIGH: data->pwm_duty = 75; break;
        case HEATER_MAX:  data->pwm_duty = 100; break;
        default: data->pwm_duty = 0; break;
    }
    
    log_index = (log_index + 1) % 100;
    if (log_index == 0) log_full = true;
}

void sendHeaterStatusPacket() {
    if (!espnow_mgr || !prop_heater) return;
    
    HeaterStatusPacket packet = {};
    espnow_mgr->fillPacketHeader(&packet.header, PACKET_HEATER_STATUS, 
                                sizeof(packet) - sizeof(ESPNowPacketHeader));
    
    packet.battery_temperature = prop_heater->getLastTemperature();
    packet.target_temperature = prop_heater->getTargetTemperature();
    packet.heater_state = prop_heater->getCurrentState();
    packet.power_consumption = prop_heater->getCurrentPowerConsumption();
    packet.stepup_enabled = prop_heater->isStepUpEnabled();
    packet.proportional_gain = prop_heater->getProportionalGain();
    packet.average_error = prop_heater->getAverageError();
    packet.heating_cycles = prop_heater->getHeatingCycles();
    packet.total_heating_time_sec = prop_heater->getTotalHeatingTimeMs() / 1000;
    
    // PWM 듀티 계산
    switch (packet.heater_state) {
        case HEATER_LOW:  packet.pwm_duty_percent = 25; break;
        case HEATER_MED:  packet.pwm_duty_percent = 50; break;
        case HEATER_HIGH: packet.pwm_duty_percent = 75; break;
        case HEATER_MAX:  packet.pwm_duty_percent = 100; break;
        default: packet.pwm_duty_percent = 0; break;
    }
    
    espnow_mgr->sendHeaterStatus(packet);
}

const char* getModeString(system_mode_t mode) {
    switch (mode) {
        case MODE_HEATER_ONLY: return "히터 전용";
        case MODE_TRANSMITTER: return "송신기 (히터+전송)";
        case MODE_RECEIVER: return "수신기 (모니터링)";
        case MODE_UNDEFINED: return "미정의";
        default: return "알 수 없음";
    }
}

// 콘솔 명령어 구현들 (다음에 계속...)
static int cmd_set_mode(int argc, char **argv) {
    if (argc < 2) {
        printf("현재 모드: %s\n", getModeString(current_mode));
        printf("사용법: mode <heater|tx|rx>\n");
        printf("  heater - 히터 전용 모드\n");
        printf("  tx     - 송신기 모드 (히터+전송)\n");
        printf("  rx     - 수신기 모드 (모니터링)\n");
        return 0;
    }
    
    system_mode_t new_mode = MODE_UNDEFINED;
    
    if (strcmp(argv[1], "heater") == 0) {
        new_mode = MODE_HEATER_ONLY;
    } else if (strcmp(argv[1], "tx") == 0) {
        new_mode = MODE_TRANSMITTER;
    } else if (strcmp(argv[1], "rx") == 0) {
        new_mode = MODE_RECEIVER;
    } else {
        printf("잘못된 모드: %s\n", argv[1]);
        return -1;
    }
    
    if (new_mode == current_mode) {
        printf("이미 %s 모드입니다\n", getModeString(new_mode));
        return 0;
    }
    
    printf("모드 변경: %s → %s\n", getModeString(current_mode), getModeString(new_mode));
    
    // 기존 모드 정리
    deinitCurrentMode();
    
    // 새 모드 초기화
    current_mode = new_mode;
    bool init_success = false;
    
    switch (new_mode) {
        case MODE_HEATER_ONLY:
            init_success = initHeaterMode();
            break;
        case MODE_TRANSMITTER:
            init_success = initTransmitterMode();
            break;
        case MODE_RECEIVER:
            init_success = initReceiverMode();
            break;
        default:
            break;
    }
    
    if (init_success) {
        saveCurrentMode();
        printf("✅ %s 모드로 전환 완료\n", getModeString(new_mode));
    } else {
        printf("❌ 모드 전환 실패\n");
        current_mode = MODE_UNDEFINED;
    }
    
    return init_success ? 0 : -1;
}

static int cmd_show_mode(int argc, char **argv) {
    printf("\n=== 시스템 정보 ===\n");
    printf("현재 모드: %s\n", getModeString(current_mode));
    
    if (current_mode != MODE_RECEIVER && prop_heater) {
        printf("히터 상태: %s\n", prop_heater->getCurrentState() == HEATER_OFF ? "OFF" : "ON");
        printf("배터리 온도: %.2f°C\n", prop_heater->getLastTemperature());
        printf("목표 온도: %.1f°C\n", prop_heater->getTargetTemperature());
        printf("비례 게인: %.1f\n", prop_heater->getProportionalGain());
    }
    
    if (current_mode != MODE_HEATER_ONLY && espnow_mgr) {
        printf("ESP-NOW: 초기화됨\n");
        printf("성공률: %.1f%%\n", espnow_mgr->getSuccessRate());
    }
    
    if (current_mode == MODE_RECEIVER) {
        printf("수신된 패킷: %lu개\n", total_packets_received);
        printf("최신 데이터: %s\n", heater_data_received ? "있음" : "없음");
    }
    
    printf("테스트 실행 중: %s\n", continuous_test_running ? "예" : "아니오");
    
    return 0;
}

// 나머지 명령어들은 기존과 유사하게 구현...
static int cmd_start_test(int argc, char **argv) {
    if (current_mode == MODE_RECEIVER) {
        printf("수신기 모드에서는 테스트를 시작할 수 없습니다\n");
        return -1;
    }
    
    uint32_t duration_minutes = 30;
    if (argc > 1) {
        duration_minutes = atoi(argv[1]);
    }
    
    continuous_test_running = true;
    test_measurement_count = 0;
    
    printf("%s 모드에서 %lu분간 테스트 시작\n", 
           getModeString(current_mode), duration_minutes);
    
    return 0;
}

static int cmd_stop_test(int argc, char **argv) {
    if (!continuous_test_running) {
        printf("실행 중인 테스트가 없습니다\n");
        return -1;
    }
    
    continuous_test_running = false;
    printf("테스트 중지\n");
    
    if (prop_heater) {
        prop_heater->printPerformanceStatistics();
    }
    
    return 0;
}

static int cmd_tune_heater(int argc, char **argv) {
    if (current_mode == MODE_RECEIVER || !prop_heater) {
        printf("히터가 없는 모드입니다\n");
        return -1;
    }
    
    if (argc < 2) {
        prop_heater->printControlParameters();
        return 0;
    }
    
    prop_heater->handleTuningCommand(argv[1]);
    return 0;
}

static int cmd_show_status(int argc, char **argv) {
    if (current_mode == MODE_RECEIVER) {
        if (!heater_data_received) {
            printf("수신된 히터 데이터가 없습니다\n");
            return -1;
        }
        
        printf("\n=== 원격 히터 상태 ===\n");
        printf("배터리 온도: %.2f°C\n", latest_heater_status.battery_temperature);
        printf("목표 온도: %.1f°C\n", latest_heater_status.target_temperature);
        printf("PWM 듀티: %d%%\n", latest_heater_status.pwm_duty_percent);
        printf("전력 소모: %.3fW\n", latest_heater_status.power_consumption);
        printf("스텝업 컨버터: %s\n", latest_heater_status.stepup_enabled ? "ON" : "OFF");
    } else if (prop_heater) {
        prop_heater->printStatus();
    } else {
        printf("상태 정보가 없습니다\n");
    }
    
    return 0;
}

static int cmd_show_log(int argc, char **argv) {
    int count = 10;
    if (argc > 1) {
        count = atoi(argv[1]);
    }
    if (count > 100) count = 100;
    
    printf("\n=== 최근 %d개 측정 데이터 ===\n", count);
    printf("시간(초)\t온도(°C)\t듀티(%%)\t오차(°C)\n");
    
    int start_idx = log_full ? (log_index - count + 100) % 100 : 
                              (log_index - count < 0) ? 0 : log_index - count;
    
    for (int i = 0; i < count; i++) {
        int idx = (start_idx + i) % 100;
        if (!log_full && idx >= log_index) break;
        
        TestDataLog* data = &test_data_log[idx];
        printf("%lu\t\t%.2f\t\t%d\t%+.2f\n",
               data->timestamp_ms / 1000, data->battery_temp, 
               data->pwm_duty, data->temperature_error);
    }
    
    return 0;
}

static int cmd_export_csv(int argc, char **argv) {
    printf("# 배터리 히터 테스트 데이터 (CSV)\n");
    printf("Time(s),Temperature(C),PWM_Duty(%%),Error(C),StepUp\n");
    
    int total_count = log_full ? 100 : log_index;
    int start_idx = log_full ? log_index : 0;
    
    for (int i = 0; i < total_count; i++) {
        int idx = (start_idx + i) % 100;
        TestDataLog* data = &test_data_log[idx];
        
        printf("%lu,%.2f,%d,%+.2f,%s\n",
               data->timestamp_ms / 1000, data->battery_temp,
               data->pwm_duty, data->temperature_error,
               data->stepup_active ? "ON" : "OFF");
    }
    
    return 0;
}

static int cmd_monitor(int argc, char **argv) {
    if (current_mode != MODE_RECEIVER) {
        printf("수신기 모드에서만 사용 가능합니다\n");
        return -1;
    }
    
    real_time_monitoring = true;
    printf("실시간 모니터링 시작 (stop으로 중지)\n");
    return 0;
}

static int cmd_send_command(int argc, char **argv) {
    if (current_mode != MODE_RECEIVER || !espnow_mgr) {
        printf("수신기 모드에서만 사용 가능합니다\n");
        return -1;
    }
    
    if (argc < 3) {
        printf("사용법: send <command> <value>\n");
        printf("예시:\n");
        printf("  send target 6.0  - 목표온도 변경\n");
        printf("  send kp 20       - 비례게인 변경\n");
        return -1;
    }
    
    ControlCommandPacket cmd_packet = {};
    espnow_mgr->fillPacketHeader(&cmd_packet.header, PACKET_CONTROL_COMMAND, 
                                sizeof(cmd_packet) - sizeof(ESPNowPacketHeader));
    
    if (strcmp(argv[1], "target") == 0) {
        cmd_packet.command_type = 1;
        cmd_packet.data.set_target.new_target_temp = atof(argv[2]);
        printf("목표온도 변경 명령 전송: %.1f°C\n", cmd_packet.data.set_target.new_target_temp);
    } else if (strcmp(argv[1], "kp") == 0) {
        cmd_packet.command_type = 2;
        cmd_packet.data.set_gain.new_kp = atof(argv[2]);
        printf("비례게인 변경 명령 전송: %.1f\n", cmd_packet.data.set_gain.new_kp);
    } else {
        printf("알 수 없는 명령: %s\n", argv[1]);
        return -1;
    }
    
    // TODO: 실제 전송 구현 필요
    return 0;
}

static int cmd_comm_stats(int argc, char **argv) {
    if (!espnow_mgr) {
        printf("ESP-NOW가 초기화되지 않음\n");
        return -1;
    }
    
    espnow_mgr->printStatistics();
    printf("데이터 전송 간격: %lu초\n", data_send_interval_ms / 1000);
    return 0;
}

static int cmd_set_interval(int argc, char **argv) {
    if (current_mode != MODE_TRANSMITTER) {
        printf("송신기 모드에서만 사용 가능합니다\n");
        return -1;
    }
    
    if (argc < 2) {
        printf("현재 전송 간격: %lu초\n", data_send_interval_ms / 1000);
        return 0;
    }
    
    uint32_t interval_sec = atoi(argv[1]);
    if (interval_sec >= 5 && interval_sec <= 300) {
        data_send_interval_ms = interval_sec * 1000;
        printf("전송 간격 변경: %lu초\n", interval_sec);
    } else {
        printf("잘못된 간격 (5~300초)\n");
    }
    
    return 0;
}

static int cmd_send_now(int argc, char **argv) {
    if (current_mode != MODE_TRANSMITTER || !espnow_mgr) {
        printf("송신기 모드에서만 사용 가능합니다\n");
        return -1;
    }
    
    printf("즉시 데이터 전송 중...\n");
    sendHeaterStatusPacket();
    espnow_mgr->sendHeartbeat();
    
    return 0;
}

static int cmd_help(int argc, char **argv) {
    printf("\n=== 통합 배터리 히터 시스템 명령어 ===\n");
    printf("\n🔧 시스템 제어:\n");
    printf("  mode <모드>     - 모드 전환 (heater/tx/rx)\n");
    printf("  info           - 현재 모드 및 상태 정보\n");
    printf("  help           - 이 도움말 출력\n");
    
    printf("\n🔥 히터 제어 (heater/tx 모드):\n");
    printf("  start [분]     - 테스트 시작\n");
    printf("  stop           - 테스트 중지\n");
    printf("  status         - 현재 상태 출력\n");
    printf("  tune <설정>    - 파라미터 튜닝\n");
    printf("    예: tune kp=15, tune target=6\n");
    
    printf("\n📊 데이터 분석:\n");
    printf("  log [개수]     - 최근 데이터 출력\n");
    printf("  csv            - CSV 형태로 전체 데이터 출력\n");
    
    printf("\n📡 송신기 모드 (tx):\n");
    printf("  interval <초>  - 데이터 전송 간격 설정\n");
    printf("  sendnow        - 즉시 데이터 전송\n");
    printf("  comm           - 통신 통계 출력\n");
    
    printf("\n📺 수신기 모드 (rx):\n");
    printf("  monitor        - 실시간 모니터링\n");
    printf("  send <명령>    - 원격 제어 명령 전송\n");
    printf("    예: send target 6.0, send kp 20\n");
    
    printf("\n현재 모드: %s\n", getModeString(current_mode));
    
    return 0;
}

// 메인 함수
extern "C" void app_main() {
    ESP_LOGI(TAG, "🔄 통합 배터리 히터 시스템 시작");
    
    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 콘솔 초기화
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    
    esp_console_config_t console_config = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 8,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));
    
    // 콘솔 명령어 등록
    register_unified_commands();
    
    // 저장된 모드 로드
    system_mode_t saved_mode = loadSavedMode();
    
    ESP_LOGI(TAG, "✅ 시스템 초기화 완료");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🎯 모드 설정이 필요합니다!");
    ESP_LOGI(TAG, "명령어: mode <heater|tx|rx>");
    ESP_LOGI(TAG, "  heater - 히터 전용 모드");
    ESP_LOGI(TAG, "  tx     - 송신기 모드 (히터+전송)");
    ESP_LOGI(TAG, "  rx     - 수신기 모드 (모니터링)");
    
    if (saved_mode != MODE_UNDEFINED) {
        ESP_LOGI(TAG, "저장된 모드 (%s)를 사용하려면: mode %s", 
                getModeString(saved_mode),
                (saved_mode == MODE_HEATER_ONLY) ? "heater" :
                (saved_mode == MODE_TRANSMITTER) ? "tx" : "rx");
    }
    
    ESP_LOGI(TAG, "도움말: help");
    ESP_LOGI(TAG, "");
    
    // 콘솔 루프
    char* line;
    while ((line = esp_console_linenoise("> ")) != NULL) {
        // 실시간 모니터링 중이면 개행
        if (real_time_monitoring) {
            printf("\n");
        }
        
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        
        if (err == ESP_ERR_NOT_FOUND) {
            printf("알 수 없는 명령어입니다. 'help' 명령어를 사용하세요.\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            printf("잘못된 인수입니다.\n");
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("명령어 실행 오류\n");
        }
        
        linenoise_history_add(line);
        free(line);
    }
}