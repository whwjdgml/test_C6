// 배터리 히터 + ESP-NOW 통합 프로토타입
// 비례제어 히터 + 원격 모니터링 기능

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_console.h"

#include "sensor_config.h"
#include "ntc_sensor.h"
#include "proportional_battery_heater.h"
#include "esp_now_manager.h"

static const char *TAG = "HEATER_ESPNOW";

// 전역 객체들
static NTCSensor* battery_ntc = nullptr;
static ProportionalBatteryHeater* prop_heater = nullptr;
static ESPNowManager* espnow_mgr = nullptr;

// 통신 설정
static bool espnow_enabled = true;
static uint32_t data_send_interval_ms = 10000;  // 10초마다 데이터 전송
static uint32_t last_data_send_ms = 0;
static uint32_t heartbeat_interval_ms = 30000;  // 30초마다 하트비트
static uint32_t last_heartbeat_ms = 0;

// 배치 로그 전송용
struct LogEntry {
    uint32_t timestamp_offset_sec;
    float temperature;
    uint8_t pwm_duty;
    float error;
};

static LogEntry log_batch[10];
static int log_batch_count = 0;
static uint32_t batch_start_time_ms = 0;

// ESP-NOW 콜백 함수들
void onRemoteCommandReceived(const uint8_t *mac, const uint8_t *data, int len);
void onDataTransmissionComplete(const uint8_t *mac, esp_now_send_status_t status);

// 히터 데이터 전송 함수들
void sendHeaterStatusPacket();
void sendTemperatureLogBatch();
void sendSystemInfoPacket();
void addToLogBatch(float temperature, uint8_t pwm_duty, float error);

// 콘솔 명령어들
static int cmd_enable_espnow(int argc, char **argv);
static int cmd_disable_espnow(int argc, char **argv);
static int cmd_set_interval(int argc, char **argv);
static int cmd_send_now(int argc, char **argv);
static int cmd_espnow_stats(int argc, char **argv);

// 콘솔 명령어 등록 (기존 히터 명령어 + ESP-NOW 명령어)
void register_espnow_commands() {
    // ESP-NOW 관련 명령어들
    const esp_console_cmd_t enable_cmd = {
        .command = "espnow_on",
        .help = "ESP-NOW 데이터 전송 활성화",
        .hint = NULL,
        .func = &cmd_enable_espnow,
    };
    esp_console_cmd_register(&enable_cmd);
    
    const esp_console_cmd_t disable_cmd = {
        .command = "espnow_off", 
        .help = "ESP-NOW 데이터 전송 비활성화",
        .hint = NULL,
        .func = &cmd_disable_espnow,
    };
    esp_console_cmd_register(&disable_cmd);
    
    const esp_console_cmd_t interval_cmd = {
        .command = "interval",
        .help = "데이터 전송 간격 설정 (초)",
        .hint = "<seconds>",
        .func = &cmd_set_interval,
    };
    esp_console_cmd_register(&interval_cmd);
    
    const esp_console_cmd_t send_cmd = {
        .command = "send_now",
        .help = "즉시 데이터 전송",
        .hint = NULL,
        .func = &cmd_send_now,
    };
    esp_console_cmd_register(&send_cmd);
    
    const esp_console_cmd_t stats_cmd = {
        .command = "comm_stats",
        .help = "ESP-NOW 통신 통계 출력",
        .hint = NULL,
        .func = &cmd_espnow_stats,
    };
    esp_console_cmd_register(&stats_cmd);
}

// ESP-NOW 콜백 함수 구현
void onRemoteCommandReceived(const uint8_t *mac, const uint8_t *data, int len) {
    ESP_LOGI(TAG, "원격 명령 수신: %d bytes", len);
    
    if (len >= sizeof(ESPNowPacketHeader)) {
        ESPNowPacketHeader* header = (ESPNowPacketHeader*)data;
        
        if (header->packet_type == PACKET_CONTROL_COMMAND && 
            len == sizeof(ControlCommandPacket)) {
            
            ControlCommandPacket* cmd = (ControlCommandPacket*)data;
            
            switch (cmd->command_type) {
                case 1: // SET_TARGET_TEMP
                    prop_heater->setTargetTemperature(cmd->data.set_target.new_target_temp);
                    ESP_LOGI(TAG, "🎯 원격 목표온도 변경: %.1f°C", 
                            cmd->data.set_target.new_target_temp);
                    break;
                    
                case 2: // SET_GAIN
                    prop_heater->setProportionalGain(cmd->data.set_gain.new_kp);
                    ESP_LOGI(TAG, "⚙️ 원격 게인 변경: Kp=%.1f", 
                            cmd->data.set_gain.new_kp);
                    break;
                    
                case 3: // HEATER_ENABLE
                    prop_heater->enableProportionalMode(cmd->data.heater_enable.enable);
                    ESP_LOGI(TAG, "🔥 원격 히터 제어: %s", 
                            cmd->data.heater_enable.enable ? "활성화" : "비활성화");
                    break;
                    
                default:
                    ESP_LOGW(TAG, "알 수 없는 원격 명령: %d", cmd->command_type);
                    break;
            }
            
            // 명령 처리 후 즉시 상태 전송
            sendHeaterStatusPacket();
        }
    }
}

void onDataTransmissionComplete(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "데이터 전송 실패 to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

// 히터 상태 패킷 전송
void sendHeaterStatusPacket() {
    if (!espnow_enabled || !espnow_mgr) return;
    
    HeaterStatusPacket packet = {};
    espnow_mgr->fillPacketHeader(&packet.header, PACKET_HEATER_STATUS, 
                                sizeof(packet) - sizeof(ESPNowPacketHeader));
    
    // 현재 히터 상태 데이터 채우기
    packet.battery_temperature = prop_heater->getLastTemperature();
    packet.target_temperature = prop_heater->getTargetTemperature();
    packet.pwm_duty_percent = 0; // 상태 기반으로 계산
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
    
    if (espnow_mgr->sendHeaterStatus(packet)) {
        ESP_LOGD(TAG, "📡 히터 상태 전송: %.2f°C → %d%% PWM", 
                packet.battery_temperature, packet.pwm_duty_percent);
    }
}

// 온도 로그 배치 전송
void sendTemperatureLogBatch() {
    if (!espnow_enabled || !espnow_mgr || log_batch_count == 0) return;
    
    TemperatureLogPacket packet = {};
    espnow_mgr->fillPacketHeader(&packet.header, PACKET_TEMPERATURE_LOG,
                                sizeof(packet) - sizeof(ESPNowPacketHeader));
    
    packet.log_count = log_batch_count;
    for (int i = 0; i < log_batch_count; i++) {
        packet.logs[i].timestamp_offset_sec = log_batch[i].timestamp_offset_sec;
        packet.logs[i].temperature = log_batch[i].temperature;
        packet.logs[i].pwm_duty = log_batch[i].pwm_duty;
        packet.logs[i].error = log_batch[i].error;
    }
    
    if (espnow_mgr->sendTemperatureLog(packet)) {
        ESP_LOGI(TAG, "📊 온도 로그 배치 전송: %d개 항목", log_batch_count);
        
        // 배치 초기화
        log_batch_count = 0;
        batch_start_time_ms = esp_timer_get_time() / 1000;
    }
}

// 시스템 정보 패킷 전송
void sendSystemInfoPacket() {
    if (!espnow_enabled || !espnow_mgr) return;
    
    SystemInfoPacket packet = {};
    espnow_mgr->fillPacketHeader(&packet.header, PACKET_SYSTEM_INFO,
                                sizeof(packet) - sizeof(ESPNowPacketHeader));
    
    packet.uptime_sec = esp_timer_get_time() / 1000000;
    packet.free_heap_size = esp_get_free_heap_size();
    packet.battery_voltage = 3.7f; // TODO: 실제 배터리 전압 읽기
    packet.battery_current = 0.0f; // TODO: 실제 전류 읽기
    packet.wifi_rssi = -50; // TODO: 실제 RSSI 읽기
    packet.error_count = 0; // TODO: 오류 카운터 구현
    strcpy(packet.firmware_version, "v1.0.0");
    
    if (espnow_mgr->sendSystemInfo(packet)) {
        ESP_LOGD(TAG, "📋 시스템 정보 전송");
    }
}

// 로그 배치에 추가
void addToLogBatch(float temperature, uint8_t pwm_duty, float error) {
    if (log_batch_count >= 10) {
        // 배치가 가득참 - 전송 후 초기화
        sendTemperatureLogBatch();
    }
    
    if (log_batch_count == 0) {
        batch_start_time_ms = esp_timer_get_time() / 1000;
    }
    
    log_batch[log_batch_count].timestamp_offset_sec = 
        (esp_timer_get_time() / 1000 - batch_start_time_ms) / 1000;
    log_batch[log_batch_count].temperature = temperature;
    log_batch[log_batch_count].pwm_duty = pwm_duty;
    log_batch[log_batch_count].error = error;
    log_batch_count++;
}

// 콘솔 명령어 구현
static int cmd_enable_espnow(int argc, char **argv) {
    espnow_enabled = true;
    printf("ESP-NOW 데이터 전송 활성화\n");
    return 0;
}

static int cmd_disable_espnow(int argc, char **argv) {
    espnow_enabled = false;
    printf("ESP-NOW 데이터 전송 비활성화\n");
    return 0;
}

static int cmd_set_interval(int argc, char **argv) {
    if (argc < 2) {
        printf("현재 전송 간격: %lu초\n", data_send_interval_ms / 1000);
        return 0;
    }
    
    uint32_t interval_sec = atoi(argv[1]);
    if (interval_sec >= 1 && interval_sec <= 300) {
        data_send_interval_ms = interval_sec * 1000;
        printf("전송 간격 변경: %lu초\n", interval_sec);
    } else {
        printf("잘못된 간격 (1~300초)\n");
    }
    
    return 0;
}

static int cmd_send_now(int argc, char **argv) {
    if (!espnow_enabled) {
        printf("ESP-NOW가 비활성화됨\n");
        return -1;
    }
    
    printf("즉시 데이터 전송 중...\n");
    sendHeaterStatusPacket();
    sendTemperatureLogBatch();
    sendSystemInfoPacket();
    espnow_mgr->sendHeartbeat();
    
    return 0;
}

static int cmd_espnow_stats(int argc, char **argv) {
    if (espnow_mgr) {
        espnow_mgr->printStatistics();
    }
    
    printf("\n=== 데이터 전송 설정 ===\n");
    printf("ESP-NOW: %s\n", espnow_enabled ? "활성화" : "비활성화");
    printf("전송 간격: %lu초\n", data_send_interval_ms / 1000);
    printf("하트비트 간격: %lu초\n", heartbeat_interval_ms / 1000);
    printf("현재 로그 배치: %d/10개\n", log_batch_count);
    
    return 0;
}

// 메인 태스크
void heater_espnow_task(void *pvParameters) {
    uint32_t measurement_count = 0;
    
    ESP_LOGI(TAG, "🔄 히터+ESP-NOW 통합 태스크 시작");
    
    while (true) {
        measurement_count++;
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // 히터 업데이트
        if (prop_heater->updateProportionalHeater()) {
            float temp = prop_heater->getLastTemperature();
            float target = prop_heater->getTargetTemperature();
            float error = target - temp;
            
            // 현재 PWM 듀티 계산
            uint8_t pwm_duty = 0;
            switch (prop_heater->getCurrentState()) {
                case HEATER_LOW:  pwm_duty = 25; break;
                case HEATER_MED:  pwm_duty = 50; break;
                case HEATER_HIGH: pwm_duty = 75; break;
                case HEATER_MAX:  pwm_duty = 100; break;
                default: pwm_duty = 0; break;
            }
            
            // 로그 배치에 추가
            addToLogBatch(temp, pwm_duty, error);
            
            // 주기적 상태 출력 (30초마다)
            if (measurement_count % 3 == 1) {
                ESP_LOGI(TAG, "🌡️ %.2f°C → %d%% PWM (목표: %.1f°C, 오차: %+.2f°C)", 
                        temp, pwm_duty, target, error);
            }
        }
        
        if (espnow_enabled && espnow_mgr) {
            // 주기적 데이터 전송
            if (current_time - last_data_send_ms >= data_send_interval_ms) {
                sendHeaterStatusPacket();
                last_data_send_ms = current_time;
            }
            
            // 하트비트 전송
            if (current_time - last_heartbeat_ms >= heartbeat_interval_ms) {
                espnow_mgr->sendHeartbeat();
                last_heartbeat_ms = current_time;
            }
            
            // 로그 배치가 가득 찼거나 5분 지났으면 전송
            if (log_batch_count >= 10 || 
                (log_batch_count > 0 && (current_time - batch_start_time_ms) >= 300000)) {
                sendTemperatureLogBatch();
            }
        }
        
        // 10초마다 측정
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// 통합 메인 함수
extern "C" void app_main() {
    ESP_LOGI(TAG, "🔥📡 배터리 히터 + ESP-NOW 통합 시스템 시작");
    
    // 콘솔 초기화 (기존 코드와 동일)
    // ... UART 콘솔 초기화 코드 ...
    
    // NTC 센서 초기화
    ESP_LOGI(TAG, "NTC 온도센서 초기화...");
    battery_ntc = new NTCSensor(BATTERY_NTC_ADC_CHANNEL, BATTERY_NTC_POWER_PIN);
    if (!battery_ntc->init()) {
        ESP_LOGE(TAG, "❌ NTC 센서 초기화 실패");
        return;
    }
    
    // 비례 제어 히터 초기화
    ESP_LOGI(TAG, "비례 제어 히터 초기화...");
    prop_heater = new ProportionalBatteryHeater(battery_ntc,
                                               STEPUP_CONVERTER_EN_PIN,
                                               BATTERY_HEATER_PWM_PIN);
    if (!prop_heater->init()) {
        ESP_LOGE(TAG, "❌ 히터 초기화 실패");
        return;
    }
    
    // ESP-NOW 초기화
    ESP_LOGI(TAG, "ESP-NOW 통신 초기화...");
    espnow_mgr = new ESPNowManager();
    if (!espnow_mgr->init()) {
        ESP_LOGE(TAG, "❌ ESP-NOW 초기화 실패");
        return;
    }
    
    // 콜백 설정
    espnow_mgr->setDataReceivedCallback(onRemoteCommandReceived);
    espnow_mgr->setSendCompleteCallback(onDataTransmissionComplete);
    
    // 기본 설정
    prop_heater->setTargetTemperature(5.0f);
    prop_heater->setProportionalGain(15.0f);
    
    // 명령어 등록
    register_espnow_commands();
    
    // 백그라운드 태스크 시작
    xTaskCreate(heater_espnow_task, "heater_espnow", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "✅ 통합 시스템 초기화 완료!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📋 ESP-NOW 추가 명령어:");
    ESP_LOGI(TAG, "  espnow_on/off  - 무선 전송 제어");
    ESP_LOGI(TAG, "  interval <초>  - 전송 간격 설정");
    ESP_LOGI(TAG, "  send_now      - 즉시 데이터 전송");
    ESP_LOGI(TAG, "  comm_stats    - 통신 통계 확인");
    ESP_LOGI(TAG, "");
    
    // 콘솔 루프 (기존과 동일)
    // ... 콘솔 처리 코드 ...
}