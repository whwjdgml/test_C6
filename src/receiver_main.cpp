// ESP-NOW 배터리 히터 원격 모니터링 수신기
// 별도의 ESP32 디바이스에서 실행하여 히터 데이터 수신 및 분석

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"

#include "esp_now_manager.h"
#include "sensor_config.h"

static const char *TAG = "HEATER_RECEIVER";

// 수신된 데이터 저장소
static HeaterStatusPacket latest_heater_status = {};
static bool heater_data_received = false;
static uint32_t last_data_time_ms = 0;

// 통계
static uint32_t total_packets_received = 0;
static uint32_t heartbeat_count = 0;
static uint32_t status_packets = 0;
static uint32_t log_packets = 0;

// ESP-NOW 매니저
static ESPNowManager* espnow_manager = nullptr;

// 실시간 모니터링 플래그
static bool real_time_monitoring = false;

// 콜백 함수들
void onHeaterDataReceived(const uint8_t *mac, const uint8_t *data, int len);
void onDataSendComplete(const uint8_t *mac, esp_now_send_status_t status);

// 콘솔 명령어들
static int cmd_show_status(int argc, char **argv);
static int cmd_start_monitor(int argc, char **argv);
static int cmd_stop_monitor(int argc, char **argv);
static int cmd_send_command(int argc, char **argv);
static int cmd_show_stats(int argc, char **argv);
static int cmd_set_sender(int argc, char **argv);

// 콘솔 명령어 등록
void register_receiver_commands() {
    const esp_console_cmd_t status_cmd = {
        .command = "status",
        .help = "최신 히터 상태 출력",
        .hint = NULL,
        .func = &cmd_show_status,
    };
    esp_console_cmd_register(&status_cmd);
    
    const esp_console_cmd_t monitor_cmd = {
        .command = "monitor",
        .help = "실시간 모니터링 시작",
        .hint = NULL,
        .func = &cmd_start_monitor,
    };
    esp_console_cmd_register(&monitor_cmd);
    
    const esp_console_cmd_t stop_cmd = {
        .command = "stop",
        .help = "실시간 모니터링 중지",
        .hint = NULL,
        .func = &cmd_stop_monitor,
    };
    esp_console_cmd_register(&stop_cmd);
    
    const esp_console_cmd_t command_cmd = {
        .command = "send",
        .help = "원격 제어 명령 전송 (예: send target 6.0, send kp 20)",
        .hint = "<command> <value>",
        .func = &cmd_send_command,
    };
    esp_console_cmd_register(&command_cmd);
    
    const esp_console_cmd_t stats_cmd = {
        .command = "stats",
        .help = "수신 통계 출력",
        .hint = NULL,
        .func = &cmd_show_stats,
    };
    esp_console_cmd_register(&stats_cmd);
    
    const esp_console_cmd_t sender_cmd = {
        .command = "sender",
        .help = "송신기 MAC 주소 설정",
        .hint = "<AA:BB:CC:DD:EE:FF>",
        .func = &cmd_set_sender,
    };
    esp_console_cmd_register(&sender_cmd);
}

// 데이터 수신 콜백
void onHeaterDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < sizeof(ESPNowPacketHeader)) {
        ESP_LOGW(TAG, "패킷 크기 너무 작음: %d bytes", len);
        return;
    }
    
    ESPNowPacketHeader* header = (ESPNowPacketHeader*)data;
    total_packets_received++;
    last_data_time_ms = esp_timer_get_time() / 1000;
    
    switch (header->packet_type) {
        case PACKET_HEATER_STATUS: {
            if (len == sizeof(HeaterStatusPacket)) {
                memcpy(&latest_heater_status, data, sizeof(HeaterStatusPacket));
                heater_data_received = true;
                status_packets++;
                
                if (real_time_monitoring) {
                    printf("\r🌡️ %.2f°C → %d%% PWM (목표: %.1f°C, 오차: %+.2f°C) [%s]     ",
                           latest_heater_status.battery_temperature,
                           latest_heater_status.pwm_duty_percent,
                           latest_heater_status.target_temperature,
                           latest_heater_status.target_temperature - latest_heater_status.battery_temperature,
                           latest_heater_status.stepup_enabled ? "ON" : "OFF");
                    fflush(stdout);
                }
                
                ESP_LOGD(TAG, "히터 상태 수신: %.2f°C → %d%% PWM", 
                        latest_heater_status.battery_temperature,
                        latest_heater_status.pwm_duty_percent);
            }
            break;
        }
        
        case PACKET_TEMPERATURE_LOG: {
            if (len == sizeof(TemperatureLogPacket)) {
                TemperatureLogPacket* log_packet = (TemperatureLogPacket*)data;
                log_packets++;
                
                ESP_LOGI(TAG, "온도 로그 수신: %d개 데이터", log_packet->log_count);
                
                // CSV 형태로 출력
                if (!real_time_monitoring) {
                    printf("\n# 온도 로그 데이터 (시간오프셋, 온도, PWM, 오차)\n");
                    for (int i = 0; i < log_packet->log_count; i++) {
                        printf("%lu,%.2f,%d,%+.2f\n",
                               log_packet->logs[i].timestamp_offset_sec,
                               log_packet->logs[i].temperature,
                               log_packet->logs[i].pwm_duty,
                               log_packet->logs[i].error);
                    }
                }
            }
            break;
        }
        
        case PACKET_SYSTEM_INFO: {
            if (len == sizeof(SystemInfoPacket)) {
                SystemInfoPacket* sys_packet = (SystemInfoPacket*)data;
                ESP_LOGI(TAG, "시스템 정보: 가동시간 %lu초, 힙메모리 %lu bytes",
                        sys_packet->uptime_sec, sys_packet->free_heap_size);
            }
            break;
        }
        
        case PACKET_HEARTBEAT: {
            heartbeat_count++;
            ESP_LOGV(TAG, "하트비트 수신 (#%lu)", heartbeat_count);
            break;
        }
        
        default:
            ESP_LOGW(TAG, "알 수 없는 패킷 타입: 0x%02X", header->packet_type);
            break;
    }
}

void onDataSendComplete(const uint8_t *mac, esp_now_send_status_t status) {
    ESP_LOGD(TAG, "명령 전송 %s", (status == ESP_NOW_SEND_SUCCESS) ? "성공" : "실패");
}

// 콘솔 명령어 구현
static int cmd_show_status(int argc, char **argv) {
    if (!heater_data_received) {
        printf("수신된 히터 데이터가 없습니다.\n");
        return -1;
    }
    
    uint32_t age_sec = (esp_timer_get_time() / 1000 - last_data_time_ms) / 1000;
    
    printf("\n=== 최신 히터 상태 (%lu초 전) ===\n", age_sec);
    printf("배터리 온도: %.2f°C\n", latest_heater_status.battery_temperature);
    printf("목표 온도: %.1f°C\n", latest_heater_status.target_temperature);
    printf("온도 오차: %+.2f°C\n", latest_heater_status.target_temperature - latest_heater_status.battery_temperature);
    printf("PWM 듀티: %d%%\n", latest_heater_status.pwm_duty_percent);
    printf("전력 소모: %.3fW\n", latest_heater_status.power_consumption);
    printf("스텝업 컨버터: %s\n", latest_heater_status.stepup_enabled ? "활성화" : "비활성화");
    printf("비례 게인: %.1f\n", latest_heater_status.proportional_gain);
    printf("평균 오차: %.2f°C\n", latest_heater_status.average_error);
    printf("가열 사이클: %lu회\n", latest_heater_status.heating_cycles);
    printf("총 가열 시간: %lu초\n", latest_heater_status.total_heating_time_sec);
    
    const char* state_names[] = {"OFF", "LOW", "MED", "HIGH", "MAX", "UNKNOWN"};
    if (latest_heater_status.heater_state < 6) {
        printf("히터 상태: %s\n", state_names[latest_heater_status.heater_state]);
    }
    
    return 0;
}

static int cmd_start_monitor(int argc, char **argv) {
    real_time_monitoring = true;
    printf("실시간 모니터링 시작 (Ctrl+C로 중지)\n");
    printf("온도 | PWM | 목표 | 오차 | 상태\n");
    return 0;
}

static int cmd_stop_monitor(int argc, char **argv) {
    real_time_monitoring = false;
    printf("\n실시간 모니터링 중지\n");
    return 0;
}

static int cmd_send_command(int argc, char **argv) {
    if (argc < 3) {
        printf("사용법: send <command> <value>\n");
        printf("예시:\n");
        printf("  send target 6.0    - 목표온도 6.0°C로 설정\n");
        printf("  send kp 20          - 비례게인 20으로 설정\n");
        printf("  send enable 1       - 히터 활성화\n");
        printf("  send enable 0       - 히터 비활성화\n");
        return -1;
    }
    
    ControlCommandPacket cmd_packet = {};
    espnow_manager->fillPacketHeader(&cmd_packet.header, PACKET_CONTROL_COMMAND, 
                                    sizeof(cmd_packet) - sizeof(ESPNowPacketHeader));
    
    if (strcmp(argv[1], "target") == 0) {
        cmd_packet.command_type = 1; // SET_TARGET_TEMP
        cmd_packet.data.set_target.new_target_temp = atof(argv[2]);
        printf("목표온도 변경 명령 전송: %.1f°C\n", cmd_packet.data.set_target.new_target_temp);
    }
    else if (strcmp(argv[1], "kp") == 0) {
        cmd_packet.command_type = 2; // SET_GAIN
        cmd_packet.data.set_gain.new_kp = atof(argv[2]);
        printf("비례게인 변경 명령 전송: %.1f\n", cmd_packet.data.set_gain.new_kp);
    }
    else if (strcmp(argv[1], "enable") == 0) {
        cmd_packet.command_type = 3; // HEATER_ENABLE
        cmd_packet.data.heater_enable.enable = (atoi(argv[2]) != 0);
        printf("히터 %s 명령 전송\n", cmd_packet.data.heater_enable.enable ? "활성화" : "비활성화");
    }
    else {
        printf("알 수 없는 명령: %s\n", argv[1]);
        return -1;
    }
    
    // TODO: 실제 전송 구현
    // espnow_manager->sendControlCommand(cmd_packet);
    
    return 0;
}

static int cmd_show_stats(int argc, char **argv) {
    printf("\n=== 수신 통계 ===\n");
    printf("총 수신 패킷: %lu개\n", total_packets_received);
    printf("히터 상태 패킷: %lu개\n", status_packets);
    printf("온도 로그 패킷: %lu개\n", log_packets);
    printf("하트비트: %lu개\n", heartbeat_count);
    
    if (heater_data_received) {
        uint32_t age_sec = (esp_timer_get_time() / 1000 - last_data_time_ms) / 1000;
        printf("마지막 데이터: %lu초 전\n", age_sec);
        
        if (age_sec > 60) {
            printf("⚠️ 송신기와 연결이 끊어진 것 같습니다\n");
        } else {
            printf("✅ 송신기와 정상 연결됨\n");
        }
    } else {
        printf("❌ 수신된 히터 데이터가 없습니다\n");
    }
    
    if (espnow_manager) {
        espnow_manager->printStatistics();
    }
    
    return 0;
}

static int cmd_set_sender(int argc, char **argv) {
    if (argc < 2) {
        printf("사용법: sender <MAC주소>\n");
        printf("예시: sender 24:6F:28:AB:CD:EF\n");
        return -1;
    }
    
    uint8_t mac[6];
    if (sscanf(argv[1], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6) {
        if (espnow_manager->setReceiverMAC(mac)) {
            printf("송신기 MAC 설정 완료: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        } else {
            printf("송신기 MAC 설정 실패\n");
        }
    } else {
        printf("잘못된 MAC 주소 형식입니다\n");
    }
    
    return 0;
}

// 수신기 메인 함수
extern "C" void app_main() {
    ESP_LOGI(TAG, "🔗 ESP-NOW 배터리 히터 모니터링 수신기 시작");
    
    // UART 콘솔 초기화
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    
    esp_console_config_t console_config = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 8,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));
    
    // ESP-NOW 매니저 초기화
    espnow_manager = new ESPNowManager();
    if (!espnow_manager->init()) {
        ESP_LOGE(TAG, "❌ ESP-NOW 초기화 실패");
        return;
    }
    
    // 콜백 함수 설정
    espnow_manager->setDataReceivedCallback(onHeaterDataReceived);
    espnow_manager->setSendCompleteCallback(onDataSendComplete);
    
    // 콘솔 명령어 등록
    register_receiver_commands();
    
    ESP_LOGI(TAG, "✅ 수신기 초기화 완료!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📋 사용 가능한 명령어:");
    ESP_LOGI(TAG, "  status        - 최신 히터 상태 확인");
    ESP_LOGI(TAG, "  monitor       - 실시간 모니터링 시작");
    ESP_LOGI(TAG, "  stop          - 모니터링 중지");
    ESP_LOGI(TAG, "  send <cmd>    - 원격 제어 명령 전송");
    ESP_LOGI(TAG, "  stats         - 수신 통계 확인");
    ESP_LOGI(TAG, "  sender <MAC>  - 송신기 MAC 설정");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "데이터 수신 대기 중...");
    
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