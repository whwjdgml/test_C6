#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "nvs_flash.h"
#include "esp_now_manager.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "sensor_config.h"
#include "ntc_sensor.h"
#include "esp_system.h"

static const char *TAG = "UNIFIED_HEATER";

// 시스템 모드
typedef enum {
    MODE_HEATER_ONLY,     // 히터만 (ESP-NOW 없음)
    MODE_TRANSMITTER,     // 송신기 (히터 + 데이터 전송)
    MODE_RECEIVER,        // 수신기 (데이터 수신 + 모니터링)
    MODE_UNDEFINED        // 미정의 (최초 실행)
} system_mode_t;

// 전역 변수
static system_mode_t current_mode = MODE_UNDEFINED;
static ESPNowManager* espnow_mgr = nullptr;

// 히터 제어 변수
static bool heater_initialized = false;
static uint8_t current_pwm_duty = 0;  // 0-100%
static bool stepup_enabled = false;

// NTC 센서 변수
static NTCSensor* ntc_sensor = nullptr;

// ESP-NOW 통신 변수
static bool espnow_initialized = false;
static uint32_t heartbeat_interval_ms = 10000;  // 10초
static uint32_t last_heartbeat_ms = 0;

// 히터 제어 함수들
bool initHeater() {
    // PWM 설정
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = HEATER_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);
    
    // PWM 채널 설정
    ledc_channel_config_t channel_config = {
        .gpio_num = BATTERY_HEATER_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    esp_err_t ret = ledc_channel_config(&channel_config);
    
    if (ret == ESP_OK) {
        heater_initialized = true;
        ESP_LOGI(TAG, "✅ 히터 PWM 초기화 완료 (GPIO%d)", BATTERY_HEATER_PWM_PIN);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ 히터 PWM 초기화 실패");
        return false;
    }
}

void setHeaterDuty(uint8_t duty_percent) {
    if (!heater_initialized || duty_percent > 100) return;
    
    uint32_t duty_value = (duty_percent * 255) / 100;  // 8-bit PWM
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    
    current_pwm_duty = duty_percent;
    stepup_enabled = (duty_percent > 0);
    
    ESP_LOGI(TAG, "히터 PWM 설정: %d%% (듀티값: %lu)", duty_percent, duty_value);
}

// NTC 센서 함수들
bool initNTC() {
    if (ntc_sensor) {
        delete ntc_sensor;
    }
    
    ntc_sensor = new NTCSensor(BATTERY_NTC_ADC_CHANNEL, BATTERY_NTC_POWER_PIN);
    
    if (ntc_sensor->init()) {
        ESP_LOGI(TAG, "✅ NTC 온도센서 초기화 완료");
        return true;
    } else {
        ESP_LOGE(TAG, "❌ NTC 온도센서 초기화 실패");
        delete ntc_sensor;
        ntc_sensor = nullptr;
        return false;
    }
}

float readBatteryTemperature() {
    if (!ntc_sensor) {
        ESP_LOGW(TAG, "NTC 센서가 초기화되지 않음");
        return -999.0f;
    }
    
    float temperature;
    if (ntc_sensor->readTemperature(&temperature)) {
        return temperature;
    } else {
        ESP_LOGW(TAG, "NTC 온도 읽기 실패");
        return -999.0f;
    }
}

// ESP-NOW 콜백 함수들
void onESPNowDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    // 인터럽트 컨텍스트에서 안전하게 실행
    printf("\n📡 수신 (길이:%d)\n> ", len);
    fflush(stdout);
}

void onESPNowDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    // 인터럽트 컨텍스트에서 안전하게 실행
    const char* status_str = (status == ESP_NOW_SEND_SUCCESS) ? "성공" : "실패";
    printf("\n📤 전송 %s\n> ", status_str);
    fflush(stdout);
}

// ESP-NOW 초기화
bool initESPNow() {
    if (espnow_mgr) {
        delete espnow_mgr;
    }
    
    espnow_mgr = new ESPNowManager();
    
    if (espnow_mgr->init()) {
        espnow_mgr->setDataReceivedCallback(onESPNowDataReceived);
        espnow_mgr->setSendCompleteCallback(onESPNowDataSent);
        espnow_initialized = true;
        ESP_LOGI(TAG, "✅ ESP-NOW 통신 초기화 완료");
        return true;
    } else {
        ESP_LOGE(TAG, "❌ ESP-NOW 통신 초기화 실패");
        delete espnow_mgr;
        espnow_mgr = nullptr;
        espnow_initialized = false;
        return false;
    }
}

// 히터 상태 전송
bool sendHeaterStatus() {
    if (!espnow_mgr || !espnow_initialized) {
        printf("❌ ESP-NOW가 초기화되지 않음\n");
        return false;
    }
    
    HeaterStatusPacket packet;
    espnow_mgr->fillPacketHeader(&packet.header, PACKET_HEATER_STATUS, sizeof(packet) - sizeof(ESPNowPacketHeader));
    
    // 현재 시스템 상태로 패킷 데이터 채우기
    packet.battery_temperature = readBatteryTemperature();
    packet.target_temperature = 5.0f;  // 기본 목표 온도
    packet.pwm_duty_percent = current_pwm_duty;
    packet.heater_state = stepup_enabled ? 1 : 0;
    packet.power_consumption = (current_pwm_duty * 2.5f) / 100.0f;  // 추정 전력
    packet.stepup_enabled = stepup_enabled;
    packet.proportional_gain = 15.0f;  // 기본 게인
    packet.average_error = 0.0f;
    packet.heating_cycles = 0;
    packet.total_heating_time_sec = 0;
    
    return espnow_mgr->sendHeaterStatus(packet);
}

// 하트비트 전송
bool sendHeartbeat() {
    if (!espnow_mgr || !espnow_initialized) {
        return false;
    }
    
    return espnow_mgr->sendHeartbeat();
}

// 간단한 명령어 처리
void process_command(char* cmd) {
    if (strncmp(cmd, "help", 4) == 0) {
        printf("=== 사용 가능한 명령어 ===\n");
        printf("help       - 이 도움말\n");
        printf("info       - 시스템 정보\n");
        printf("test       - 테스트 메시지\n");
        printf("mem        - 메모리 정보\n");
        printf("cls        - 화면 지우기\n");
        printf("mode       - 모드 설정/확인\n");
        printf("  mode heater  - 히터 전용 모드\n");
        printf("  mode tx      - 송신기 모드\n");
        printf("  mode rx      - 수신기 모드\n");
        printf("espnow     - ESP-NOW 통신 상태/제어\n");
        printf("  espnow init    - ESP-NOW 초기화\n");
        printf("  espnow send    - 히터 상태 전송\n");
        printf("  espnow beat    - 하트비트 전송\n");
        printf("heater     - 히터 상태 확인\n");
        printf("  heater init    - 히터 초기화\n");
        printf("  heater on      - 히터 켜기 (25%%)\n");
        printf("  heater off     - 히터 끄기\n");
        printf("  heater <0-100> - PWM 듀티 설정\n");
        printf("ntc        - NTC 온도센서 상태\n");
        printf("  ntc init       - NTC 센서 초기화\n");
        printf("  ntc read       - 온도 읽기\n");
    }
    else if (strncmp(cmd, "info", 4) == 0) {
        printf("=== 시스템 정보 ===\n");
        printf("상태: 정상 동작\n");
        printf("칩: ESP32-C6\n");
        printf("IDF Version: %s\n", esp_get_idf_version());
    }
    else if (strncmp(cmd, "mem", 3) == 0) {
        printf("=== 메모리 정보 ===\n");
        printf("Free heap: %lu bytes\n", (unsigned long)esp_get_free_heap_size());
        printf("Min free heap: %lu bytes\n", (unsigned long)esp_get_minimum_free_heap_size());
    }
    else if (strncmp(cmd, "test", 4) == 0) {
        printf("✅ 테스트 명령어 실행됨!\n");
        ESP_LOGI(TAG, "테스트 로그 메시지");
        for (int i = 0; i < 3; i++) {
            printf("카운트: %d\n", i);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
    else if (strncmp(cmd, "cls", 3) == 0) {
        printf("\033[2J\033[H");  // ANSI 클리어 스크린
        printf("화면이 지워졌습니다.\n");
    }
    else if (strncmp(cmd, "mode", 4) == 0) {
        // mode 뒤의 인수 확인
        char* arg = strchr(cmd, ' ');
        if (arg) {
            arg++; // 공백 다음 문자로 이동
            if (strncmp(arg, "heater", 6) == 0) {
                current_mode = MODE_HEATER_ONLY;
                printf("✅ 히터 전용 모드로 설정되었습니다.\n");
                printf("ESP-NOW 통신 없이 히터만 동작합니다.\n");
            }
            else if (strncmp(arg, "tx", 2) == 0) {
                current_mode = MODE_TRANSMITTER;
                printf("📡 송신기 모드로 설정되었습니다.\n");
                printf("히터 동작 + 데이터 전송을 수행합니다.\n");
                if (initESPNow()) {
                    printf("✅ ESP-NOW 자동 초기화 완료\n");
                } else {
                    printf("❌ ESP-NOW 초기화 실패\n");
                }
            }
            else if (strncmp(arg, "rx", 2) == 0) {
                current_mode = MODE_RECEIVER;
                printf("📺 수신기 모드로 설정되었습니다.\n");
                printf("데이터 수신 + 모니터링을 수행합니다.\n");
                if (initESPNow()) {
                    printf("✅ ESP-NOW 자동 초기화 완료\n");
                } else {
                    printf("❌ ESP-NOW 초기화 실패\n");
                }
            }
            else {
                printf("❌ 잘못된 모드입니다. 사용법: mode <heater|tx|rx>\n");
            }
        } else {
            const char* mode_str = (current_mode == MODE_HEATER_ONLY) ? "heater" :
                                  (current_mode == MODE_TRANSMITTER) ? "tx" :
                                  (current_mode == MODE_RECEIVER) ? "rx" : "undefined";
            printf("현재 모드: %s\n", mode_str);
            printf("사용법: mode <heater|tx|rx>\n");
        }
    }
    else if (strncmp(cmd, "espnow", 6) == 0) {
        char* arg = strchr(cmd, ' ');
        if (arg) {
            arg++;
            if (strncmp(arg, "init", 4) == 0) {
                if (initESPNow()) {
                    printf("✅ ESP-NOW 통신 초기화 완료!\n");
                } else {
                    printf("❌ ESP-NOW 통신 초기화 실패!\n");
                }
            }
            else if (strncmp(arg, "send", 4) == 0) {
                if (!espnow_initialized || !espnow_mgr) {
                    printf("❌ ESP-NOW가 초기화되지 않았습니다. 'espnow init' 먼저 실행하세요.\n");
                } else if (sendHeaterStatus()) {
                    printf("📡 히터 상태 전송 완료\n");
                } else {
                    printf("❌ 히터 상태 전송 실패\n");
                }
            }
            else if (strncmp(arg, "beat", 4) == 0) {
                if (!espnow_initialized || !espnow_mgr) {
                    printf("❌ ESP-NOW가 초기화되지 않았습니다. 'espnow init' 먼저 실행하세요.\n");
                } else if (sendHeartbeat()) {
                    printf("💓 하트비트 전송 완료\n");
                } else {
                    printf("❌ 하트비트 전송 실패\n");
                }
            }
            else {
                printf("❌ 잘못된 ESP-NOW 명령어: %s\n", arg);
                printf("사용법: espnow init | espnow send | espnow beat\n");
            }
        } else {
            printf("=== ESP-NOW 통신 상태 ===\n");
            printf("초기화: %s\n", espnow_initialized ? "완료" : "미완료");
            printf("현재 모드: %s\n", (current_mode == MODE_TRANSMITTER) ? "송신기" :
                                   (current_mode == MODE_RECEIVER) ? "수신기" : "비활성");
            if (espnow_mgr && espnow_initialized) {
                espnow_mgr->printStatistics();
            } else {
                printf("먼저 'espnow init' 또는 tx/rx 모드로 설정하세요.\n");
            }
        }
    }
    else if (strncmp(cmd, "heater", 6) == 0) {
        char* arg = strchr(cmd, ' ');
        if (arg) {
            arg++;
            if (strncmp(arg, "init", 4) == 0) {
                if (initHeater()) {
                    printf("✅ 히터 시스템 초기화 완료!\n");
                } else {
                    printf("❌ 히터 시스템 초기화 실패!\n");
                }
            }
            else if (strncmp(arg, "on", 2) == 0) {
                setHeaterDuty(25);  // 25% 듀티
                printf("🔥 히터 켜기 (25%% PWM)\n");
            }
            else if (strncmp(arg, "off", 3) == 0) {
                setHeaterDuty(0);   // 0% 듀티
                printf("❄️ 히터 끄기 (0%% PWM)\n");
            }
            else {
                // 숫자인지 확인 (PWM 듀티 설정)
                int duty = atoi(arg);
                if (duty >= 0 && duty <= 100) {
                    setHeaterDuty(duty);
                    printf("🔧 히터 PWM 설정: %d%%\n", duty);
                } else {
                    printf("❌ 잘못된 듀티값: %s (0-100 허용)\n", arg);
                }
            }
        } else {
            printf("=== 히터 시스템 상태 ===\n");
            printf("초기화: %s\n", heater_initialized ? "완료" : "미완료");
            printf("현재 PWM: %d%%\n", current_pwm_duty);
            printf("스텝업 컨버터: %s\n", stepup_enabled ? "ON" : "OFF");
            if (!heater_initialized) {
                printf("먼저 'heater init' 명령어로 초기화하세요.\n");
            }
        }
    }
    else if (strncmp(cmd, "ntc", 3) == 0) {
        char* arg = strchr(cmd, ' ');
        if (arg) {
            arg++;
            if (strncmp(arg, "init", 4) == 0) {
                if (initNTC()) {
                    printf("✅ NTC 온도센서 초기화 완료!\n");
                    if (ntc_sensor) {
                        ntc_sensor->printCalibrationInfo();
                    }
                } else {
                    printf("❌ NTC 온도센서 초기화 실패!\n");
                }
            }
            else if (strncmp(arg, "read", 4) == 0) {
                float temp = readBatteryTemperature();
                if (temp != -999.0f) {
                    printf("🌡️ 배터리 온도: %.2f°C\n", temp);
                    
                    // 온도에 따른 상태 표시
                    if (temp < 0) {
                        printf("❄️ 매우 추움 - 히터 필요\n");
                    } else if (temp < 5) {
                        printf("🧊 추움 - 히터 권장\n");
                    } else if (temp < 10) {
                        printf("😐 적정 온도\n");
                    } else {
                        printf("🔥 따뜻함\n");
                    }
                } else {
                    printf("❌ 온도 읽기 실패!\n");
                    printf("먼저 'ntc init' 명령어로 초기화하세요.\n");
                }
            }
            else {
                printf("❌ 잘못된 NTC 명령어: %s\n", arg);
                printf("사용법: ntc init | ntc read\n");
            }
        } else {
            printf("=== NTC 온도센서 상태 ===\n");
            printf("초기화: %s\n", ntc_sensor ? "완료" : "미완료");
            if (ntc_sensor) {
                float temp = readBatteryTemperature();
                if (temp != -999.0f) {
                    printf("현재 온도: %.2f°C\n", temp);
                } else {
                    printf("온도 읽기: 실패\n");
                }
            } else {
                printf("먼저 'ntc init' 명령어로 초기화하세요.\n");
            }
        }
    }
    else if (strlen(cmd) > 0) {
        printf("❌ 알 수 없는 명령어: '%s'\n", cmd);
        printf("도움말: help\n");
    }
}

extern "C" void app_main() {
    printf("\n=== ESP32-C6 Battery Heater System ===\n");
    
    // NVS 초기화 (ESP-NOW에 필요)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 로깅 설정
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "🔄 통합 배터리 히터 시스템 시작");
    
    // USB Serial JTAG 드라이버 초기화
    usb_serial_jtag_driver_config_t config = {
        .tx_buffer_size = 512,
        .rx_buffer_size = 512,
    };
    usb_serial_jtag_driver_install(&config);
    
    ESP_LOGI(TAG, "✅ USB Serial JTAG 드라이버 초기화 완료!");
    ESP_LOGI(TAG, "✅ 시스템 초기화 완료!");
    
    // USB Serial JTAG 안정화 대기 및 연결 확인
    vTaskDelay(pdMS_TO_TICKS(1000));  // 더 긴 대기시간
    
    // 부팅 완료 배너를 첫 번째 입력 대기 시에 표시하도록 변경
    bool banner_shown = false;
    
    char input_buffer[100];
    int pos = 0;
    
    while (1) {
        uint8_t data[64];
        
        // USB Serial JTAG에서 직접 읽기 (100ms 타임아웃)
        int len = usb_serial_jtag_read_bytes(data, sizeof(data), 100 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            // 첫 번째 입력 시 배너 표시
            if (!banner_shown) {
                const char* banner = 
                    "\n"
                    "╔════════════════════════════════════════════════╗\n"
                    "║          ESP32-C6 Battery Heater System       ║\n"
                    "║                부팅 완료                      ║\n"
                    "╚════════════════════════════════════════════════╝\n"
                    "\n"
                    "🎯 시스템 상태:\n"
                    "  - CPU: ESP32-C6 @ 160MHz\n"
                    "  - RAM: 사용 가능\n"
                    "  - Flash: 83.4% 사용 중\n"
                    "  - USB Serial JTAG: ✅ 준비됨\n"
                    "\n"
                    "📋 주요 명령어:\n"
                    "  help       - 전체 명령어 목록\n"
                    "  espnow     - ESP-NOW 무선통신\n"
                    "  heater     - 배터리 히터 제어\n"
                    "  ntc        - 온도센서 읽기\n"
                    "  mode       - 동작 모드 전환\n"
                    "\n"
                    "💡 명령어를 입력하고 Enter를 누르세요\n"
                    "\n> ";
                
                usb_serial_jtag_write_bytes((const uint8_t*)banner, strlen(banner), 1000);
                printf("%s", banner);
                fflush(stdout);
                banner_shown = true;
            }
            
            for (int i = 0; i < len; i++) {
                char c = data[i];
                
                if (c == '\n' || c == '\r') {
                    printf("\n");
                    input_buffer[pos] = '\0';
                    if (pos > 0) {
                        process_command(input_buffer);
                        pos = 0;
                    }
                    printf("> ");
                    fflush(stdout);
                }
                else if (c == 8 || c == 127) {  // Backspace
                    if (pos > 0) {
                        pos--;
                        // 강화된 백스페이스 에코백
                        const char backspace_seq[] = "\b \b";
                        usb_serial_jtag_write_bytes((const uint8_t*)backspace_seq, 3, 10);
                        printf("\b \b");
                        fflush(stdout);
                    }
                }
                else if (c >= 32 && c < 127 && pos < 99) {  // 출력 가능한 문자
                    input_buffer[pos++] = c;
                    // 강화된 에코백 - 직접 USB Serial JTAG로 전송
                    usb_serial_jtag_write_bytes(&c, 1, 10);
                    printf("%c", c);  // 추가 에코백
                    fflush(stdout);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}