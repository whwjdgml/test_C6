#ifndef ESP_NOW_MANAGER_H
#define ESP_NOW_MANAGER_H

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <cstring>

// ESP-NOW 패킷 타입 정의
typedef enum {
    PACKET_HEATER_STATUS = 0x01,     // 히터 상태 데이터
    PACKET_TEMPERATURE_LOG = 0x02,   // 온도 로그 데이터
    PACKET_CONTROL_COMMAND = 0x03,   // 원격 제어 명령
    PACKET_SYSTEM_INFO = 0x04,       // 시스템 정보
    PACKET_HEARTBEAT = 0x05          // 하트비트 (연결 확인)
} espnow_packet_type_t;

// 공통 패킷 헤더
struct ESPNowPacketHeader {
    uint8_t packet_type;             // 패킷 타입
    uint8_t sequence_number;         // 순서 번호 (재전송 감지용)
    uint32_t timestamp_ms;           // 타임스탬프 (ms)
    uint8_t sender_mac[6];          // 송신자 MAC 주소
    uint16_t data_length;           // 데이터 길이
} __attribute__((packed));

// 히터 상태 패킷 (가장 중요한 데이터)
struct HeaterStatusPacket {
    ESPNowPacketHeader header;
    float battery_temperature;       // 배터리 온도 (°C)
    float target_temperature;       // 목표 온도 (°C)
    uint8_t pwm_duty_percent;       // PWM 듀티비 (%)
    uint8_t heater_state;           // 히터 상태 (heater_state_t)
    float power_consumption;        // 전력 소모 (W)
    bool stepup_enabled;            // 스텝업 컨버터 상태
    float proportional_gain;        // 현재 Kp 값
    float average_error;            // 평균 제어 오차 (°C)
    uint32_t heating_cycles;        // 총 가열 사이클 수
    uint32_t total_heating_time_sec; // 총 가열 시간 (초)
} __attribute__((packed));

// 온도 로그 패킷 (여러 측정값을 한번에 전송)
struct TemperatureLogPacket {
    ESPNowPacketHeader header;
    uint8_t log_count;              // 로그 개수 (최대 10개)
    struct {
        uint32_t timestamp_offset_sec;  // 기준 시간으로부터 오프셋 (초)
        float temperature;              // 온도 (°C)
        uint8_t pwm_duty;              // PWM 듀티 (%)
        float error;                   // 온도 오차 (°C)
    } logs[10];
} __attribute__((packed));

// 제어 명령 패킷 (원격에서 히터 제어)
struct ControlCommandPacket {
    ESPNowPacketHeader header;
    uint8_t command_type;           // 명령 타입
    union {
        struct {
            float new_target_temp;      // 새로운 목표 온도
        } set_target;
        struct {
            float new_kp;              // 새로운 비례 게인
        } set_gain;
        struct {
            bool enable;               // 히터 활성화/비활성화
        } heater_enable;
        uint8_t raw_data[32];          // 기타 명령용 원시 데이터
    } data;
} __attribute__((packed));

// 시스템 정보 패킷
struct SystemInfoPacket {
    ESPNowPacketHeader header;
    uint32_t uptime_sec;            // 가동 시간 (초)
    uint32_t free_heap_size;        // 사용 가능한 힙 메모리
    float battery_voltage;          // 배터리 전압 (V)
    float battery_current;          // 배터리 전류 (mA)
    uint8_t wifi_rssi;             // WiFi 신호 세기
    uint8_t error_count;           // 누적 오류 개수
    char firmware_version[16];      // 펌웨어 버전
} __attribute__((packed));

// ESP-NOW 통신 매니저 클래스
class ESPNowManager {
private:
    static const char* TAG;
    
    // 통신 설정
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t receiver_mac[6] = {0};     // 수신기 MAC 주소
    bool receiver_paired = false;
    
    // 패킷 관리
    uint8_t sequence_counter = 0;
    uint32_t packets_sent = 0;
    uint32_t packets_failed = 0;
    uint32_t last_heartbeat_ms = 0;
    
    // 콜백 함수 포인터
    void (*on_data_received)(const uint8_t *mac, const uint8_t *data, int len) = nullptr;
    void (*on_send_complete)(const uint8_t *mac, esp_now_send_status_t status) = nullptr;
    
    // 내부 메서드
    bool initWiFi();
    bool initESPNow();
    static void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
    static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);

public:
    ESPNowManager();
    ~ESPNowManager();
    
    // 초기화 및 설정
    bool init();
    bool addPeer(const uint8_t* mac_address);
    bool setReceiverMAC(const uint8_t* mac_address);
    void setDataReceivedCallback(void (*callback)(const uint8_t *mac, const uint8_t *data, int len));
    void setSendCompleteCallback(void (*callback)(const uint8_t *mac, esp_now_send_status_t status));
    
    // 데이터 전송 메서드들
    bool sendHeaterStatus(const HeaterStatusPacket& packet);
    bool sendTemperatureLog(const TemperatureLogPacket& packet);
    bool sendSystemInfo(const SystemInfoPacket& packet);
    bool sendHeartbeat();
    
    // 원격 제어 명령 처리
    bool processControlCommand(const ControlCommandPacket& packet);
    
    // 통계 및 상태
    uint32_t getPacketsSent() const { return packets_sent; }
    uint32_t getPacketsFailed() const { return packets_failed; }
    float getSuccessRate() const;
    bool isReceiverPaired() const { return receiver_paired; }
    
    // 유틸리티
    void printMAC(const uint8_t* mac);
    void printStatistics();
    void fillPacketHeader(ESPNowPacketHeader* header, espnow_packet_type_t type, uint16_t data_len);
    
    void deinit();
};

#endif // ESP_NOW_MANAGER_H