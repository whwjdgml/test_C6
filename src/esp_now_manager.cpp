#include "esp_now_manager.h"
#include "esp_timer.h"
#include "esp_mac.h"

const char* ESPNowManager::TAG = "ESP_NOW_MGR";

// 전역 인스턴스 포인터 (콜백 함수용)
static ESPNowManager* g_espnow_instance = nullptr;

ESPNowManager::ESPNowManager() {
    g_espnow_instance = this;
}

ESPNowManager::~ESPNowManager() {
    deinit();
    g_espnow_instance = nullptr;
}

bool ESPNowManager::init() {
    ESP_LOGI(TAG, "ESP-NOW 매니저 초기화 시작");
    
    // WiFi 초기화
    if (!initWiFi()) {
        ESP_LOGE(TAG, "WiFi 초기화 실패");
        return false;
    }
    
    // ESP-NOW 초기화
    if (!initESPNow()) {
        ESP_LOGE(TAG, "ESP-NOW 초기화 실패");
        return false;
    }
    
    // 브로드캐스트 피어 추가
    if (!addPeer(broadcast_mac)) {
        ESP_LOGW(TAG, "브로드캐스트 피어 추가 실패");
    }
    
    ESP_LOGI(TAG, "✅ ESP-NOW 매니저 초기화 완료");
    
    // 자신의 MAC 주소 출력
    uint8_t my_mac[6];
    esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "내 MAC 주소: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
    
    return true;
}

bool ESPNowManager::initWiFi() {
    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // WiFi 초기화
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // 채널 1 고정 (ESP-NOW는 같은 채널에서만 통신 가능)
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    
    ESP_LOGI(TAG, "WiFi STA 모드 초기화 완료 (채널 1)");
    return true;
}

bool ESPNowManager::initESPNow() {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(onDataReceived));
    ESP_ERROR_CHECK(esp_now_register_send_cb(onDataSent));
    
    ESP_LOGI(TAG, "ESP-NOW 프로토콜 초기화 완료");
    return true;
}

void ESPNowManager::onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (g_espnow_instance && g_espnow_instance->on_data_received) {
        g_espnow_instance->on_data_received(recv_info->src_addr, data, len);
    }
    
    ESP_LOGD(ESPNowManager::TAG, "데이터 수신: %d bytes from %02X:%02X:%02X:%02X:%02X:%02X",
             len, recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
}

void ESPNowManager::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (g_espnow_instance) {
        if (status == ESP_NOW_SEND_SUCCESS) {
            g_espnow_instance->packets_sent++;
        } else {
            g_espnow_instance->packets_failed++;
        }
        
        if (g_espnow_instance->on_send_complete) {
            g_espnow_instance->on_send_complete(mac_addr, status);
        }
    }
    
    ESP_LOGV(ESPNowManager::TAG, "전송 완료: %s to %02X:%02X:%02X:%02X:%02X:%02X",
             (status == ESP_NOW_SEND_SUCCESS) ? "성공" : "실패",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
}

bool ESPNowManager::addPeer(const uint8_t* mac_address) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac_address, 6);
    peerInfo.channel = 1;  // 채널 1 고정
    peerInfo.encrypt = false;  // 암호화 없음 (배터리 절약)
    
    esp_err_t result = esp_now_add_peer(&peerInfo);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "피어 추가 성공: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_address[0], mac_address[1], mac_address[2],
                 mac_address[3], mac_address[4], mac_address[5]);
        return true;
    } else if (result == ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGD(TAG, "피어 이미 존재함");
        return true;
    } else {
        ESP_LOGE(TAG, "피어 추가 실패: %s", esp_err_to_name(result));
        return false;
    }
}

bool ESPNowManager::setReceiverMAC(const uint8_t* mac_address) {
    memcpy(receiver_mac, mac_address, 6);
    receiver_paired = addPeer(mac_address);
    
    if (receiver_paired) {
        ESP_LOGI(TAG, "수신기 MAC 설정: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_address[0], mac_address[1], mac_address[2],
                 mac_address[3], mac_address[4], mac_address[5]);
    }
    
    return receiver_paired;
}

void ESPNowManager::fillPacketHeader(ESPNowPacketHeader* header, espnow_packet_type_t type, uint16_t data_len) {
    header->packet_type = type;
    header->sequence_number = sequence_counter++;
    header->timestamp_ms = esp_timer_get_time() / 1000;
    header->data_length = data_len;
    
    // 자신의 MAC 주소 추가
    esp_read_mac(header->sender_mac, ESP_MAC_WIFI_STA);
}

bool ESPNowManager::sendHeaterStatus(const HeaterStatusPacket& packet) {
    esp_err_t result = esp_now_send(receiver_paired ? receiver_mac : broadcast_mac,
                                   (uint8_t*)&packet, sizeof(HeaterStatusPacket));
    
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "히터 상태 전송 실패: %s", esp_err_to_name(result));
        packets_failed++;
        return false;
    }
    
    ESP_LOGV(TAG, "히터 상태 전송: %.2f°C → %d%% PWM", 
             packet.battery_temperature, packet.pwm_duty_percent);
    return true;
}

bool ESPNowManager::sendTemperatureLog(const TemperatureLogPacket& packet) {
    esp_err_t result = esp_now_send(receiver_paired ? receiver_mac : broadcast_mac,
                                   (uint8_t*)&packet, sizeof(TemperatureLogPacket));
    
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "온도 로그 전송 실패: %s", esp_err_to_name(result));
        packets_failed++;
        return false;
    }
    
    ESP_LOGD(TAG, "온도 로그 전송: %d개 데이터", packet.log_count);
    return true;
}

bool ESPNowManager::sendSystemInfo(const SystemInfoPacket& packet) {
    esp_err_t result = esp_now_send(receiver_paired ? receiver_mac : broadcast_mac,
                                   (uint8_t*)&packet, sizeof(SystemInfoPacket));
    
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "시스템 정보 전송 실패: %s", esp_err_to_name(result));
        packets_failed++;
        return false;
    }
    
    ESP_LOGD(TAG, "시스템 정보 전송: 가동시간 %lu초", packet.uptime_sec);
    return true;
}

bool ESPNowManager::sendHeartbeat() {
    struct {
        ESPNowPacketHeader header;
        uint32_t heartbeat_counter;
        float battery_temp;
        bool heater_active;
    } heartbeat_packet;
    
    fillPacketHeader(&heartbeat_packet.header, PACKET_HEARTBEAT, sizeof(heartbeat_packet) - sizeof(ESPNowPacketHeader));
    heartbeat_packet.heartbeat_counter = packets_sent;
    heartbeat_packet.battery_temp = 0.0f;  // 실제 온도는 별도 설정
    heartbeat_packet.heater_active = false; // 실제 상태는 별도 설정
    
    esp_err_t result = esp_now_send(receiver_paired ? receiver_mac : broadcast_mac,
                                   (uint8_t*)&heartbeat_packet, sizeof(heartbeat_packet));
    
    if (result == ESP_OK) {
        last_heartbeat_ms = esp_timer_get_time() / 1000;
        ESP_LOGV(TAG, "하트비트 전송: #%lu", heartbeat_packet.heartbeat_counter);
        return true;
    } else {
        ESP_LOGW(TAG, "하트비트 전송 실패: %s", esp_err_to_name(result));
        packets_failed++;
        return false;
    }
}

bool ESPNowManager::processControlCommand(const ControlCommandPacket& packet) {
    ESP_LOGI(TAG, "제어 명령 수신: 타입 %d", packet.command_type);
    
    // TODO: 실제 제어 명령 처리 로직 구현
    // 예: 목표 온도 변경, 히터 활성화/비활성화 등
    
    return true;
}

float ESPNowManager::getSuccessRate() const {
    uint32_t total = packets_sent + packets_failed;
    if (total == 0) return 100.0f;
    return (float)packets_sent / total * 100.0f;
}

void ESPNowManager::setDataReceivedCallback(void (*callback)(const uint8_t *mac, const uint8_t *data, int len)) {
    on_data_received = callback;
}

void ESPNowManager::setSendCompleteCallback(void (*callback)(const uint8_t *mac, esp_now_send_status_t status)) {
    on_send_complete = callback;
}

void ESPNowManager::printMAC(const uint8_t* mac) {
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void ESPNowManager::printStatistics() {
    ESP_LOGI(TAG, "=== ESP-NOW 통신 통계 ===");
    ESP_LOGI(TAG, "전송 성공: %lu개", packets_sent);
    ESP_LOGI(TAG, "전송 실패: %lu개", packets_failed);
    ESP_LOGI(TAG, "성공률: %.1f%%", getSuccessRate());
    ESP_LOGI(TAG, "수신기 연결: %s", receiver_paired ? "연결됨" : "미연결");
    ESP_LOGI(TAG, "마지막 하트비트: %lu초 전", (esp_timer_get_time() / 1000 - last_heartbeat_ms) / 1000);
}

void ESPNowManager::deinit() {
    if (esp_now_deinit() == ESP_OK) {
        ESP_LOGI(TAG, "ESP-NOW 정리 완료");
    }
    
    receiver_paired = false;
    packets_sent = 0;
    packets_failed = 0;
}