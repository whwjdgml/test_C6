
#ifndef DISPLAY_UTILS_H
#define DISPLAY_UTILS_H

#include "sensor_types.h"

class DisplayUtils {
public:
    // 시스템 정보 출력
    static void printSystemInfo();
    
    // 센서 데이터 출력 (정밀도 순서)
    static void printSensorData(const SensorData &data);
    
    // 센서 융합 데이터 출력
    static void printFusedData(const FusedSensorData &fused);
    
    // 측정 헤더 출력
    static void printMeasurementHeader(uint32_t count, uint32_t uptime_sec);
    
    // 진단 결과 출력
    static void printDiagnostics(const SensorData &data);
    
    // 시작 메시지 출력
    static void printStartupMessage();

private:
    // 내부 출력 함수들
    static void printTableHeader(const char* title);
    static void printTableFooter();
    static void printTableRow(const char* label, const char* value, const char* unit = "");
    static void printSeparator();
    
    // 상태 아이콘
    static const char* getStatusIcon(bool available);
    static const char* getReliabilityIcon(float diff);
};

#endif // DISPLAY_UTILS_H