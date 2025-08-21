/**
 * @file display_utils.h
 * @brief 콘솔 출력을 위한 유틸리티 클래스 정의
 *
 * 이 파일은 센서 데이터, 시스템 정보, 진단 결과 등을
 * 일관된 형식으로 콘솔에 출력하는 Helper 함수들을 포함하는
 * DisplayUtils 클래스를 정의합니다. 모든 함수는 정적(static)으로
 * 선언되어 객체 생성 없이 사용할 수 있습니다.
 */
#ifndef DISPLAY_UTILS_H
#define DISPLAY_UTILS_H

#include "sensor_types.h"

/**
 * @class DisplayUtils
 * @brief 데이터 출력을 위한 정적 유틸리티 함수 모음
 */
class DisplayUtils {
public:
    /**
     * @brief 시스템의 기본 정보를 출력합니다. (ESP-IDF 버전, 칩 정보 등)
     */
    static void printSystemInfo();
    
    /**
     * @brief 원시 센서 데이터를 정밀도 순서로 정렬하여 출력합니다.
     * @param data 출력할 센서 데이터 구조체
     */
    static void printSensorData(const SensorData &data);
    
    /**
     * @brief 여러 센서의 값을 융합(평균 등)한 최종 데이터를 출력합니다.
     * @param fused 출력할 융합된 센서 데이터 구조체
     */
    static void printFusedData(const FusedSensorData &fused);
    
    /**
     * @brief 주기적인 측정값 출력의 헤더를 표시합니다.
     * @param count 현재까지의 측정 횟수
     * @param uptime_sec 시스템 가동 시간 (초)
     */
    static void printMeasurementHeader(uint32_t count, uint32_t uptime_sec);
    
    /**
     * @brief 센서 진단 결과를 상세히 출력합니다.
     * @param data 진단에 사용될 센서 데이터
     */
    static void printDiagnostics(const SensorData &data);
    
    /**
     * @brief 프로그램 시작 시 환영 메시지를 출력합니다.
     */
    static void printStartupMessage();

private:
    /**
     * @brief 테이블 형식의 출력에서 제목 부분을 출력합니다.
     * @param title 테이블의 제목
     */
    static void printTableHeader(const char* title);

    /**
     * @brief 테이블 형식의 출력에서 꼬리말(닫는 선)을 출력합니다.
     */
    static void printTableFooter();

    /**
     * @brief 테이블의 한 행을 '레이블: 값 단위' 형식으로 출력합니다.
     * @param label 행의 제목 (예: "온도")
     * @param value 출력할 값 (문자열)
     * @param unit 값의 단위 (예: "°C")
     */
    static void printTableRow(const char* label, const char* value, const char* unit = "");

    /**
     * @brief 구분선을 출력합니다.
     */
    static void printSeparator();
    
    /**
     * @brief 센서의 작동 상태(온라인/오프라인)에 따라 아이콘을 반환합니다.
     * @param available 센서 작동 여부
     * @return const char* 상태를 나타내는 아이콘 문자열 (예: "✅", "❌")
     */
    static const char* getStatusIcon(bool available);

    /**
     * @brief 센서 값의 편차에 따라 신뢰도 아이콘을 반환합니다.
     * @param diff 다른 센서와의 값 차이
     * @return const char* 신뢰도를 나타내는 아이콘 문자열
     */
    static const char* getReliabilityIcon(float diff);
};

#endif // DISPLAY_UTILS_H