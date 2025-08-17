#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Arduino.h를 직접 포함하여 아두이노 기능 사용
#include <Arduino.h>
#include <Wire.h>

// app_main 함수는 C 스타일로 호출되어야 하므로 extern "C"로 감싸줌
extern "C" void app_main(void)
{
    // 아두이노 프레임워크 초기화 (필수)
    initArduino();

    // 아두이노의 Serial 함수 사용
    Serial.begin(115200);
    while(!Serial);

    Serial.println("ESP-IDF with Arduino Component Test");
    Serial.println("Scanning I2C bus...");

    // 아두이노의 Wire 라이브러리 사용
    Wire.begin(6, 7); // XIAO C6의 기본 I2C 핀 (SDA=6, SCL=7)

    byte error, address;
    int nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        }
        else if (error==4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else

    while (1)
    {
        // ESP-IDF의 vTaskDelay 함수 사용
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}