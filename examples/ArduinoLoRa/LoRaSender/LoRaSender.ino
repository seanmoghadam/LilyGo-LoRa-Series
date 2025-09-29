// Only supports SX1276/SX1278
// 仅支持 SX1276/SX1278 无线电模块,SX1280,SX1262等其他无线电模块请使用RadioLibExamples目录的示例
#include <LoRa.h>
#include "LoRaBoards.h"
#include "RS485SoilSensor.hpp"

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ 868.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER 17
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW 125.0
#endif

#if !defined(USING_SX1276) && !defined(USING_SX1278)
#error "LoRa example is only allowed to run SX1276/78. For other RF models, please run examples/RadioLibExamples"
// 仅支持 SX1276/SX1278 无线电模块,SX1280,SX1262等其他无线电模块请使用RadioLibExamples目录的示例
#endif

int counter = 0;
RS485SoilSensor soilSensor; // Create soil sensor instance

void setup()
{
    setupBoards();
    // When the power is turned on, a delay is required.
    delay(1500);

    Serial.println("LoRa Sender");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(CONFIG_RADIO_FREQ * 1000000))
    {
        Serial.println("Starting LoRa failed!");
        while (1)
            ;
    }

    LoRa.setTxPower(CONFIG_RADIO_OUTPUT_POWER);

    LoRa.setSignalBandwidth(CONFIG_RADIO_BW * 1000);

    LoRa.setSpreadingFactor(10);

    LoRa.setPreambleLength(16);

    LoRa.setSyncWord(0xAB);

    LoRa.disableCrc();

    LoRa.disableInvertIQ();

    LoRa.setCodingRate4(7);

    // Initialize soil sensor
    Serial.println("Initializing RS485 Soil Sensor...");
    if (soilSensor.init())
    {
        Serial.println("Soil sensor initialized successfully");
    }
    else
    {
        Serial.println("Failed to initialize soil sensor");
    }
}

void loop()
{
    Serial.print("Sending packet: ");
    Serial.println(counter);

    // Read all sensor data in one call (more efficient)
    RS485SoilSensor::SensorData sensorData = soilSensor.readAllData();

    if (sensorData.valid)
    {
        // Print sensor readings to serial
        Serial.print("Moisture: ");
        Serial.print(sensorData.moisture);
        Serial.println("%");
        Serial.print("Temperature: ");
        Serial.print(sensorData.temperature);
        Serial.println("°C");
        Serial.println("-----------------------------");

        // Create JSON packet with all sensor data
        String jsonData = "{";
        jsonData += "\"id\":" + String(counter) + ",";
        jsonData += "\"moisture\":" + String(sensorData.moisture, 1) + ",";
        jsonData += "\"temp\":" + String(sensorData.temperature, 1) + ",";
        jsonData += "}";

        // Send packet via LoRa
        LoRa.beginPacket();
        LoRa.print(jsonData);
        LoRa.endPacket();

        if (u8g2)
        {
            char buf[256];
            u8g2->clearBuffer();
            u8g2->drawStr(0, 12, "Transmitting: OK!");
            snprintf(buf, sizeof(buf), "Packet: %d", counter);
            u8g2->drawStr(0, 24, buf);
            snprintf(buf, sizeof(buf), "M:%.1f%% T:%.1fC", sensorData.moisture, sensorData.temperature);
            u8g2->drawStr(0, 48, buf);
            u8g2->sendBuffer();
        }
    }
    else
    {
        Serial.println("Failed to read sensor data");

        // Send error packet
        String errorData = "{\"id\":" + String(counter) + ",\"error\":\"sensor_read_failed\"}";
        LoRa.beginPacket();
        LoRa.print(errorData);
        LoRa.endPacket();

        if (u8g2)
        {
            char buf[256];
            u8g2->clearBuffer();
            u8g2->drawStr(0, 12, "Sensor Error!");
            snprintf(buf, sizeof(buf), "Packet: %d", counter);
            u8g2->drawStr(0, 24, buf);
            u8g2->drawStr(0, 36, "Check wiring");
            u8g2->sendBuffer();
        }
    }

    counter++;
    delay(5000);
}
