// Only supports SX1276/SX1278
// 仅支持 SX1276/SX1278 无线电模块,SX1280,SX1262等其他无线电模块请使用RadioLibExamples目录的示例
#include <LoRa.h>
#include "LoRaBoards.h"
#include "RS485SoilSensor.hpp"
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ 868.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER 17
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW 125.0
#endif

// Deep sleep configuration
#define SLEEP_DURATION_MINUTES 1
#define uS_TO_S_FACTOR 1000000ULL                  // Conversion factor for micro seconds to seconds
#define SLEEP_TIME_S (SLEEP_DURATION_MINUTES * 60) // Sleep time in seconds

#if !defined(USING_SX1276) && !defined(USING_SX1278)
#error "LoRa example is only allowed to run SX1276/78. For other RF models, please run examples/RadioLibExamples"
// 仅支持 SX1276/SX1278 无线电模块,SX1280,SX1262等其他无线电模块请使用RadioLibExamples目录的示例
#endif

int counter = 0;
RTC_DATA_ATTR int bootCount = 0; // Store boot count in RTC memory to persist through deep sleep
RS485SoilSensor soilSensor;      // Create soil sensor instance
uint8_t deviceId;

struct __attribute__((packed)) SensorPayload
{
    uint8_t deviceId;    // 1 byte
    uint16_t moisture;   // 2 bytes
    int16_t temperature; // 2 bytes
    // total 5 bytes
};

// Function prototypes
void enterDeepSleep();
void disablePowerHungryComponents();

void setup()
{
    setupBoards();
    // When the power is turned on, a delay is required.
    delay(1500);

    // Print wakeup reason for debugging
    printWakeupReason();

    // Increment boot number and print it every reboot
    ++bootCount;
    counter = bootCount; // Use bootCount as counter for packet numbering
    Serial.println("Boot number: " + String(bootCount));

    // Generate single byte device ID from MAC address
    uint8_t mac[6];
    WiFi.macAddress(mac);
    deviceId = mac[5]; // Use last byte of MAC (0-255)

    Serial.print("Device ID: ");
    Serial.println(deviceId);

    // Disable power-hungry components early
    disablePowerHungryComponents();

    Serial.println("LoRa Sender - Deep Sleep Mode");
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
        // Give sensor time to stabilize after initialization
        delay(2000);
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

    // Give system time to stabilize after wakeup
    delay(1000);

    // Read all sensor data in one call (more efficient)
    RS485SoilSensor::SensorData sensorData = soilSensor.readAllData();

    // Retry up to 5 times if sensor read fails
    int retryCount = 0;
    const int maxRetries = 5;

    while (!sensorData.valid && retryCount < maxRetries)
    {
        retryCount++;
        Serial.printf("Sensor read failed (attempt %d/%d), retrying after delay...\n", retryCount, maxRetries);
        delay(2000);
        sensorData = soilSensor.readAllData();
    }

    if (!sensorData.valid)
    {
        Serial.printf("Sensor read failed after %d attempts\n", maxRetries);
    }
    else if (retryCount > 0)
    {
        Serial.printf("Sensor read successful after %d retries\n", retryCount);
    }

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

        // Create binary payload
        SensorPayload payload;
        payload.deviceId = deviceId;
        payload.moisture = (uint16_t)(sensorData.moisture * 10);
        payload.temperature = (int16_t)(sensorData.temperature * 10);

        // Send binary data
        LoRa.beginPacket();
        LoRa.write((uint8_t *)&payload, sizeof(payload));
        LoRa.endPacket();

        // Wait for LoRa transmission to complete
        Serial.println("LoRa packet sent, waiting for transmission to complete...");
        delay(1000);

        if (u8g2)
        {
            char buf[256];
            u8g2->clearBuffer();
            u8g2->drawStr(0, 12, "Transmitting: OK!");
            snprintf(buf, sizeof(buf), "Packet: %d", counter);
            u8g2->drawStr(0, 24, buf);
            snprintf(buf, sizeof(buf), "M:%.1f%% T:%.1fC", sensorData.moisture, sensorData.temperature);
            u8g2->drawStr(0, 48, buf);
            snprintf(buf, sizeof(buf), "Sleep: %dm", SLEEP_DURATION_MINUTES);
            u8g2->drawStr(0, 60, buf);
            u8g2->sendBuffer();
        }
    }
    else
    {
        Serial.println("Failed to read sensor data");

        // Error payload - just device ID and packet ID with error flag
        struct __attribute__((packed)) ErrorPayload
        {
            uint8_t deviceId;
            uint16_t packetId;
            uint8_t errorCode;
        } errorPayload;

        errorPayload.deviceId = deviceId;
        errorPayload.packetId = counter;
        errorPayload.errorCode = 1; // sensor read failed

        LoRa.beginPacket();
        LoRa.write((uint8_t *)&errorPayload, sizeof(errorPayload));
        LoRa.endPacket();

        // Wait for LoRa transmission to complete
        Serial.println("Error packet sent, waiting for transmission to complete...");
        delay(1000);

        if (u8g2)
        {
            char buf[256];
            u8g2->clearBuffer();
            u8g2->drawStr(0, 12, "Sensor Error!");
            snprintf(buf, sizeof(buf), "Packet: %d", counter);
            u8g2->drawStr(0, 24, buf);
            u8g2->drawStr(0, 36, "Check wiring");
            snprintf(buf, sizeof(buf), "Sleep: %dm", SLEEP_DURATION_MINUTES);
            u8g2->drawStr(0, 48, buf);
            u8g2->sendBuffer();
        }
    }

    // Wait longer for display to be visible and data to be processed
    Serial.println("Waiting before entering deep sleep...");
    delay(5000);

    // Enter deep sleep
    enterDeepSleep();
}

void disablePowerHungryComponents()
{
    Serial.println("Disabling power-hungry components...");

    // Disable WiFi completely
    WiFi.mode(WIFI_OFF);
    esp_wifi_deinit();
    Serial.println("WiFi disabled");

    // Disable Bluetooth
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    Serial.println("Bluetooth disabled");

// Turn off the onboard LED if available
#ifdef BOARD_LED
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, !LED_ON);
    Serial.println("Board LED turned off");
#endif
}

void enterDeepSleep()
{
    Serial.println("Preparing for deep sleep...");

    // Give time for any pending operations to complete
    delay(1000);

    // Deinitialize the soil sensor to save power
    soilSensor.deinit();
    Serial.println("RS485 sensor deinitialized");
    delay(500);

    // Turn off the LoRa radio
    LoRa.end();
    Serial.println("LoRa radio turned off");
    delay(500);

    // Turn off the display if it exists
    if (u8g2)
    {
        u8g2->clearBuffer();
        u8g2->drawStr(0, 12, "Entering");
        u8g2->drawStr(0, 24, "Deep Sleep...");
        u8g2->sendBuffer();
        delay(2000); // Longer delay to see the message
        u8g2->clearDisplay();
        Serial.println("Display cleared");
    }

    // Additional delay to ensure all components are properly shut down
    delay(1000);

    // Configure timer wakeup
    esp_sleep_enable_timer_wakeup(SLEEP_TIME_S * uS_TO_S_FACTOR);
    Serial.printf("Setup ESP32 to sleep for %d minutes\n", SLEEP_DURATION_MINUTES);

// Configure GPIO wakeup (optional - wake on button press)
#ifdef BUTTON_PIN
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0); // Wake on LOW
    Serial.println("Button wakeup enabled");
#endif

    Serial.println("Going to sleep now...");
    Serial.flush(); // Make sure all serial output is sent before sleeping

    // Enter deep sleep
    esp_deep_sleep_start();
}
