/**
 * @file      RS485SoilSensor.hpp
 * @author    Sensor Project
 * @license   MIT
 * @date      2025-09-28
 *
 */

#pragma once

#include <HardwareSerial.h>
#include <ModbusMaster.h>

// Pin definitions for your board
#define TX_PIN 43 // DI des RS485-Moduls
#define RX_PIN 44  // RO des RS485-Moduls
#define RTS_PIN 15 // DE/RE gemeinsam

class RS485SoilSensor
{
public:
    // Sensor data structure
    struct SensorData
    {
        float moisture;    // %
        float temperature; // °C
        bool valid;        // Data validity flag
    };

#if defined(ARDUINO)
    RS485SoilSensor(HardwareSerial &serial, int rxPin = RX_PIN, int txPin = TX_PIN, int reDePin = RTS_PIN, uint8_t addr = 0x01)
    {
        __serial = &serial;
        __rxPin = rxPin;
        __txPin = txPin;
        __reDePin = reDePin;
        __addr = addr;
    }
#endif

    RS485SoilSensor()
    {
#if defined(ARDUINO)
        __serial = &Serial2; // Using Serial2 for RS485
        __rxPin = RX_PIN;    // RX pin 44
        __txPin = TX_PIN;    // TX pin 43
        __reDePin = RTS_PIN; // RE/DE pin 15
#endif
        __addr = 0x01;        // Default slave address
    }

    ~RS485SoilSensor()
    {
        deinit();
    }

    bool init()
    {
        return begin();
    }

    void deinit()
    {
        if (__serial) {
            __serial->end();
            Serial.println("RS485 Serial ended");
        }

        // Set RE/DE pin to low power state (receive mode, low power)
        if (__reDePin != -1)
        {
            digitalWrite(__reDePin, LOW);
            pinMode(__reDePin, INPUT); // Set to input to reduce current draw
        }

        // Set TX/RX pins to input to reduce power consumption
        if (__txPin != -1)
        {
            pinMode(__txPin, INPUT);
        }
        if (__rxPin != -1)
        {
            pinMode(__rxPin, INPUT);
        }

        Serial.println("RS485 pins set to low power mode");
    }

    // Read all sensor data at once
    SensorData readAllData()
    {
        SensorData data = {0};

        // Read 3 holding registers starting from address 0x0000
        uint8_t result = __modbus.readHoldingRegisters(0x0000, 3);

        if (result == __modbus.ku8MBSuccess)
        {
            // Parse response data
            float humidity = __modbus.getResponseBuffer(0) * 0.1;             // Convert to %
            float temperature = (int16_t)__modbus.getResponseBuffer(1) * 0.1; // Handle signed temp

            // Assign to data structure
            data.moisture = humidity;
            data.temperature = temperature;
            data.valid = true;

            // Debug output
            Serial.printf("Soil Sensor - Humidity: %.1f %%RH\n", data.moisture);
            Serial.printf("Soil Sensor - Temperature: %.1f °C\n", data.temperature);
            Serial.println("------------------------");
        }
        else
        {
            Serial.printf("Soil Sensor - Modbus Error: 0x%X\n", result);
            data.valid = false;
        }

        return data;
    }

    // Individual read functions for backward compatibility
    float readMoisture()
    {
        SensorData data = readAllData();
        return data.valid ? data.moisture : -1.0;
    }

    float readTemperature()
    {
        SensorData data = readAllData();
        return data.valid ? data.temperature : -999.0;
    }

    // Set sensor address (for configuration)
    bool setAddress(uint8_t newAddr)
    {
        __addr = newAddr;
        __modbus.begin(__addr, *__serial);
        return true;
    }

    // Check if sensor is responding
    bool isConnected()
    {
        // Try to read one register to test connection
        uint8_t result = __modbus.readHoldingRegisters(0x0000, 1);
        return (result == __modbus.ku8MBSuccess);
    }

private:
    ModbusMaster __modbus;

    bool begin()
    {
#if defined(ARDUINO)
        __serial->begin(4800, SERIAL_8N1, __rxPin, __txPin);

        printf("RS485 Serial started at 4800 bps on RX: %d, TX: %d\n", __rxPin, __txPin);

        // Initialize RE/DE control pin
        pinMode(__reDePin, OUTPUT);
        digitalWrite(__reDePin, LOW); // Start in receive mode

        // Initialize Modbus communication with slave address
        __modbus.begin(__addr, *__serial);

        // Set callbacks for pre and post transmission
        __modbus.preTransmission(preTransmission);
        __modbus.postTransmission(postTransmission);

        // Store instance pointer for static callbacks
        __instance = this;

        Serial.println("RS485 initialization complete");
        Serial.printf("Using RX: %d, TX: %d, RE/DE: %d\n", __rxPin, __txPin, __reDePin);

        // Test communication
        bool connected = isConnected();
        if (connected)
        {
            Serial.println("RS485 Soil Sensor: Connection test successful");
        }
        else
        {
            Serial.println("RS485 Soil Sensor: Connection test failed");
            Serial.println("Check wiring and sensor power");
        }

        return connected;
#endif
        return false;
    }

    // Static callback functions for RS485 transmission control
    static void preTransmission()
    {
        if (__instance)
        {
            digitalWrite(__instance->__reDePin, HIGH); // Enable transmit mode
            delayMicroseconds(10);
        }
    }

    static void postTransmission()
    {
        if (__instance)
        {
            digitalWrite(__instance->__reDePin, LOW); // Enable receive mode
            delayMicroseconds(10);
        }
    }

protected:
    HardwareSerial *__serial;
    int __rxPin;        // RX pin (44)
    int __txPin;        // TX pin (43)
    int __reDePin;      // RE/DE control pin (15)
    uint8_t __addr;     // Modbus slave address

    // Static instance pointer for callbacks
    static RS485SoilSensor *__instance;
};

// Static member definition
RS485SoilSensor *RS485SoilSensor::__instance = nullptr;
