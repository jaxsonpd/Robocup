/** 
 * @file BluetoothForward.ino
 * @brief This is the sketch file to forward data from the serial port to the Bluefruit LE module, and vice versa.
 * @author Jack Duignan (JackpDuignan@gmail.com)
 * @date 2023-08-03
 */


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

#include "BluefruitConfig.h"
// ===================================== Types/Constants ==============================
#define FACTORYRESET_ENABLE 1 // Factory reset feature is enabled if set to 1
#define VERBOSE_MODE false // If set to true, debug information will be printed to the serial port

#define SERIAL_BAUD_RATE 115200
#define SERIAL1_BAUD_RATE 115200

#define BUFSIZE 128

// Uncomment to simulate data from the Serial1 port
#define SIMULATE

// ===================================== Globals ======================================
// Create a bluefruit object using hardware SPI (SCK/MOSI/MISO) pin definitions in BluefruitConfig.h
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// ===================================== Function Definitions =========================
/** 
 * @brief Initialize the Bluefruit LE module
 * @param verboseMode Whether or not to print debug information to the serial port
 * @param factoryReset Whether or not to perform a factory reset
 * 
 * @return true if initialization was successful
 */
bool bluetooth_init(bool verboseMode, bool factoryReset) {
    // Initialize the module
    Serial.println(F("Initializing the Bluefruit LE module:"));

    if (!ble.begin(VERBOSE_MODE)) {
        Serial.println(F("Couldn't find Bluefruit, make sure it's in Command mode & check wiring?"));
        return false;
    }

    Serial.println("OK!");

    // Perform a factory reset to make sure everything is in a known state
    if (factoryReset) {
        Serial.println(F("Performing a factory reset: "));
        if (!ble.factoryReset()) {
            Serial.println(F("Couldn't factory reset"));
            return false;
        }
    }

    // Disable command echo from Bluefruit
    ble.echo(false);

    // Print Bluefruit information
    Serial.println(F("Requesting Bluefruit info:"));
    ble.info();

    ble.verbose(verboseMode);  // debug info is a little annoying after this point!

    Serial.println( F("Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);
    return true;
}

/** 
 * @brief Initalise the serial ports and print a welcome message
 * @param serialBaudRate The baud rate for the usb serial port
 * @param serial1BaudRate The baud rate for the Serial1 port
 * 
 * @return true if initialization was successful
 */
bool serial_init(uint32_t serialBaudRate, uint32_t serial1BaudRate) {
    // Initialize the serial ports
    Serial.begin(serialBaudRate);
    Serial1.begin(serial1BaudRate);

    delay(1000);

    // Print a welcome message
    Serial.print(F("Serial port initialized at: "));
    Serial.print(serialBaudRate);
    Serial.println(F(" baud"));
    
    Serial.print(F("Serial1 port initialized at: "));
    Serial.print(serial1BaudRate);
    Serial.println(F(" baud"));

    return true;
}


void setup(void) {
    // Initialize the serial ports
    if(!serial_init(SERIAL_BAUD_RATE, SERIAL1_BAUD_RATE)) {
        Serial.println(F("Serial port initialization failed!"));
        while (1);
    }

    Serial.println(" ***************************************** ");

    // Initialize the Bluefruit LE module
    if(!bluetooth_init(VERBOSE_MODE, FACTORYRESET_ENABLE)) {
        Serial.println(F("Bluetooth initialization failed!"));
        while (1);
    }


    Serial.println(F("Setup complete!"));

    /* Wait for connection */
    while (!ble.isConnected()) {
        delay(500);
    }

    Serial.println(" ***************************************** ");
}

void loop(void) {
    // Forward data from the serial port to the Bluefruit LE module
    // Check for user input
    if (Serial1.available()) {
        char inputs[BUFSIZE+1];
        size_t n;

        // Read user input
        Serial1.readBytesUntil('\n', inputs, BUFSIZE);
        inputs[n] = 0;

        Serial.print(F("Sending: "));
        Serial.println(inputs);

        // Send user input to the Bluefruit LE module
        ble.print(inputs);
    }

    // Forward data from the Bluefruit LE module to the serial port
    // Check for data from the Bluefruit LE module
    if (ble.available()) {
        char c = ble.read();
        Serial.println(c);

        // Send data to the serial1 port
        Serial1.print(c);
    }

    #ifdef SIMULATE // Simulate data from the Serial1 port
        char buffer[BUFSIZE];

        sprintf(buffer, "Current time: %lu ms\n", millis());

        Serial.print("Sending -> ");
        Serial.print(buffer);

        ble.print(buffer);

        delay(2000);
    #endif // SIMULATE
}