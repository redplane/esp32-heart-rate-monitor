#include <Arduino.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <ModbusRTU.h>

//#region Modbus RTU

#define SOFTWARE_SERIAL_TX_PIN          32
#define SOFTWARE_SERIAL_RX_PIN          33
#define SOFTWARE_SERIAL_BAUD_RATE       9600

int modbusSlaveId = 1;
SoftwareSerial softwareSerial;

ModbusRTU modbusRtu;

//#endregion

//#region ECG


#define PIN_LOW_NEGATIVE        2
#define PIN_LOW_POSITIVE        15
#define PIN_HEART_RATE_OUTPUT   34


float _ecgValue = 9999.0;
unsigned long _measurementTime = 0;

TaskHandle_t _doMeasurementTask;

//#endregion


void buildModbusRtuMessage() {

    float localEcgValue = 9999.0;

    if (!isnan(_ecgValue)) {
        localEcgValue = _ecgValue;
    }

    uint16_t ecgRegisters[2];
    memcpy(ecgRegisters, &localEcgValue, 4);

    modbusRtu.removeHreg(0, 2);

    modbusRtu.addHreg(0, ecgRegisters[0], 1);
    modbusRtu.addHreg(1, ecgRegisters[1], 1);
}

// Do weather query from api.
static void doMeasurement(void *parameter) {
    while (true) {
        auto measurementDelay = millis() - _measurementTime;
        if (measurementDelay >= 300) {

            _measurementTime = millis();

            if ((digitalRead(PIN_LOW_POSITIVE) == 1) || (digitalRead(PIN_LOW_NEGATIVE) == 1)) {
                Serial.println('!');
                _ecgValue = 9999.0;
            } else {
                auto ecgValue = analogRead(PIN_HEART_RATE_OUTPUT);
                _ecgValue = (float) ecgValue;
                Serial.println(ecgValue);
            }


        }
        vTaskDelay(20);
    }
}


void setup() {

    Serial.begin(115200);
    Serial.println(F("Start weather ..."));

    // Modbus RTU begin
    softwareSerial.begin(SOFTWARE_SERIAL_BAUD_RATE, SWSERIAL_8N1, SOFTWARE_SERIAL_RX_PIN,
                         SOFTWARE_SERIAL_TX_PIN, false, 95, 11);

    modbusRtu.begin(&softwareSerial);
    modbusRtu.slave(modbusSlaveId);

    xTaskCreatePinnedToCore(
            doMeasurement,   /* Task function. */
            "doMeasurement",     /* name of task. */
            10000,       /* Stack size of task */
            nullptr,        /* parameter of the task */
            1,           /* priority of the task */
            &_doMeasurementTask,      /* Task handle to keep track of created task */
            0);          /* pin task to core 0 */

    buildModbusRtuMessage();
}

void loop() {

    buildModbusRtuMessage();
    modbusRtu.task();
    yield();
}