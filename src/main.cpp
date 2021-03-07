#include <Arduino.h>
#include <Wire.h>

#define PIN_LOW_NEGATIVE        2
#define PIN_LOW_POSITIVE        3
#define PIN_HEART_RATE_OUTPUT   A0

// Function that executes whenever data is received from master
void receiveEvent(int howMany) {
    Wire.write("Hello world");
    return;
    Serial.println("(receiveEvent) start");
    if ((digitalRead(PIN_LOW_POSITIVE) == 1) || (digitalRead(PIN_LOW_NEGATIVE) == 1)) {
        Wire.write(-1);
        Serial.print("ECG value = ");
        Serial.println(-1);
        return;
    }

    // send the value of analog input 0:
    auto ecgValue = analogRead(PIN_HEART_RATE_OUTPUT);
    Wire.write(ecgValue);
    Serial.print("ECG value = ");
    Serial.println(ecgValue);
}

void setup() {

    // initialize the serial communication:
    Serial.begin(9600);

    Wire.onReceive(receiveEvent);
    Wire.begin(0x08);


    // write your initialization code here
    pinMode(PIN_LOW_NEGATIVE, INPUT);
    pinMode(PIN_LOW_POSITIVE, INPUT);

    Serial.println("DONE");

}

void loop() {
//    if((digitalRead(PIN_LOW_POSITIVE) == 1)||(digitalRead(PIN_LOW_NEGATIVE) == 1)){
//        Serial.println('!');
//    }
//    else{
//
////        analogSetAttenuation(ADC_11db);
//        // send the value of analog input 0:
//        Serial.println(analogRead(PIN_HEART_RATE_OUTPUT));
//    }
    //Wait for a bit to keep serial data from saturating
//    delay(50);
}