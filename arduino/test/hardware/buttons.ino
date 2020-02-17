#include <Arduino.h>
#include <HardwareSerial.h>

// #define ESP_SERIAL
// #include <ros.h>
// #include <pouco2000_ros/SwitchsMode.h> 
// #include <std_msgs/UInt8.h>

const int p_button = 4;
const int p_switch = 5;
const int p_pot = A0;

void setup() {
    pinMode(p_button,INPUT);
    pinMode(p_pot,INPUT);
    pinMode(p_switch,INPUT);
    Serial.begin(9600);
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
}

void loop() {
    Serial.print("test : ");
    Serial.print(digitalRead(p_button));
    Serial.print(" ");
    Serial.println(digitalRead(p_switch));
    Serial.print(" ");
    Serial.println(analogRead(p_pot));
    
    delay(100);
}
