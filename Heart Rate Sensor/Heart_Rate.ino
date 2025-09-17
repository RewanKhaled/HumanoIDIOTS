#include <MAX30102.h>
#include <Wire.h>
#include <Pulse.h>
#include <LiquidCrystal_I2C.h>               
#include <avr/sleep.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;

long lastBeat = 0;
int beatAvg = 0, SPO2 = 0, SPO2f = 0;
uint8_t istate = 0, sleep_counter = 0;

const uint8_t spo2_table[184] PROGMEM = {
    100, 99, 98, 97, 97, 96, 95, 95, 94, 93, 92, 92, 91, 90, 89, 89,
    88, 87, 86, 86, 85, 84, 83, 83, 82, 81, 80, 80, 79, 78, 77, 77,
    76, 75, 74, 74, 73, 72, 71, 71, 70, 69, 68, 68, 67, 66, 65, 65,
    64, 63, 62, 62, 61, 60, 59, 59, 58, 57, 56, 56, 55, 54, 53, 53,
    52, 51, 50, 50, 49, 48, 47, 47, 46, 45, 44, 44, 43, 42, 41, 41,
    40, 39, 38, 38, 37, 36, 35, 35, 34, 33, 32, 32, 31, 30, 29, 29,
    28, 27, 26, 26, 25, 24, 23, 23, 22, 21, 20, 20, 19, 18, 17, 17,
    16, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 5,
    4, 3, 2, 2, 1, 0
};

void displayMessage(const String &line1, const String &line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

void setup() {
    lcd.begin(16, 2);
    lcd.init();       
    lcd.backlight();   
    lcd.clear();  
    Serial.begin(115200);
    Serial.println("Sensor is Ready");
    lcd.print("Sensor is Ready");
    delay(2000);

    if (!sensor.begin()) {
        Serial.println("Sensor Error Restart Required");
        displayMessage("Sensor Error", "Restart Required");
        while (1);
    }
    sensor.setup();
}

void loop() {
    sensor.check();
    if (!sensor.available()) {
        delay(100); 
        return;
    }

    uint32_t irValue = sensor.getIR();
    uint32_t redValue = sensor.getRed();
    sensor.nextSample();

    if (irValue < 5000) {
        Serial.println("Place Finger To Start");
        displayMessage("Place Finger", "To Start");
        delay(200);
        return;
    }

    int16_t IR_signal = pulseIR.dc_filter(irValue);
    bool beatIR = pulseIR.isBeat(pulseIR.ma_filter(IR_signal));

    if (beatIR) {
        long btpm = 60000 / (millis() - lastBeat);
        if (btpm > 0 && btpm < 200) beatAvg = bpm.filter((int16_t)btpm);
        lastBeat = millis();

        long numerator   = (pulseRed.avgAC() * pulseIR.avgDC()) / 256;
        long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / 256;
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    Serial.print("Pulse Rate = ");
    Serial.println(beatAvg);
    lcd.print("Pulse Rate = ");
    lcd.print(beatAvg);
    lcd.setCursor(0, 1);
    Serial.println("Normal bpm= 60-100");
    lcd.print("bpm= 60-100");
}
