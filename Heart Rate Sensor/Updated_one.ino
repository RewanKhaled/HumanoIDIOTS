#include <MAX30102.h>
#include <Wire.h>
#include <Pulse.h>
#include <LiquidCrystal_I2C.h>               

LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30102 sensor;
Pulse pulseIR;  // Create an instance of the Pulse class for IR signal processing
MAFilter bpm;  // Create an instance of the filter for calculating BPM

long lastBeat = 0;  // Store the time of the last detected beat
int beatAvg = 0;  // Store the average beats per minute (BPM)

void displayMessage(const String &line1, const String &line2) {
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
        while (1);  // Halt the program if sensor initialization fails
    }
    sensor.setup();  // Set up the sensor for use
}

void loop() {
    sensor.check();
    if (!sensor.available()) {
        delay(100); 
        return;
    }

    uint32_t irValue = sensor.getIR();  // Read the IR sensor value
    sensor.nextSample();
    if (irValue < 5000) {  // If the IR value is too low, ask for finger placement
        Serial.println("Place Finger To Start");
        displayMessage("Place Finger", "To Start");
        delay(200);
        return;
    }

    int16_t IR_signal = pulseIR.dc_filter(irValue);  // Filter the IR signal for DC component
    bool beatIR = pulseIR.isBeat(pulseIR.ma_filter(IR_signal));  // Check if a beat is detected

    if (beatIR) {   // If a beat is detected
        long btpm = 60000 / (millis() - lastBeat);  // Calculate beats per minute (BPM)
        if (btpm > 0 && btpm < 200) beatAvg = bpm.filter((int16_t)btpm);  // Filter the BPM value
        lastBeat = millis();  // Update the last beat time
    }

    lcd.setCursor(0, 0);
    Serial.print("Pulse Rate = ");
    Serial.println(beatAvg);
    lcd.print("Pulse Rate = ");
    lcd.print(beatAvg);
    lcd.setCursor(0, 1);
    Serial.println("Normal bpm= 60-100");
    lcd.print("bpm= 60-100");
}