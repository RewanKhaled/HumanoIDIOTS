#include <Wire.h>
#include <LiquidCrystal_I2C.h>   
#include "MAX30105.h"
LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30105 particleSensor;

void setup()
{
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.init();       
  lcd.backlight();   
  lcd.clear();  
  Serial.println("Initializing...");
  Serial.println("Ready");
  lcd.print("Ready");
  delay(2000);


  // Initialize sensor
  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  //The LEDs are very low power and won't affect the temp reading much but
  //you may want to turn off the LEDs to avoid any local heating
  particleSensor.setup(0); //Configure sensor. Turn off LEDs
  //particleSensor.setup(); //Configure sensor. Use 25mA for LED drive

  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
}

void loop()
{
  float temperature = particleSensor.readTemperature();

  Serial.print("temperatureC=");
  Serial.print(temperature, 4);
  lcd.setCursor(0, 0);
  lcd.print("temp C=");
  lcd.print(temperature);

  float temperatureF = particleSensor.readTemperatureF(); //Because I am a bad global citizen

  Serial.print(" temperatureF=");
  Serial.print(temperatureF, 4);
  lcd.setCursor(0, 1);
  lcd.print("temp F=");
  lcd.print(temperatureF);


  Serial.println();
}
