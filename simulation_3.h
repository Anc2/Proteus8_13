// include the library code:
#include <LiquidCrystal.h>
#include "DHT.h"

//show where the actuator and sensors are connected
#define pirPin 8
#define pushButtonPin 9
#define relayPin 10
int fanSwitch = 11;
int fanControl = A0;
//show where the dht11 sensor was connected
#define DHTPIN 12

#define DHTTYPE DHT11  // we are using DHT11

DHT dht(DHTPIN, DHTTYPE);

// Variables will change:
int ledState = HIGH;        // the current state of the output pin
int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 10;    // the debounce time; increase if the output flickers

// state where the arduino pin number it is connected to on LCD
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  //begin the serial monitor comm.
  Serial.begin(9600);
  //beging the dht11 sensor
  dht.begin();
  //state the fxn for the inputs and outputs
  pinMode(pirPin, INPUT);
  pinMode(pushButtonPin, INPUT_PULLUP);
  pinMode(relayPin, OUTPUT);
  pinMode(fanSwitch, INPUT_PULLUP);
  pinMode(fanControl, OUTPUT);
  // Print a welcome message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("HELLO THERE?");
  delay(100);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MOTION BASED");
  lcd.setCursor(0, 0);
  lcd.print("HOME AUTOMATION");
  delay(100);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SIMULATION");
  delay(100);
  lcd.clear();
  //read the pir sensor
  while (digitalRead(pirPin) == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("   NO MOTION");
    lcd.setCursor(0, 1);
    lcd.print("DETECTED IN ROOM");
    Serial.println("NO MOTION DETECTED");
    delay(50);
    turnOffBulb();
    delay(100);
  }
  if (digitalRead(pirPin) == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MOTION DETECTED");
    Serial.println("MOTION DETECTED");
    delay(50);
    turnOnBulb();
    delay(100);
  }
}

void turnOnBulb() {
  digitalWrite(relayPin, HIGH);
  Serial.println("LIGHT BULB TURNED ON");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LIGHT BULB");
  lcd.setCursor(0, 1);
  lcd.print("TURNED ON");
}

void turnOffBulb() {
  digitalWrite(relayPin, LOW);
  Serial.println("LIGHT BULB TURNED OFF");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LIGHT BULB");
  lcd.setCursor(0, 1);
  lcd.print("TURNED OFF");
}

void controlFan() {

  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  //print out the readings
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.println(digitalRead(fanSwitch));

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T: " +String(t, 1)+ "'C H: " + String(h,0) + "%");    

  if (digitalRead(fanSwitch)== 1) {
    if (t > 37.00) {
      for (int i = 50; i < 256; i++) {
        analogWrite(fanControl, i);
        lcd.setCursor(0, 1);
        lcd.print("FAN ON"); 
      }
    }
      if (t < 35.00) {
        analogWrite(fanControl, LOW);
        lcd.setCursor(0, 1);
        lcd.print("FAN OFF"); 
      }    
  }

  if (digitalRead(fanSwitch) == 0) {
    analogWrite(fanControl, LOW);
    lcd.setCursor(0, 1);
    lcd.print("FAN OFF"); 
  }
}

void loop() {
  //read the pir sensor
  bool readPir = digitalRead(pirPin);
  // read the state of the switch into a local variable:
  int reading = digitalRead(pushButtonPin);
  controlFan();
  delay(100);
  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        ledState = !ledState;
      }
    }
  }

  //use an if condition to check for the motion
  if (ledState == 0) {
    if (readPir == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MOTION DETECTED");
      Serial.println("MOTION DETECTED pushbutton: " + String(reading) + " LED state: " + String(ledState));
      delay(50);
      turnOnBulb();
      delay(100);
    }
  }

  if (ledState == 1) {
    if (readPir == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MOTION DETECTED");
      lcd.setCursor(0, 1);
      lcd.print("USER OFF LIGHT");
      Serial.println("MOTION DETECTED pushbutton: " + String(reading) + " LED state: " + String(ledState));
      delay(50);
      turnOffBulb();
      delay(100);
    }

    if (readPir == 0) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("NO MOTION DETTED");
      lcd.setCursor(0, 1);
      lcd.print("USER OFF LIGHT");
      Serial.println("NO MOTION DETECTED pushbutton: " + String(reading) + " LED state: " + String(ledState));
      delay(50);
      turnOffBulb();
      delay(100);
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
  delay(100);
}
