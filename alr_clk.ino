#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); //Instance of Accelorometer
const int segmentPins[7] = {2, 3, 4, 5, 6, 7, 8};  // A, B, C, D, E, F, G
const int displayPins[4] = {12, 11, 10, 9};  // Control pins for the four displays
const int buttonPin = A2; //Changes from minutes to hours
const int switchPin = A1; //Changes to and from edit mode
const int dialPin = A7; //Dial that allows 0-23 or 0-59 based on A1 pin
const int relayPin = A3; //Controls relay for light
const int alarmPin = A0; //Switch that control if we are editing alarm time


//Struct for storing time data
struct Time {
  int hours;
  int minutes;
  int seconds;
  int milliseconds;
};

//Init varibles
bool alarmMode;
bool timeChangingMode;
bool hourChangingMode;
bool lastButtonState;
bool buttonState;
bool switchState;

bool motionDetected;
double lastPos;

bool alarmOn = false;

int secondCounter = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  
struct Time currentTime = {19,24,0,0};
struct Time alarmTime = {20,0,0,0};
struct Time *timePtr = &currentTime;
struct Time *alarmPtr = &alarmTime;
int refreshRate = 5;

          

const byte digitPatterns[10] = {
  0b00111111,  // 0
  0b00000110,  // 1
  0b01011011,  // 2
  0b01001111,  // 3
  0b01100110,  // 4
  0b01101101,  // 5
  0b01111101,  // 6
  0b00000111,  // 7
  0b01111111,  // 8
  0b01101111   // 9
};

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
  }
  
  for (int i = 0; i < 4; i++) {
    pinMode(displayPins[i], OUTPUT);
  }

  pinMode(buttonPin, INPUT);
  pinMode(dialPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(alarmPin, INPUT_PULLUP);
  accel.begin();
  if (!accel.begin()) {
    Serial.println("No ADXL345 detected. Check your wiring!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_2_G);
  
}



void loop() {
  unsigned long startTime = millis(); //Start of loop timer
  
  accelSensor(timePtr);

  GetAlarmMode(); //gets the state of the switch

  //If we are in alarm mode. We will show the alarm time, and be able to modify it.
  //Else,do the same things but showing the current time.

  if(alarmMode) {
    timeChange(alarmPtr); //Handle when A2 is flipped. User changes current time.
    displayTime(alarmTime, 100); //output to display, linger for 5 seconds;
  }
  else {
    timeChange(timePtr); //Handle when A2 is flipped. User changes current time.
    displayTime(currentTime, 100); //output to display, linger for 5 seconds;
  }
  

  alarm(timePtr, alarmPtr); //Check if alarm is triggered, and handle alarm functionality
  unsigned long elapsedMillis = (millis() - startTime); //get the time for this iteration
  startTime = millis(); //set the start time for the next iteration to be the end of this iteration
  updateTime(timePtr, elapsedMillis); //update time for the elapsed period (end - start)


}

void accelSensor(struct Time *t) {
  if (t->seconds - secondCounter == 5 && !alarmOn) {
    sensors_event_t event;
    accel.getEvent(&event);
    double total = fabs(event.acceleration.x) + fabs(event.acceleration.y) + fabs(event.acceleration.z);
    // Print the X, Y, Z axis values
    Serial.print("Total "); Serial.println(total);
    delay(1);
    secondCounter = t->seconds;

    if(fabs(total) - fabs(lastPos) > 0.2) {
      motionDetected = true;
      Serial.println("Motion Detected");
    }
    else {
      motionDetected = false;
    }
    lastPos = total;
  }
  if (secondCounter ==  55) {
    secondCounter = 0;
  }
  
}
void GetAlarmMode() {
  int alarmState = digitalRead(alarmPin);
  
  if(alarmState == HIGH) {
    alarmMode = true;
  }
  else {
    alarmMode = false;
  }

}

void timeChange(struct Time *t ) {
  int reading = digitalRead(buttonPin);
  int switchD = digitalRead(switchPin);


  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
        buttonState = reading;
        if (buttonState == HIGH) {
          hourChangingMode = !hourChangingMode;
        }
    }
  }
  lastButtonState = reading;

  if (switchD == HIGH) {
    timeChangingMode = true;
  }
  else {
    timeChangingMode = false;
  }

  if (timeChangingMode) {
    int potValue = analogRead(dialPin);
    int timeValue;
    if(hourChangingMode) {
      delay(20);
      timeValue = map(potValue, 0, 1023, 0, 23);
      t->hours = timeValue;
    }
    else {
      delay(20);
      timeValue = map(potValue, 0, 1023, 0, 59);
      t->minutes = timeValue;
    }
    
    Serial.println(potValue);
  }

}

void alarm(struct Time *t, struct Time *a) {
  if (t->hours + 1 == a->hours && motionDetected) {
    alarmOn = true;
    digitalWrite(relayPin, HIGH);
  }
  else if (t->hours == a->hours) {
    digitalWrite(relayPin, HIGH);

  }
  else if (alarmOn && ( t->hours + 1 != a->hours || t->hours != a->hours)) {
    digitalWrite(relayPin, LOW);
    alarmOn = false;

  }
  else {
    digitalWrite(relayPin,LOW);

  }
}


void displayTime(struct Time t, int countSpeed) {
  int minutes= t.minutes;
  int hours = t.hours;
  
  // Extract each digit, 0 is the left most digit. I.E. it is the tens place for hours in a 24 hour clock
  int digits[4];
  digits[0] = (hours / 10) % 10;          
  digits[1] = hours % 10;     
  digits[2] = (minutes / 10) % 10;      
  digits[3] = minutes % 10;             
  
  unsigned long startTime = millis();

  while (millis() - startTime < countSpeed) {
  for (int i = 0; i < 4; i++) {
    // Turn off all displays
    for (int j = 0; j < 4; j++) {
      digitalWrite(displayPins[j], HIGH);
    }
    
    // Set the active display
    digitalWrite(displayPins[i], LOW);
    
    // Display the digit
    setSegments(digits[i]);
    delay(5);

  }
  }
}


void setSegments(int number) {
  byte pattern = digitPatterns[number];
  for (int i = 0; i < 7; i++) {
    digitalWrite(segmentPins[i], (pattern >> i) & 0x01);
  }
}



void updateTime(struct Time *t, unsigned long elapsedMillis) {
  t->milliseconds += elapsedMillis;
  while (t->milliseconds >= 1000) {
  

      t->milliseconds -= 1000;
      t->seconds++;
      if (t->seconds >= 60) {
          t->seconds = 0;
          t->minutes++;
          if (t->minutes >= 60) {
              t->minutes = 0;
              t->hours++;
              if (t->hours >= 24) {
                  t->hours = 0;
              }
          }
      }
  }
}


