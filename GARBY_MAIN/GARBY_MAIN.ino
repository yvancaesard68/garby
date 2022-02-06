//Linefollower
#define OBSTACLE_ULTRASONIC_TRIGGER_PIN 22
#define OBSTACLE_ULTRASONIC_ECHO_PIN 23

const int Rright_irsensor = 49;
const int right_irsensor = 47;
const int center_irsensor = 45;
const int left_irsensor = 43;
const int Lleft_irsensor = 41;

const int enA = 3;
const int in1 = 2;
const int in2 = 4;
const int in3 = 8;
const int in4 = 7;
const int enB = 6;

const int M1_Speed = 50; // speed of motor 1
const int M2_Speed = 50; // speed of motor 2
const int LeftRotationSpeed = 80;  // Left Rotation Speed
const int RightRotationSpeed = 80; // Right Rotation Speed

//Ultra LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int DISPOSE_GABAGE_THRESHOLD = 70;

const int BIO_ULTRASONIC_TRIGGER_PIN = 26;
const int BIO_ULTRASONIC_ECHO_PIN = 27;

const int NON_BIO_ULTRASONIC_TRIGGER_PIN = 30;
const int NON_BIO_UTRASONIC_ECHO_PIN = 31;

// Printed in the LCD;

const int BIO_READINGS_MAX = 10;
int bioReadingsCount = 0;
int bioPercentageTotal = 0;
int bioPercentage = 0;

const int NON_BIO_READINGS_MAX = 10;
int nonBioReadingsCount = 0;
int nonBioPercentageTotal = 0;
int nonBioPercentage = 0;

#define LID_PIN A5
int lidState = 0;

#define Vsensor A2
float Vout = 0.0;
int Vin = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;

// ESP
#include <ArduinoJson.h>
String message = "";
bool messageReady = false;

void setup() {
  // Line Follow
  pinMode(OBSTACLE_ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(OBSTACLE_ULTRASONIC_ECHO_PIN, INPUT);

  pinMode(Rright_irsensor, INPUT);
  pinMode(right_irsensor, INPUT);
  pinMode(center_irsensor, INPUT);
  pinMode(left_irsensor, INPUT);
  pinMode(Lleft_irsensor, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  // LCD
  lcd.begin();
  lcd.backlight();

  // Lid
  pinMode(LID_PIN, INPUT_PULLUP);

  // Ultrasonic for Bins
  pinMode(BIO_ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(BIO_ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(NON_BIO_ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(NON_BIO_UTRASONIC_ECHO_PIN, INPUT);

  Serial.begin(9600);
}

void loop() {
  continueMoving();
  //eval();
}

void diagnose() {
  readAllSensorValues();
  showSensorValues();
  handleEsp();
}

bool waitingForLidToClose = false;
bool currentlyInTransit = false;

void eval() {

  bool lid_is_closed = !digitalRead(LID_PIN);

  bool lid_is_open = !lid_is_closed;

  if (lid_is_closed) {

    if (waitingForLidToClose) {
      waitingForLidToClose = false;
      informLIDIsClosed();
      delay(1000);
    }

    bool full_capacity = (bioPercentage >= DISPOSE_GABAGE_THRESHOLD) && (nonBioPercentage >= DISPOSE_GABAGE_THRESHOLD);
    bool not_full_capacity = !full_capacity;
    readAllSensorValues();

    if (not_full_capacity) {
      showSensorValues();
      handleEsp();
    }

    if (full_capacity) {
      informPrepping(3);
      delay(1000);
      informPrepping(2);
      delay(1000);
      informPrepping(1);
      delay(1000);
      informMoving();
      delay(1000);
      currentlyInTransit = true;
      // wait for a while
      // initiate throwing of trash
    }

  }

  // don't read sensor value
  if (lid_is_open) {
    // inform that lid is open
    // anticipate closing of lid
    // no need to handleEsp()

    waitingForLidToClose = true;
    informLIDIsOpen();
  }

}

void informLIDIsClosed() {
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Lid closed");
  lcd.setCursor(3, 1);
  lcd.print("Thank You!");
}

void informLIDIsOpen() {
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Pls Close");
}

void informPrepping(int countdown) {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Preparing: ");
  lcd.print(countdown);
}

void informMoving() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Moving");
}

void readAllSensorValues() {
  readBioSensor();
  readNonBioSensor();
  readBatterySensor();
}

void readBioSensor() {
  digitalWrite(BIO_ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(BIO_ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(BIO_ULTRASONIC_TRIGGER_PIN, LOW);
  long timedelay = pulseIn(BIO_ULTRASONIC_ECHO_PIN, HIGH);
  long distance1 = 0.0343 * (timedelay / 2);
  int currentReading = map(distance1, 30, 2, 0, 100);
  int newPercentage = constrain(currentReading, 0, 100);
  bioPercentageTotal += newPercentage;
  bioReadingsCount += 1;
  bioPercentage = bioPercentageTotal / bioReadingsCount;
  if (bioReadingsCount > BIO_READINGS_MAX) {
    bioPercentageTotal = bioPercentage;
    bioReadingsCount = 1;
  }
}

void readNonBioSensor() {
  digitalWrite(NON_BIO_ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(NON_BIO_ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(NON_BIO_ULTRASONIC_TRIGGER_PIN, LOW);
  long td = pulseIn(NON_BIO_UTRASONIC_ECHO_PIN, HIGH);
  long distance2 = 0.0343 * (td / 2);
  int currentReading = map(distance2, 30, 2, 0, 100);
  int newPercentage = constrain(currentReading, 0, 100);
  nonBioPercentageTotal += newPercentage;
  nonBioReadingsCount += 1;
  nonBioPercentage = nonBioPercentageTotal / nonBioReadingsCount;
  if (nonBioReadingsCount > NON_BIO_READINGS_MAX) {
    nonBioPercentageTotal = nonBioPercentage;
    nonBioReadingsCount = 1;
  }
}

void readBatterySensor() {
  int voltval = analogRead(Vsensor);
  Vout = (voltval * 5.0) / 1024.0;
  Vin = Vout / (R2 / (R1 + R2));
}


void showSensorValues() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bio:");
  lcd.setCursor(4, 0);
  lcd.print(bioPercentage);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("NonBio:");
  lcd.setCursor(7, 1);
  lcd.print(nonBioPercentage);
  lcd.print("%");
  lcd.setCursor(8, 0);
  lcd.print("Batt:");
  lcd.print(Vin);
  lcd.print("v");
}

void continueMoving() {

//  digitalWrite(OBSTACLE_ULTRASONIC_TRIGGER_PIN, LOW);
//  delayMicroseconds(2);
//  digitalWrite(OBSTACLE_ULTRASONIC_TRIGGER_PIN, HIGH);
//  delayMicroseconds(10);
  digitalWrite(OBSTACLE_ULTRASONIC_TRIGGER_PIN, LOW);
  long duration = pulseIn(OBSTACLE_ULTRASONIC_ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;

  int Rright_sensor = digitalRead(Rright_irsensor);
  int right_sensor = digitalRead(right_irsensor);
  int center_sensor = digitalRead(center_irsensor);
  int left_sensor = digitalRead(left_irsensor);
  int Lleft_sensor = digitalRead(Lleft_irsensor);

  if (6 > 5) { //If the reading is more than 5 cm, then the robot can run.

    if (Rright_sensor == HIGH && right_sensor == HIGH && center_sensor == HIGH && left_sensor == HIGH && Lleft_sensor == LOW) {
      left();
    }
    if (Rright_sensor == HIGH && right_sensor == HIGH && center_sensor == HIGH && left_sensor == LOW && Lleft_sensor == HIGH) {
      left();
    }
    if (Rright_sensor == HIGH && right_sensor == HIGH && center_sensor == HIGH && left_sensor == LOW && Lleft_sensor == LOW) {
      left();
    }
    if (Rright_sensor == HIGH && right_sensor == HIGH && center_sensor == LOW && left_sensor == LOW && Lleft_sensor == LOW) {
      left();
    }
    if (Rright_sensor == HIGH && right_sensor == HIGH && center_sensor == LOW && left_sensor == LOW && Lleft_sensor == HIGH) {
      left();
    }


    if (Rright_sensor == LOW && right_sensor == HIGH && center_sensor == HIGH && left_sensor == HIGH && Lleft_sensor == HIGH) {
      right();
    }
    if (Rright_sensor == HIGH && right_sensor == LOW && center_sensor == HIGH && left_sensor == HIGH && Lleft_sensor == HIGH) {
      right();
    }
    if (Rright_sensor == LOW && right_sensor == LOW && center_sensor == HIGH && left_sensor == HIGH && Lleft_sensor == HIGH) {
      right();
    }
    if (Rright_sensor == LOW && right_sensor == LOW && center_sensor == LOW && left_sensor == HIGH && Lleft_sensor == HIGH) {
      right();
    }
    if (Rright_sensor == HIGH && right_sensor == LOW && center_sensor == LOW && left_sensor == HIGH && Lleft_sensor == HIGH) {
      right();
    }

    if (Rright_sensor == LOW && right_sensor == LOW && center_sensor == LOW && left_sensor == LOW && Lleft_sensor == LOW) {
      stap();
    }
    if (Rright_sensor == HIGH && right_sensor == HIGH && center_sensor == HIGH && left_sensor == HIGH && Lleft_sensor == HIGH) {
      stap();
    }
    if (Rright_sensor == HIGH && right_sensor == HIGH && center_sensor == LOW && left_sensor == HIGH && Lleft_sensor == HIGH) {
      forward();
    }
  }
  else {
    stap();
  }
}

void right() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}

void left() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}

void forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);

}

void stap() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void backward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}

void handleEsp() {
    // Monitor serial communication
  while(Serial.available()) {
    message = Serial.readString();
    messageReady = true;
  }
   
  // Only process message if there's one
  if(messageReady) {
    // The only messages we'll parse will be formatted in JSON
    DynamicJsonDocument doc(1024); // ArduinoJson version 6+
    // Attempt to deserialize the message
    DeserializationError error = deserializeJson(doc,message);

    //String ip = doc["ip"];

    //Serial.print(ip);
    if(error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      messageReady = false;
      return;
    }
    if(doc["type"] == "request") {
      doc["type"] = "response";
      // Get data from analog sensors
      
      doc["Bio"] = bioPercentage;
      doc["NonBio"] = nonBioPercentage;
      doc["batt"] = Vin;
     // doc["switch_on_led"] == digitalWrite(led,HIGH);
      //doc["switch_off_led"] == digitalWrite(led,LOW);
      serializeJson(doc,Serial);
    }

    if(doc["type"] == "switch_on_led") {
      informMoving();
      delay(3000);
    }

//    if(doc["type"] == "switch_off_led") {
//      digitalWrite(led, LOW);
//    }
    
    messageReady = false;
  }
}
