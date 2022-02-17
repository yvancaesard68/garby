//Servo
#include <Servo.h> 
int servoPin1 = 44;
int servoPin2 = 46; 

Servo Servo1, Servo2;

//Linefollower
#include <SPI.h>
#include <MFRC522.h>

const int Rright_irsensor = 49;
const int right_irsensor = 47;
const int center_irsensor = 45;
const int left_irsensor = 43;
const int Lleft_irsensor = 41;

#define SS_PIN 53
#define RST_PIN 5
MFRC522 mfrc522(SS_PIN, RST_PIN);

const int enA = 3;
const int in1 = 2;
const int in2 = 4;
const int in3 = 8;
const int in4 = 7;
const int enB = 6;

const int buzzer = 13;

const String RFID_A = "8B 38 2B 25";
const String RFID_B = "9B B2 82 25";
const String RFID_C = "E9 AB 4B 98";
const String RFID_D = "2B 5C 84 25";
const String RFID_D2 = "EB BE 7A 25";
const String RFID_E = "FB 5A 7D 25";
const String RFID_F = "FB 1E 80 25";
const String RFID_G = "83 E8 70 1A";

const int POS_A = 1;
const int POS_B = 2;
const int POS_C = 3;
const int POS_D = 4;
const int POS_D2 = 8;
const int POS_E = 5;
const int POS_F = 6;
const int POS_G = 7;

int dispose_state = 0;
int DS_NORMAL = 0;
int DS_ALIGNED = 1;
int DS_BACK = 2;
int DS_PIVOTING = 3;
int DS_END = 4;

int currentPos = POS_G;
int prevPos = POS_F;

const int SPEED_FORWARD = 120;
const int SPEED_BACKWARD = 120;
const int SPEED_TURN = 170;

bool anticipatingNewPath = false;

bool needsToTurn = false;
bool hasTurned = false;

//Ultra LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int DISPOSE_GABAGE_THRESHOLD = 25;
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

bool override = false;

void setup() {
  // Line Follow
  // pinMode(OBSTACLE_ULTRASONIC_TRIGGER_PIN, OUTPUT);
  // pinMode(OBSTACLE_ULTRASONIC_ECHO_PIN, INPUT);

  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2); 

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

  initLineFollower();

  Serial.begin(9600);

  testThrowTrash();
}

void loop() {
  //continueMoving();
  eval();
}

void diagnose() {
  readAllSensorValues();
  showSensorValues();
  handleEsp();
}

bool waitingForLidToClose = false;
bool currentlyInTransit = false;

void eval() {

  handleEsp(); // <- esp should be readable always

  if (currentlyInTransit) {
    moveProtocol();
    return;
  }

  bool lid_is_closed = !digitalRead(LID_PIN);

  bool lid_is_open = !lid_is_closed;

  if (lid_is_closed) {

    if (waitingForLidToClose) {
      waitingForLidToClose = false;
      informLIDIsClosed();
      delay(1000);
    }

    readAllSensorValues();
    bool full_capacity = (bioPercentage >= DISPOSE_GABAGE_THRESHOLD) && (nonBioPercentage >= DISPOSE_GABAGE_THRESHOLD);

    showSensorValues();
        
    if (full_capacity && override) {
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
  int currentReading = map(distance1, 10, 2, 0, 100);
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
  int currentReading = map(distance2, 10, 2, 0, 100);
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

//=== === === ===
// ESP HANLDER
//=== === === ===

void handleEsp() {

  if (override && currentlyInTransit) {
    return;
  }
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
      override = true;
      delay(3000);
    }

//    if(doc["type"] == "switch_off_led") {
//      digitalWrite(led, LOW);
//    }
    
    messageReady = false;
  }
}

//=== === ===
// LINE FOLLOWER
//=== === ===

void initLineFollower() {
  SPI.begin();      // Initiate  SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522
  delay(4);       // Optional delay. Some board do need more time after init to be ready, see Readme
  mfrc522.PCD_DumpVersionToSerial();

  pinMode(Rright_irsensor, INPUT);
  pinMode(right_irsensor, INPUT);
  pinMode(center_irsensor, INPUT);
  pinMode(left_irsensor, INPUT);
  pinMode(Lleft_irsensor, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(buzzer, OUTPUT);
}

void moveProtocol() {

  if (needsToTurn) {
    pivotToRight();
    return;
  }

  scanRfid();


  if (dispose_state == DS_ALIGNED) {
    throwSequenceAlignStep();
    return;
  } else if (dispose_state == DS_BACK) {
    throwSequenceBackwards();
    return;
  } else if (dispose_state == DS_PIVOTING) {
    throwSequencePivot();
    return;
  }

  if (6 > 5) {
    leftTurnIfNeeded();
    rightTurnIfNeeded();
    forwardIfNeeded();
    stopIfNeeded();
  } else {
    stap();
  }

}

// === === === === === === === === === === === === === ===
// CHECK TURN CONDITIONS

void leftTurnIfNeeded() {
  if (pattern(0,1,1,1,1)) {
    left();
  } else if (pattern(1,0,1,1,1)) {
    left();
  } else if (pattern(0,0,1,1,1)) {
    left();
  } else if (pattern(0,0,0,1,1)) {
    left();
  } else if (pattern(1,0,0,1,1)) {
    left();
  } else if (pattern(0,0,0,0,1)) {
    left();
  }
}

void rightTurnIfNeeded() {
  if (pattern(1,1,1,1,0)) {
    right();
  } else if (pattern(1,1,1,0,1)) {
    right();
  } else if (pattern(1,1,1,0,0)) {
    right();
  } else if (pattern(1,1,0,0,0)) {
    right();
  } else if (pattern(1,1,0,0,1)) {
    right();
  } else if (pattern(1,0,0,0,0)) {
    right();
  }
}

void forwardIfNeeded() {
  if (pattern(1,1,0,1,1)) {
    forward();
  } else if (pattern(1,0,0,0,1)) {
    forward();
  }
}

void stopIfNeeded() {
 if (pattern(1,1,1,1,1)) {
    stap();
    recover();
  }
}

void recover() {
  if (currentPos == POS_C) {
    left();
  } else {
    right();
  }
}

// === === === === === === === === === === === === === ===
// GARBY SPECIFIC CODE:

void right() {
  if (currentPos == POS_C && prevPos == POS_B) {
    return;
  }
  if (currentPos == POS_E && prevPos == POS_D) {
    return;
  }
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, SPEED_TURN);
  analogWrite(enB, SPEED_TURN);
}

void left() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, SPEED_TURN);
  analogWrite(enB, SPEED_TURN);
}

void forward() {
  if (currentPos == POS_D2 && prevPos == POS_D && dispose_state == DS_NORMAL) {
    dispose_state = DS_ALIGNED;
    return;
  }
  if (currentPos == POS_A && dispose_state == DS_END) {
    dispose_state = DS_NORMAL;
    hasTurned = false;
    override = false;
    currentlyInTransit = false;
    stap();
    return;
  }
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, SPEED_FORWARD);
  analogWrite(enB, SPEED_FORWARD);
}

void backward() {
  if (currentPos == POS_D && prevPos == POS_D2) {
    fullStop();
    delay(3000);
    dispose_state = DS_PIVOTING;
    return;
  }
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, SPEED_BACKWARD);
  analogWrite(enB, SPEED_BACKWARD);
}

void stap() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// === === 
void pivotToRight() {
  right();

  if (pattern(1,1,1,1,1)) {
    anticipatingNewPath = true;
  }

  int C_S = digitalRead(center_irsensor);

  bool alignedToNewPath = C_S == LOW;
  if (alignedToNewPath && anticipatingNewPath) {
    fullStop();
    delay(3000);
    needsToTurn = false;
    anticipatingNewPath = false;
    hasTurned = true;
  }

}

// === === === === === === === === === === === === === ===

void scanRfid() {

  bool rfidIsPresent = mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial();
  bool rfidIsNotPresent = !rfidIsPresent;

  if (rfidIsNotPresent) {
    return;
  }

  String content = "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
    content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }

  content.toUpperCase();

  int tempPos = 0;

  if (content.substring(1) == RFID_A) {

    if (currentPos != POS_A) {
      prevPos = currentPos;
      currentPos = POS_A;
    }

  }

  if (content.substring(1) == RFID_B) {

    if (currentPos != POS_B) {
      prevPos = currentPos;
      currentPos = POS_B;
    }

  }

  if (content.substring(1) == RFID_C) {
    if (currentPos != POS_C) {
      prevPos = currentPos;
      currentPos = POS_C;
    }
  }

  if (content.substring(1) == RFID_D) {
    if (currentPos != POS_D) {
      prevPos = currentPos;
      currentPos = POS_D;
    }
  }

  if (content.substring(1) == RFID_D2) {
    if (currentPos != POS_D2) {
      prevPos = currentPos;
      currentPos = POS_D2;
    }
  }

  if (content.substring(1) == RFID_E) {
    if (currentPos != POS_E) {
      prevPos = currentPos;
      currentPos = POS_E;
    }
  }

  if (content.substring(1) == RFID_F) {
    if (currentPos != POS_F) {
      prevPos = currentPos;
      currentPos = POS_F;
    }
  }

  if (content.substring(1) == RFID_G) {
    if (currentPos != POS_G) {
      prevPos = currentPos;
      currentPos = POS_G;
    }
  }

}

// === === === === === === === === === === === === === ===
// THROW SEQUENCE

void fullStop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void throwSequenceAlignStep() {
  fullStop();
  throwSequenceDisposeStep();
  dispose_state = DS_BACK;
}

void throwSequenceDisposeStep() {
  throwTrash();
}

void throwTrash() {
   delay(2000);
   Servo1.write(10);
   delay(10000);
   Servo1.write(160);
   delay(2000);
   Servo2.write(180);  
   delay(10000);  
   Servo2.write(20);
   delay(2000); 
}

void testThrowTrash() {
   delay(2000);
   Servo1.write(10);
   delay(2000);
   Servo1.write(160);
   delay(2000);
   Servo2.write(180);  
   delay(2000);  
   Servo2.write(20);
   delay(2000); 
}

void throwSequenceBackwards() {
  backward();
}

void throwSequencePivot() {
  right();

  if (pattern(1,1,1,1,1)) {
    anticipatingNewPath = true;
  }

  int C_S = digitalRead(center_irsensor);
  bool alignedToNewPath = C_S == LOW;

  if (alignedToNewPath && anticipatingNewPath) {
    fullStop();
    delay(3000);
    dispose_state = DS_END;
    anticipatingNewPath = false;
  }
}

bool pattern(int LL, int L, int C, int R, int RR) {
  int LL_S = digitalRead(Lleft_irsensor);
  int L_S = digitalRead(left_irsensor);
  int C_S = digitalRead(center_irsensor);
  int R_S = digitalRead(right_irsensor);
  int RR_S = digitalRead(Rright_irsensor);
  return (RR == RR_S && R == R_S && C == C_S && L == L_S && LL == LL_S);
}
