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

const int SPEED_FORWARD = 60;
const int SPEED_BACKWARD = 60;
const int SPEED_TURN = 85;

bool anticipatingNewPath = false;

bool needsToTurn = false;
bool hasTurned = false;

void setup() {
  initLineFollower();
}

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

void loop() {
  moveProtocol();
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
  if (currentPos == POS_C && prevPos == POS_G) {
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
  if (currentPos == POS_G && prevPos == POS_F) {
    return;
  }
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
  if (currentPos == POS_F && prevPos == POS_E && !hasTurned) {
    fullStop();
    delay(3000);
    needsToTurn = true;
    return;
  }
  if (currentPos == POS_G && dispose_state == DS_END) {
    dispose_state = DS_NORMAL;
    hasTurned = false;
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
  delay(3000);
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
