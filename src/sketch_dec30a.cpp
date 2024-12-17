
#include <Mouse.h>
#include <Keyboard.h>
#include <JC_Button.h>  

constexpr int RXLED = 17;   // The RX LED has a defined Arduino pin
constexpr int PinSW = 15;   // Reading Push Button switch
constexpr int PinDT = 16;   // Reading DT signal
constexpr int PinCLK = 7;   // Generating interrupts using CLK signal
constexpr int PinBtn1 = 4;  // Push Buttons
constexpr int PinBtn2 = 5;
constexpr int PinBtn3 = 6;

constexpr int PinSegA = 14;
constexpr int PinSegB = 10;
constexpr int PinSegC = 8;
constexpr int PinSegD = A3;
constexpr int PinSegE = A2;
constexpr int PinSegF = A0;
constexpr int PinSegG = A1;
constexpr int PinSegDP = 9;

static constexpr int SegPins[] = {PinSegA, PinSegB, PinSegC, PinSegD, PinSegE, PinSegF, PinSegG, PinSegDP};
static constexpr int SegPinsSize = sizeof(SegPins) / sizeof(PinSegA);

ToggleButton btnSW(PinSW);

enum ePBtn { ePinBtn1, ePinBtn2, ePinBtn3, ePBtnEnd };
ToggleButton btnPB[ePBtnEnd] = {PinBtn1, PinBtn2, PinBtn3};

volatile boolean turnDetected;  // need volatile for Interrupts
volatile boolean rotationdirection;  // CW or CCW rotation

// 0-9, A-F.
uint8_t digitToSegment(const uint8_t digit) {
  constexpr const uint8_t digitSegments[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,
    0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71
  };
  return digitSegments[digit & 0x0F];
}

// Interrupt routine runs if CLK goes from HIGH to LOW
void Ext_INT1_ISR() {
  //delay(4);  // delay for Debouncing //delayMicroseconds
  if ( digitalRead(PinCLK) )
    rotationdirection = digitalRead(PinDT);
  else
    rotationdirection = !digitalRead(PinDT);
  turnDetected = true;
}

bool checkTime(const unsigned long &currentMillis, unsigned long &lastMillis, unsigned long wait, bool restart) {
  if ( currentMillis - lastMillis >= wait ) {
    if ( restart )
      lastMillis = currentMillis;  
    return true;
  }
  return false;
}

void buttonAction(const ToggleButton &button) {
  if ( button.changed() ) {
    Serial.print("button lastChanged:");
    Serial.println(button.lastChange());

    Serial.print("button toggle:");
    Serial.println(button.toggleState());
  }
}

void setup() {
  pinMode(RXLED, OUTPUT);   // Set RX LED as an output
  pinMode(PinCLK, INPUT);
  pinMode(PinDT, INPUT);  

  btnSW.begin();
  for ( auto &btn : btnPB )
    btn.begin();

  for ( const int &pin : SegPins )
    pinMode(pin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PinCLK), Ext_INT1_ISR, FALLING);

  Serial.begin(115200);     // This pipes to the serial monitor
  
  Keyboard.begin();
  Mouse.begin();
}

void loop() {
  const unsigned long currentMillis = millis();

  btnSW.read();
  for ( auto &btn : btnPB )
    btn.read();

  for ( auto &btn : btnPB )
    buttonAction(btn);

  // Heartbeat
  constexpr unsigned long heartFlashDelay  = 1000;
  static bool heartLEDToggle = false;
  static unsigned long heartBeatMillis = 0;
  if ( checkTime(currentMillis, heartBeatMillis, heartFlashDelay, true) ) {
    if ( (heartLEDToggle = !heartLEDToggle) )
      TXLED0;
    else
      TXLED1;
  }

  // Rotation
  constexpr unsigned MaxCount = 500;
  constexpr unsigned long rotateDelay  = 100;
  static unsigned long rotateMillis = 0; 
  static long rotateCount = 0;
  if ( checkTime(currentMillis, rotateMillis, rotateDelay, true) ) {
    if ( turnDetected ) {
      turnDetected = false;
      const long offset = 1 + 0.25 * rotateCount;
      if ( rotationdirection ) {
        rotateCount = min(rotateCount + offset, MaxCount);
      } else {
        rotateCount = max(rotateCount - offset, 1);
      }
      Serial.println(String(rotateCount).c_str());
    }
  }

  // Button SW
  if ( btnSW.changed() ) {
    Serial.print("btnSW toggle:");
    Serial.println(btnSW.toggleState());
    digitalWrite(RXLED, !btnSW.toggleState()); // 0 lights up LED
  }

  // Keyboard & Mouse
  if ( btnSW.toggleState() ) {
    // Different timer for Mouse / Keyboard to jitter order
    static unsigned long outMouseMillis = 0; 
    if ( checkTime(currentMillis, outMouseMillis, rotateCount * 10 + random(1, min(rotateCount*100, 2000)), true) ) {
      if ( btnPB[ePinBtn1].toggleState() ) 
        Mouse.click(MOUSE_LEFT);
    }

    static unsigned long outKeyMillis = 0; 
    if ( checkTime(currentMillis, outKeyMillis, rotateCount * 10 + random(1, min(rotateCount*100, 2000)), true) ) {
      if ( btnPB[ePinBtn2].toggleState() ) 
        Keyboard.write(' ');
    }
  }

  // Walk in a random direction. Allow key release after disable.
  static constexpr const char walkKeys[] = "wsad";
  static char pressedWalkKey = 0;
  static unsigned long outWalkMillis = 0; 
  if ( pressedWalkKey || btnSW.toggleState() ) {
    if ( checkTime(currentMillis, outWalkMillis, rotateCount * 10 + 10 * random(1, min(rotateCount*100, 2000)), true) ) {
      if ( pressedWalkKey ) {
        Keyboard.release(pressedWalkKey);
        pressedWalkKey = 0;
      } else {
        if ( btnPB[ePinBtn3].toggleState() ) {
          if ( (pressedWalkKey = walkKeys[random(0, strlen(walkKeys))]) )
            Keyboard.press(pressedWalkKey);
        }
      }
    }
  }

  static unsigned litSegPinIdx = 0;
  static unsigned long segLitMillis = 0;
  static unsigned long segLitDelay = 0;
  static bool segLit = false;
  if (checkTime(currentMillis, segLitMillis, segLitDelay, true)) {
    if ( segLit ) {
      segLit = false;
      segLitDelay = 1;
      digitalWrite(SegPins[litSegPinIdx], false);
    } else {
      if ( !btnSW.toggleState() ) {
        segLit = true;
        segLitDelay = 1;
        if ( ++litSegPinIdx >= SegPinsSize )
          litSegPinIdx = 0;
        const unsigned valueToWrite = ((float)rotateCount / MaxCount) * 0xF;
        if ( digitToSegment(valueToWrite) & (1 << litSegPinIdx) )
          digitalWrite(SegPins[litSegPinIdx], true);
      }
    }
  }


}



