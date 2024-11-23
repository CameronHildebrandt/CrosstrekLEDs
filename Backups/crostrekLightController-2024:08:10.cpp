#include <Adafruit_NeoPixel.h>

// GLOBALS //
// TODO: make #define (these are also globals.. y no capital camel?)
const int selectorUp = 3;
const int selectorDn = 4;
const int killUp = 2;
const int killDn = 5;
const int lightBarDetectorPin = A0;
const int bumperLightDetectorPin = A1;
const int dimmerPin = A2;
// #define STRIP_0_PIN 12
// #define STRIP_1_PIN 6
// #define STRIP_2_PIN 8
// #define STRIP_3_PIN 7
#define STRIP_0_PIN 12
#define STRIP_1_PIN 11
#define STRIP_2_PIN 10
#define STRIP_3_PIN 9
#define LED_LINE_LEN 50
#define LED_LINE_COUNT 4
#define LED_COUNT LED_LINE_COUNT*LED_LINE_LEN
const uint8_t ConnectedLEDs[LED_LINE_COUNT] = {44, 12, 7, 0};
const uint8_t WhitePixels[4] = {50, 51, 57, 58};

int KillSwitchState;
bool IsLightBarOn = false;
bool IsBumperLightOn = false;


// 0 = No Light, 1 = Christmas Light, 2 = SMD LED, 3 = Light Strip // TODO: fix array indexing, single dimensional arr??
const uint8_t pixelType[LED_LINE_COUNT][LED_LINE_LEN] = {
  {2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3}, // Strip 0: Buttons + Driver's Side Doors
  {2,2,2,2,2,2,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2}, // Strip 1: Dash + Steering Wheel
  {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3}, // Strip 2: Footwells + Heated Seat
  {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3}, // Strip 3: Passenger's Footwells, Radio, Passenger's Side Doors
};

// 0 = Off, 1 = White, 2 = Red, 3 = Blue
const uint8_t pixelOEMColour[LED_COUNT] = {
  2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Strip 0: Buttons + Driver's Side Doors
  1,1,2,2,2,3,3,1,1,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Strip 1: Dash + Steering Wheel
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Strip 2: Footwells + Heated Seat
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Strip 3: Passenger's Footwells, Radio, Passenger's Side Doors
};

// 0-255
uint8_t pixelBrightness[LED_COUNT] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Strip 0: Buttons + Driver's Side Doors
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Strip 1: Dash + Steering Wheel
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Strip 2: Footwells + Heated Seat
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Strip 3: Passenger's Footwells, Radio, Passenger's Side Doors
};


Adafruit_NeoPixel strip0(LED_LINE_LEN, STRIP_0_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip1(LED_LINE_LEN, STRIP_1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED_LINE_LEN, STRIP_2_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3(LED_LINE_LEN, STRIP_3_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel Strips[LED_LINE_COUNT] = {strip0, strip1, strip2, strip3};
// GLOBALS //


// FORWARD DECLARATIONS //
void selectorDnFunc();
void selectorUpFunc();
void lightBarCallback(bool);
void bumperLightCallback(bool);
// FORWARD DECLARATIONS //


// CLASSES //
class Switch {
  private:
    unsigned long lastInputMillis;
    unsigned long inputSensitivity = 50;
    int up, dn;
    bool isMomentary;
    int switchState;


    void handleInputMomentary() {
      // Ensures single button press => single input
      if(digitalRead(dn)) {
        if(millis() - lastInputMillis > inputSensitivity) {
           selectorDnFunc();
        }
        lastInputMillis = millis();
      }

      if(digitalRead(up)) {
        if(millis() - lastInputMillis > inputSensitivity) {
           selectorUpFunc();
        }
        lastInputMillis = millis();
      }
    }

    void handleInputStatic() {
      if(digitalRead(dn)) {
        switchState = 1;
        KillSwitchState = 1;
        return;
      }

      if(digitalRead(up)) {
        switchState = 2;
        KillSwitchState = 2;
        return;
      }

      switchState = 0;
      KillSwitchState = 0;
    }

  public:
    Switch(int u, int d, bool m) {
      up = u;
      dn = d;
      isMomentary = m;
    }

    void handleInput() {
      if(isMomentary) { handleInputMomentary(); }
      else { handleInputStatic(); }
    }

    int state() {
      if(isMomentary) { return -1; }
      return switchState;
    }
};


class Detector {
  private:
    int detectorPin;
    int defaultState;
    int detectorType;

  public:
    Detector(int dp, bool ds, int dt) {
      detectorPin = dp;
      defaultState = ds;
      detectorType = dt;
    }

    void handleState() {
      switch (detectorType) {
        case 0:
          lightBarCallback(digitalRead(detectorPin) != defaultState);
          break;
        case 1:
          bumperLightCallback(digitalRead(detectorPin) != defaultState);
          break;
      }
    }
};


class Dimmer {
  private:
    int dimmerPin;

  public:
    Dimmer(int dp) {
      dimmerPin = dp;
    }

    void handleState() {
      // Exponential mapping of brightness values
      // dimmerPin should read 0-6 (0 brightest). This function converts that to 255-1 (255 brightest)
      int brightness = 510/(1+exp((analogRead(dimmerPin)+125)/175));

      for(int i = 0; i < LED_LINE_COUNT; i++) {
        Strips[i].setBrightness(brightness);
      }
    }
};


class LED {
  private:
    int colour = 0;
    int Red;
    int Green;
    int Blue;

    int style = 0;
    int numStyles = 7;

    unsigned long speed = 10;
    unsigned long lastUpdateMillis;

    // TEMP used to track where we are in the flash
    bool notYetImplementedFlash = true;


    // HELPER FUNCTIONS //
    void generateRGBFromCurrentState() {
      if(colour >= 0 && colour <= 255) {
        Red = 255;
        Green = 0;
        Blue = colour;
      }

      else if(colour > 255 && colour <= 510) {
        Red = 510-colour;
        Green = 0;
        Blue = 255;
      }

      else if(colour > 510 && colour <= 765) {
        Red = 0;
        Green = colour-510;
        Blue = 255;
      }

      else if(colour > 765 && colour <= 1020) {
        Red = 0;
        Green = 255;
        Blue = 1020-colour;
      }

      else if(colour > 1020 && colour <= 1275) {
        Red = colour-1020;
        Green = 255;
        Blue = 0;
      }

      else if(colour > 1275 && colour <= 1530) {
        Red = 255;
        Green = 1530-colour;
        Blue = 0;
      }
    }

    void setSpeedometerWhitePixels() {
      for(int pixel=0; pixel<4; pixel++) {
        set(WhitePixels[pixel], 255, 255, 255);
      }
    }

    uint8_t convertConnIdxToRealIdx(uint8_t connIdx) {
      uint8_t strip = 0;
      uint8_t stripLed = connIdx;
      for(int i; i<LED_LINE_COUNT; i++) {
        if(stripLed - ConnectedLEDs[i] >= 0) {
          strip++;
          stripLed -= ConnectedLEDs[i];
        } else {
          break;
        }
      }

      return strip*LED_LINE_LEN + stripLed;
    }

    bool isReadyToUpdate() {
      return millis() - lastUpdateMillis > speed;
    }
    // HELPER FUNCTIONS //

    // STYLE FUNCTIONS //
    void cycleWhite() {
      if(isReadyToUpdate()) {
        colour++;
        if(colour >= 1530) {
          colour = 0;
        }

        generateRGBFromCurrentState();
        setAllFromCurrentState();
        setSpeedometerWhitePixels();

        show();

        lastUpdateMillis = millis();
      }
    }

    void cycle() {
      if(isReadyToUpdate()) {
        colour++;
        if(colour >= 1530) {
          colour = 0;
        }

        generateRGBFromCurrentState();
        setAllFromCurrentState();
        show();

        lastUpdateMillis = millis();
      }
    }

    void sparkle() {
      setAll(0, 255, 0);
      show();
    }

    void matrix() {
      if(lastUpdateMillis + speed * 25 < millis()) {
        if(notYetImplementedFlash) {
          setAll(0, 255, 0);
          show();
          notYetImplementedFlash = false;
        } else {
          setAll(0, 100, 0);
          show();
          notYetImplementedFlash = true;
        }
        lastUpdateMillis = millis();
      }
    }

    void wave() {
      if(lastUpdateMillis + speed * 25 < millis()) {
        if(notYetImplementedFlash) {
          setAll(0, 0, 255);
          show();
          notYetImplementedFlash = false;
        } else {
          setAll(0, 0, 100);
          show();
          notYetImplementedFlash = true;
        }
        lastUpdateMillis = millis();
      }
    }

    void music() {
      if(lastUpdateMillis + speed * 25 < millis()) {
        if(notYetImplementedFlash) {
          setAll(255, 0, 175);
          show();
          notYetImplementedFlash = false;
        } else {
          setAll(100, 0, 68);
          show();
          notYetImplementedFlash = true;
        }
        lastUpdateMillis = millis();
      }
    }

    void red() {
      setAll(255, 0, 0);
      show();
    }
    // STYLE FUNCTIONS //


  public:
    void set(int i, int r, int g, int b) {
      int stripNum = floor(i / LED_LINE_LEN);
      int ledIndex = i % LED_LINE_LEN;


      // Dash Lights //
      bool isLightBarDashLight = (stripNum == 1 && ledIndex == 5);
      bool isBumperLightDashLight = (stripNum == 1 && ledIndex == 6);

      if(isLightBarDashLight) {
        if(IsLightBarOn) {
          r = r*0.25; g = g*0.25; b = b*0.25;
        } else {
          r = 0; g = 0; b = 0;
        }
      }
      else if(isBumperLightDashLight) {
        if(IsBumperLightOn) {
          r = r; g = g; b = b;
        } else {
          r = 0; g = 0; b = 0;
        }
      }
      // Dash Lights //


      // Set the Light //
      switch (pixelType[stripNum][ledIndex]) {
        case 0: break;
        case 1: Strips[stripNum].setPixelColor(ledIndex, Strips[stripNum].Color(g, r, b)); break;
        case 2: Strips[stripNum].setPixelColor(ledIndex, Strips[stripNum].Color(r, g, b)); break;
        case 3: Strips[stripNum].setPixelColor(ledIndex, Strips[stripNum].Color(r, b, g)); break;
      }
      // Set the Light //
    }

    void setAll(int r, int g, int b) {
      for(int i = 0; i < LED_COUNT; i++) {
        set(i, r, g, b);
      }
    }

    void setOEM() {
      for(int i = 0; i < LED_COUNT; i++) {
        switch(pixelOEMColour[i]) {
          case 0: set(i, 0, 0, 0);       break;
          case 1: set(i, 255, 255, 255); break;
          case 2: set(i, 255, 0, 0);     break;
          case 3: set(i, 0, 0, 255);     break;
        }
      }
    }

    void setAllFromCurrentState() {
      for(int i = 0; i < LED_COUNT; i++) {
        set(i, Red, Green, Blue);
      }
    }

    void show() {
      for(int i = 0; i < LED_LINE_COUNT; i++) {
        Strips[i].show();
      }
    }

    void update() {
      // Kill Switch //
      switch(KillSwitchState) {
        case 0: break; // Default - Run normal updates
        case 1: setOEM(); show(); return; // OEM Override - Don't run normal updates
        case 2: setAll(0, 0, 0); show(); return; // OFF Override - Don't run normal updates
      }
      // Kill Switch //


      // Update Using Respective Style Function //
      switch(style) {
        case 0: cycleWhite(); break;
        case 1: cycle(); break;
        case 2: sparkle(); break;
        case 3: matrix(); break;
        case 4: wave(); break;
        case 5: music(); break;
        case 6: red(); break;
        default: style = 0; break;
      }
      // Update Using Respective Style Function //
    }


    // TODO: read brightness dial (and light bar dash lights?) while starting up

    // void startupAnimation() {
    void startupAnimation(class killSwitch, class dimmer, class lightBarDetector, class bumperLightDetector) {
      uint8_t connectedLEDCount = 0;
      for(int i=0; i<LED_LINE_COUNT; i++) { connectedLEDCount += ConnectedLEDs[i]; }

      uint8_t startupAnimationPixelsLen = connectedLEDCount;
      uint8_t startupAnimationPixels[startupAnimationPixelsLen];
      for(int i=0; i<startupAnimationPixelsLen; i++) { startupAnimationPixels[i] = i; }

      uint8_t pixelStartFreq = 5; // Start new pixel every n updates
      uint8_t concurrentPixelStarts = 1; // Start n pixels per update
      uint8_t updateStep = 10; // Increase pixel brightness by n every update
      uint8_t maxBrightness = 255;
      uint8_t numUpdates = ceil((connectedLEDCount*pixelStartFreq)/concurrentPixelStarts) + ceil(maxBrightness/updateStep); // Time to start all pixels + complete the final pixel


      // Handle State - TODO: do smart things with this info - choose startup animation based on killswitch state, etc.
      dimmer.handleState();
      killSwitch.handleInput();
      lightBarDetector.handleState();
      bumperLightDetector.handleState();



      // Start fading random pixels to white
      for(int i=0; i<numUpdates; i++) {
        // TODO: group pixels together? (gauge dial 3 pixels == one pixel?)


        // Select next pixel to start (if there are pixels left to start)
        if(i % pixelStartFreq == 0 && ceil((connectedLEDCount*pixelStartFreq)/concurrentPixelStarts) > i && startupAnimationPixelsLen > 0) {
          for(int j=0; j<concurrentPixelStarts; j++) {
            // Select random pixel
            uint8_t pick = random(0, startupAnimationPixelsLen);
            uint8_t randomlySelectedLED = convertConnIdxToRealIdx(startupAnimationPixels[pick]);
            startupAnimationPixels[pick] = startupAnimationPixels[startupAnimationPixelsLen - 1]; // Rearrange array for efficient picking
            startupAnimationPixelsLen--;

            // Start selected pixel
            pixelBrightness[randomlySelectedLED] = updateStep;
          }
        } else { // When skipping the above code, simulate its computation time for a consistent animation speed
          delay(10); // TODO: NEEDS TUNING
        }


        // Update all pixels that are not off or max
        for(int j=0; j<connectedLEDCount; j++) {
          int realIdx = convertConnIdxToRealIdx(j);
          if(pixelBrightness[realIdx] != 0 && pixelBrightness[realIdx] < maxBrightness) {
            pixelBrightness[realIdx] = min(pixelBrightness[realIdx] + updateStep, maxBrightness);
            set(realIdx, pixelBrightness[realIdx], pixelBrightness[realIdx], pixelBrightness[realIdx]);
          }
        }

        show();
      }
      

      // TODO: write other fade functions for kill switch state (OEM mode => fade footwells off)
      // Start fading to starting colors
      for(int i=255; i>=0; i--) {
        setAll(255, i, i); // Not very flexible, but simple. Take 255 steps and reduce brightness by 1 each step to fade to red
        setSpeedometerWhitePixels(); // Keep the white pixels white
        show();
      }
    }


    // shutdownAnimation() {
    //     set(i, r, g, b);
    //     set(i, r, g, b);
    //     set(i, r, g, b);
    //     set(i, r, g, b);
    //     set(i, r, g, b);
    // }


    void changeStyleUp() {
      style++;
      if(style >= numStyles) {
        style = 0;
      }
    }

    void changeStyleDn() {
      style--;
      if(style < 0) {
        style = numStyles - 1;
      }
    }
};
// CLASSES //



// GLOBALS //
Switch selectorSwitch = Switch(selectorUp, selectorDn, true);
Switch killSwitch = Switch(killUp, killDn, false);
Detector lightBarDetector = Detector(lightBarDetectorPin, false, 0);
Detector bumperLightDetector = Detector(bumperLightDetectorPin, false, 1);
Dimmer dimmer = Dimmer(dimmerPin);
LED led = LED();
// GLOBALS //



// FUNCTIONS //
void selectorDnFunc() {
  led.changeStyleDn();
}

void selectorUpFunc() {
  led.changeStyleUp();
}

void lightBarCallback(bool detected) {
  IsLightBarOn = detected;
}

void bumperLightCallback(bool detected) {
  IsBumperLightOn = detected;
}
// FUNCTIONS //



// ARDUINO //
void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A3));

  // Init Detectors
  lightBarCallback(false);
  bumperLightCallback(false);

  // Init Strips
  for(int i = 0; i < LED_LINE_COUNT; i++) {
    Strips[i].begin();
    Strips[i].clear();
  }

  // Init Pins
  pinMode(selectorUp, INPUT);
  pinMode(selectorDn, INPUT);
  pinMode(STRIP_0_PIN, OUTPUT);
  pinMode(STRIP_1_PIN, OUTPUT);
  pinMode(STRIP_2_PIN, OUTPUT);
  pinMode(STRIP_3_PIN, OUTPUT);

  // Run Startup Animation
  // dimmer.handleState();
  // led.startupAnimation();


  // Test to handle all stateful classes in the startup animation?
  led.startupAnimation(killSwitch, dimmer, lightBarDetector, bumperLightDetector);
}

void loop() {
  dimmer.handleState();
  killSwitch.handleInput();
  selectorSwitch.handleInput();
  lightBarDetector.handleState();
  bumperLightDetector.handleState();
  led.update();
}
// ARDUINO //

