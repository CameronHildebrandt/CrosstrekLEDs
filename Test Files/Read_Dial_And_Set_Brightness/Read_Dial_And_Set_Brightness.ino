// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        3 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 3 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 100 // Time (in milliseconds) to pause between pixels

int ap0 = A0;
int val = 0;
int brtns = 255;

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  pixels.begin();
  pixels.clear();
  Serial.begin(9600);
}

void loop() {
  // Readingt the dial //
  val = analogRead(ap0);
  //  ((val+125)/175)*(-230/6)+255 // 25-255
  //  brtns = 10;
  
//  brtns = ((val+125)/175)*(-254/6)+255; // LINEAR
  brtns = 510/(1+exp((val+125)/175)); // EXPONENTIAL (adding "-1 makes the min == off)
  
  Serial.println(brtns);
  pixels.setBrightness(brtns);
  // Readingt the dial //

  // Setting LED //
  //  pixels.clear();

  for(int i=0; i<NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 0, 255));
    pixels.show();
    delay(DELAYVAL);
  }



//
//  // Readingt the dial //
//  val = analogRead(ap0);
//  //  ((val+125)/175)*(-230/6)+255 // 25-255
//  //  brtns = 10;
//  
////  brtns = ((val+125)/175)*(-254/6)+255; // LINEAR
//  brtns = 510/(1+exp((val+125)/175)); // EXPONENTIAL
//  
//  Serial.println(brtns);
//  pixels.setBrightness(brtns);
//  // Readingt the dial //
//
//  for(int i=0; i<NUMPIXELS; i++) {
//    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
//    pixels.show();
//    delay(DELAYVAL);
//  }
//  // Setting LED //
}
