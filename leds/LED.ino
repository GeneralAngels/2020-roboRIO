#include <FastLED.h>

#define NUM_MATRIX 8

#define NUM_LEDS NUM_MATRIX * 64

#define DATA_PIN 3

CRGB leds[NUM_LEDS];

uint8_t buffer[4];

void setup() {
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  Serial.begin(9600);
}

CRGB color = CRGB::Blue;

bool offseted = false;

void loop() {
  if (Serial.available() > 0) {
    if (!offseted) {
      uint8_t i = Serial.read();
      if (i < 2)
        offseted = true;
    } else {
      Serial.readBytes(buffer, 4);
      color = CRGB((buffer[1]-2)*8, (buffer[2]-2)*8, (buffer[3]-2)*8);
    }
  }
  if (buffer[0] == 0) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = color;
    }
    FastLED.show();
    delay(50);
  } else if (buffer[0] == 1) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
    }
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = color;
      if (i % 8 == 0) {
        FastLED.show();
        delay(50);
      }
    }
  }
}
