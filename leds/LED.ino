#include <FastLED.h>

#define NUM_MATRIX 8

#define NUM_ROW 8

#define NUM_LEDS NUM_MATRIX * NUM_ROW * NUM_ROW

#define DATA_PIN 3

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  Serial.begin(9600);
}

CRGB color = CRGB::Blue;

uint8_t buffer[3];

uint8_t index = 0;

uint8_t mode = 0;

int led = 0;

void loop() {
  while (Serial.available() > 0) {
    // Read last
    uint8_t input = Serial.read();
    if (input < 2) {
      // Update color
      if (index == 3)
        color = CRGB((buffer[0] - 2) * 8, (buffer[1] - 2) * 8, (buffer[2] - 2) * 8);
      // Change mode
      mode = input;
      // Reset index
      index = 0;
    } else {
      // Write buffer
      if (index < 3)
        buffer[index++] = input;
    }
  }
  if (mode == 0) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = color;
    }
    FastLED.show();
  } else if (mode == 1) {
    if (led < NUM_LEDS) {
      for (uint8_t a = 0; a < NUM_ROW; ++a) {
        leds[led + a] = color;
      }
      led += NUM_ROW;
      FastLED.show();
    } else {
      for (led = 0; led < NUM_LEDS; ++led) {
        leds[led] = CRGB::Black;
      }
      FastLED.show();
      led = 0;
    }
  }
  delay(20);
}