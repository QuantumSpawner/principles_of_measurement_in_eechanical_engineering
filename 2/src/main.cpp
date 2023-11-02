#include <Arduino.h>

// project include
#include "principles_of_measurement.hpp"

void setup() { Serial.begin(115200); }

void loop() {
  principles_of_measurement.update();
  Serial.printf("fan power consumption: %f\n",
                principles_of_measurement.get_power_consumption());
}
