#include <Arduino.h>

// project include
#include "principles_of_measurement.hpp"

void setup() {
  Serial.begin(115200);
}

void loop() {
  principles_of_measurement.update();
  auto [left_wheel_speed, right_wheel_speed] =
      principles_of_measurement.get_wheel_speeds();

  Serial.printf("wheel speed:\n\tleft: %f\n\tright: %f)\n", left_wheel_speed, right_wheel_speed);
  Serial.printf("vehicle speed: %f\n", (left_wheel_speed + right_wheel_speed) / 2.0F * VEHICLE_WHEEL_RADIUS);
}
