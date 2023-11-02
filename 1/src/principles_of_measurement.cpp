#include "principles_of_measurement.hpp"

// stl include
#include <math.h>
#include <tuple>
#include <functional>

// library include
#include <arduino-timer.h>

PrinciplesOfMeasurement::PrinciplesOfMeasurement() {
  // register encoder timer callback
  timer_.every(ENCODER_UPDATE_PERIOD,
               *static_cast<std::function<bool(void*)>>(
                    std::bind(&PrinciplesOfMeasurement::encoder_timer_callback,
                              this, std::placeholders::_1))
                    .target<bool (*)(void*)>(),
               nullptr);

  // initialize encoders
  left_encoder_.attachFullQuad(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
  right_encoder_.attachFullQuad(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
}

void PrinciplesOfMeasurement::update() { timer_.tick(); }

std::tuple<float, float> PrinciplesOfMeasurement::get_wheel_speeds() {
  return std::make_tuple(left_wheel_speed_, right_wheel_speed_);
}

bool PrinciplesOfMeasurement::encoder_timer_callback(void* /*arg*/) {
  left_wheel_speed_ = 2 * M_PI * left_encoder_.getCount() * 1000.0F /
                      ENCODER_RESOLUTION / ENCODER_UPDATE_PERIOD;
  right_wheel_speed_ = 2 * M_PI * right_encoder_.getCount() * 1000.0F /
                       ENCODER_RESOLUTION / ENCODER_UPDATE_PERIOD;
  return true;
}

PrinciplesOfMeasurement principles_of_measurement;
