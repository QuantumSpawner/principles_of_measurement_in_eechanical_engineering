#include "principles_of_measurement.hpp"

// stl include
#include <math.h>

#include <functional>
#include <tuple>

// library include
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <arduino-timer.h>

PrinciplesOfMeasurement::PrinciplesOfMeasurement()
    : vehicle_speed_pid_(&wheel_speed_, &target_wheel_speed_, &wheel_speed_,
                         CONTROL_PID_KP, CONTROL_PID_KI, CONTROL_PID_KD,
                         DIRECT) {
  // register timer callback
  timer_.every(ENCODER_UPDATE_PERIOD,
               *static_cast<std::function<bool(void*)>>(
                    std::bind(&PrinciplesOfMeasurement::encoder_timer_callback,
                              this, std::placeholders::_1))
                    .target<bool (*)(void*)>(),
               nullptr);
  timer_.every(CONTROL_UPDATE_PERIOD,
               *static_cast<std::function<bool(void*)>>(
                    std::bind(&PrinciplesOfMeasurement::control_timer_callback,
                              this, std::placeholders::_1))
                    .target<bool (*)(void*)>(),
               nullptr);
  timer_.every(
      POWER_CONSUMPTION_UPDATE_PERIOD,
      *static_cast<std::function<bool(void*)>>(
           std::bind(&PrinciplesOfMeasurement::power_consumption_timer_callback,
                     this, std::placeholders::_1))
           .target<bool (*)(void*)>(),
      nullptr);

  // initialize encoders
  left_encoder_.attachFullQuad(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
  right_encoder_.attachFullQuad(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);

  // initialize PID controller
  // assume only goes forward
  vehicle_speed_pid_.SetOutputLimits(0.0F, 1.0F);
  vehicle_speed_pid_.SetMode(AUTOMATIC);

  // initialize servo
  ESP32PWM::allocateTimer(1);
  fan_servo_.setPeriodHertz(50);
  fan_servo_.attach(FAN_SERVO_PIN);
  fan_servo_.writeMicroseconds(FAN_SERVO_NUTRAL);

  // initialize power consumption
  pinMode(POWER_VOLTAGE_PIN, INPUT);
  pinMode(POWER_SHUNT_PIN, INPUT);
}

void PrinciplesOfMeasurement::update() { timer_.tick(); }

std::tuple<float, float> PrinciplesOfMeasurement::get_wheel_speeds() {
  return std::make_tuple(left_wheel_speed_, right_wheel_speed_);
}

float PrinciplesOfMeasurement::get_power_consumption() {
  return power_consumption_;
}

bool PrinciplesOfMeasurement::encoder_timer_callback(void* /*arg*/) {
  left_wheel_speed_ = 2 * M_PI * left_encoder_.getCount() * 1000.0F /
                      ENCODER_RESOLUTION / ENCODER_UPDATE_PERIOD;
  right_wheel_speed_ = 2 * M_PI * right_encoder_.getCount() * 1000.0F /
                       ENCODER_RESOLUTION / ENCODER_UPDATE_PERIOD;
  return true;
}

bool PrinciplesOfMeasurement::control_timer_callback(void* /*arg*/) {
  wheel_speed_ = (left_wheel_speed_ + right_wheel_speed_) / 2.0F;

  vehicle_speed_pid_.Compute();

  fan_servo_.writeMicroseconds(
      FAN_SERVO_NUTRAL + (FAN_SERVO_MAX - FAN_SERVO_NUTRAL) * control_output_);

  return true;
}

bool PrinciplesOfMeasurement::power_consumption_timer_callback(void* /*arg*/) {
  float voltage = static_cast<float>(analogRead(POWER_VOLTAGE_PIN)) /
                  POWER_ADC_RESOLUTION * POWER_ADC_REFERENCE /
                  POWER_VOLTAGE_DIVIDER_RATIO;
  float current = static_cast<float>(analogRead(POWER_SHUNT_PIN)) /
                  POWER_ADC_RESOLUTION * POWER_ADC_REFERENCE /
                  POWER_SHUNT_RESISTANCE;
  power_consumption_ = voltage * current;
  return true;
}

double PrinciplesOfMeasurement::target_wheel_speed_ =
    VEHICLE_TARGET_SPEED / VEHICLE_WHEEL_RADIUS;

PrinciplesOfMeasurement principles_of_measurement;
