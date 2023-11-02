// stl include
#include <tuple>

// library include
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <arduino-timer.h>

#define VEHICLE_TARGET_SPEED 0.1F   // m/s
#define VEHICLE_WHEEL_RADIUS 0.05F  // m

#define ENCODER_UPDATE_PERIOD 10  // ms
#define ENCODER_RESOLUTION 20
#define ENCODER_LEFT_A_PIN 1
#define ENCODER_LEFT_B_PIN 2
#define ENCODER_RIGHT_A_PIN 3
#define ENCODER_RIGHT_B_PIN 4

#define CONTROL_UPDATE_PERIOD 50  // ms

// should be tuned
#define CONTROL_PID_KP 1.0
#define CONTROL_PID_KI 0.0
#define CONTROL_PID_KD 0.0

#define FAN_SERVO_PIN 5
#define FAN_SERVO_NUTRAL 1500
#define FAN_SERVO_MAX 2000

#define POWER_CONSUMPTION_UPDATE_PERIOD 10  // ms
#define POWER_VOLTAGE_DIVIDER_RATIO 0.1F
#define POWER_SHUNT_RESISTANCE 0.1F
#define POWER_ADC_RESOLUTION 4096
#define POWER_ADC_REFERENCE 3.3F
#define POWER_VOLTAGE_PIN 6
#define POWER_SHUNT_PIN 7

/// @brief Class for the principles of measurement.
class PrinciplesOfMeasurement {
 public:
  /// @brief Constructor.
  PrinciplesOfMeasurement();

  /// @brief Update the state of the class.
  void update();

  /// @brief Get the wheel speeds in rad/s.
  std::tuple<float, float> get_wheel_speeds();

  /// @brief Get the power consumption in W.
  float get_power_consumption();

 private:
  /// @brief Callback function for the measuring the wheel speed.
  bool encoder_timer_callback(void* /*arg*/);

  /// @brief Callback function for the control.
  bool control_timer_callback(void* /*arg*/);

  /// @brief Callback function for the measuring the fan power consumptuon.
  bool power_consumption_timer_callback(void* /*arg*/);

  /// @brief Timer to periodically update.
  Timer<3> timer_;

  /// @brief Motor encoder.
  ESP32Encoder left_encoder_, right_encoder_;

  /// @brief Wheel speeds.
  float left_wheel_speed_, right_wheel_speed_;

  /// @brief PID controller for vehicle speed.
  PID vehicle_speed_pid_;

  /// @brief Servo to control the fan.
  Servo fan_servo_;

  /// @brief Target wheel speed.
  static double target_wheel_speed_;

  /// @brief PID speed input.
  double wheel_speed_;

  /// @brief PID control output.
  double control_output_;

  /// @brief Power consumption.
  float power_consumption_;
};

extern PrinciplesOfMeasurement principles_of_measurement;
