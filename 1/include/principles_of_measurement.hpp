// stl include
#include <tuple>

// library include
#include <arduino-timer.h>
#include <ESP32Encoder.h>

#define VEHICLE_WHEEL_RADIUS 0.05F  // m

#define ENCODER_UPDATE_PERIOD 10 // ms
#define ENCODER_RESOLUTION 20
#define ENCODER_LEFT_A_PIN 1
#define ENCODER_LEFT_B_PIN 2
#define ENCODER_RIGHT_A_PIN 3
#define ENCODER_RIGHT_B_PIN 4

/// @brief Class for the principles of measurement.
class PrinciplesOfMeasurement {
 public:
  /// @brief Constructor.
  PrinciplesOfMeasurement();

  /// @brief Update the state of the class.
  void update();

  /// @brief Get the wheel speeds in rad/s.
  std::tuple<float, float> get_wheel_speeds();

 private:
  /// @brief Callback function for measuring the wheel speed.
  bool encoder_timer_callback(void* /*arg*/);

  /// @brief Timer to periodically update.
  Timer<1> timer_;

  /// @brief Motor encoder.
  ESP32Encoder left_encoder_, right_encoder_;

  /// @brief Wheel speeds.
  float left_wheel_speed_, right_wheel_speed_;
};

extern PrinciplesOfMeasurement principles_of_measurement;
