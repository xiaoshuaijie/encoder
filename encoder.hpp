#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Thread-owned four-wheel quadrature encoder sampler with Topic publishing
constructor_args:
  - topic_name: "encoder"
  - publish_period_ms: 5
  - counts_per_rev: 1050.0
  - task_stack_depth: 768
  - thread_priority: LibXR::Thread::Priority::HIGH
template_args: []
required_hardware:
  - encoder_fl_a
  - encoder_fl_b
  - encoder_fr_a
  - encoder_fr_b
  - encoder_bl_a
  - encoder_bl_b
  - encoder_br_a
  - encoder_br_b
depends: []
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "app_framework.hpp"
#include "gpio.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "timebase.hpp"

namespace Module
{

class QuadratureDecoder
{
 public:
  QuadratureDecoder(LibXR::GPIO& a, LibXR::GPIO& b);

  QuadratureDecoder(const QuadratureDecoder&) = delete;
  QuadratureDecoder(QuadratureDecoder&&) = delete;
  QuadratureDecoder& operator=(const QuadratureDecoder&) = delete;
  QuadratureDecoder& operator=(QuadratureDecoder&&) = delete;

  void Init();
  [[nodiscard]] int32_t GetCount() const;
  void ResetCount();

 private:
  uint8_t ReadPhase();
  static void OnEdge(bool in_isr, QuadratureDecoder* self);

  static constexpr int8_t kTransitionTable[16] = {
      0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};

  LibXR::GPIO& a_;
  LibXR::GPIO& b_;
  std::atomic<uint32_t> count_{0U};
  uint8_t last_state_ = 0U;
};

}  // namespace Module

class Encoder : public LibXR::Application
{
 public:
  enum MotorId : size_t
  {
    kFrontLeft = 0U,
    kFrontRight,
    kBackLeft,
    kBackRight,
    kMotorCount
  };

  struct MotorData
  {
    std::array<int32_t, kMotorCount> count{};
    std::array<double, kMotorCount> angle_rad{};
    std::array<float, kMotorCount> speed_rad_s{};
    uint32_t sample_time_ms = 0U;
    uint32_t sequence = 0U;
  };

  using Sample = MotorData;

  Encoder(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
          const char* topic_name, uint32_t publish_period_ms,
          float counts_per_rev, size_t task_stack_depth,
          LibXR::Thread::Priority thread_priority);

  Encoder(const Encoder&) = delete;
  Encoder(Encoder&&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  Encoder& operator=(Encoder&&) = delete;

  LibXR::Topic& DataTopic();
  LibXR::Topic& SampleTopic() { return data_topic_; }
  [[nodiscard]] const MotorData& LatestData() const { return latest_data_; }
  void ResetAll();
  void OnMonitor() override {}

 private:
  static constexpr uint32_t kWorkerPeriodMs = 5U;
  using WheelCounts = std::array<int32_t, kMotorCount>;

  static const char* ValidateTopicName(const char* topic_name);
  static void ThreadFunc(Encoder* self);

  [[nodiscard]] WheelCounts ReadCounts() const;
  void PublishIfDue();

  std::array<Module::QuadratureDecoder, kMotorCount> decoders_;
  float counts_per_rev_ = 0.0F;
  uint32_t publish_period_ms_ = 5U;
  LibXR::Topic data_topic_;
  std::optional<WheelCounts> previous_counts_;
  LibXR::MicrosecondTimestamp last_sample_time_{};
  MotorData latest_data_{};
  uint32_t sequence_ = 0U;
  bool has_published_ = false;
  LibXR::Thread thread_;
};
