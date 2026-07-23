#include "Encoder.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#include "libxr_def.hpp"

namespace
{

using WheelCounts = std::array<int32_t, Encoder::kMotorCount>;

LibXR::GPIO& FindGPIO(LibXR::HardwareContainer& hw, const char* name)
{
  return *hw.FindOrExit<LibXR::GPIO>({name});
}

struct EncoderKinematics
{
  std::array<double, Encoder::kMotorCount> angle_rad{};
  std::array<float, Encoder::kMotorCount> speed_rad_s{};
};

/**
 * Convert the four accumulated encoder counts into wheel angles and speeds.
 * The first sample has no previous count, so it reports angles with zero speed.
 */
EncoderKinematics BuildKinematics(
    const WheelCounts& current_counts,
    const std::optional<WheelCounts>& previous_counts, float dt_seconds,
    float counts_per_rev)
{
  EncoderKinematics sample{};
  if (!std::isfinite(dt_seconds) || !std::isfinite(counts_per_rev) ||
      counts_per_rev <= 0.0F)
  {
    return sample;
  }

  constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
  const double radians_per_count = kTwoPi / counts_per_rev;
  const bool can_calculate_speed =
      previous_counts.has_value() && dt_seconds > 0.0F;

  for (size_t wheel = 0U; wheel < current_counts.size(); ++wheel)
  {
    sample.angle_rad[wheel] =
        static_cast<double>(current_counts[wheel]) * radians_per_count;

    if (!can_calculate_speed)
    {
      continue;
    }

    // Unsigned subtraction preserves the delta when the 32-bit counter wraps.
    // Map the modular result back to the signed range so reverse motion remains
    // negative across both the INT32_MIN and INT32_MAX boundaries.
    const uint32_t modular_delta =
        static_cast<uint32_t>(current_counts[wheel]) -
        static_cast<uint32_t>((*previous_counts)[wheel]);
    const int64_t count_delta =
        modular_delta <=
                static_cast<uint32_t>(std::numeric_limits<int32_t>::max())
            ? static_cast<int64_t>(modular_delta)
            : static_cast<int64_t>(modular_delta) - (INT64_C(1) << 32);

    sample.speed_rad_s[wheel] =
        static_cast<float>(static_cast<double>(count_delta) *
                           radians_per_count / dt_seconds);
  }

  return sample;
}

}  // namespace

namespace Module
{

QuadratureDecoder::QuadratureDecoder(LibXR::GPIO& a, LibXR::GPIO& b)
    : a_(a), b_(b)
{
}

void QuadratureDecoder::Init()
{
  const LibXR::GPIO::Configuration config = {
      LibXR::GPIO::Direction::FALL_RISING_INTERRUPT, LibXR::GPIO::Pull::UP};
  ASSERT(a_.SetConfig(config) == LibXR::ErrorCode::OK);
  ASSERT(b_.SetConfig(config) == LibXR::ErrorCode::OK);
  ASSERT(a_.RegisterCallback(
             LibXR::GPIO::Callback::Create(&QuadratureDecoder::OnEdge, this)) ==
         LibXR::ErrorCode::OK);
  ASSERT(b_.RegisterCallback(
             LibXR::GPIO::Callback::Create(&QuadratureDecoder::OnEdge, this)) ==
         LibXR::ErrorCode::OK);

  last_state_ = ReadPhase();
  ASSERT(a_.EnableInterrupt() == LibXR::ErrorCode::OK);
  ASSERT(b_.EnableInterrupt() == LibXR::ErrorCode::OK);
}

int32_t QuadratureDecoder::GetCount() const
{
  const uint32_t modular_count = count_.load(std::memory_order_relaxed);
  return std::bit_cast<int32_t>(modular_count);
}

void QuadratureDecoder::ResetCount()
{
  count_.store(0U, std::memory_order_relaxed);
}

uint8_t QuadratureDecoder::ReadPhase()
{
  const uint8_t a = a_.Read() ? 1U : 0U;
  const uint8_t b = b_.Read() ? 1U : 0U;
  return static_cast<uint8_t>((a << 1U) | b);
}

void QuadratureDecoder::OnEdge(bool in_isr, QuadratureDecoder* self)
{
  (void)in_isr;
  const uint8_t new_state = self->ReadPhase();
  const uint8_t index =
      static_cast<uint8_t>((self->last_state_ << 2U) | new_state);
  const int8_t transition = kTransitionTable[index];

  if (transition > 0)
  {
    self->count_.fetch_add(1U, std::memory_order_relaxed);
  }
  else if (transition < 0)
  {
    self->count_.fetch_sub(1U, std::memory_order_relaxed);
  }
  self->last_state_ = new_state;
}

}  // namespace Module

const char* Encoder::ValidateTopicName(const char* topic_name)
{
  ASSERT(topic_name != nullptr && topic_name[0] != '\0');
  return topic_name;
}

Encoder::Encoder(LibXR::HardwareContainer& hw,
                 LibXR::ApplicationManager& app, const char* topic_name,
                 uint32_t publish_period_ms, float counts_per_rev,
                 size_t task_stack_depth,
                 LibXR::Thread::Priority thread_priority)
    : decoders_{{
          {FindGPIO(hw, "encoder_fl_a"), FindGPIO(hw, "encoder_fl_b")},
          {FindGPIO(hw, "encoder_fr_a"), FindGPIO(hw, "encoder_fr_b")},
          {FindGPIO(hw, "encoder_bl_a"), FindGPIO(hw, "encoder_bl_b")},
          {FindGPIO(hw, "encoder_br_a"), FindGPIO(hw, "encoder_br_b")},
      }},
      counts_per_rev_(counts_per_rev),
      publish_period_ms_(publish_period_ms),
      data_topic_(LibXR::Topic::CreateTopic<MotorData>(
          ValidateTopicName(topic_name)))
{
  ASSERT(std::isfinite(counts_per_rev_) && counts_per_rev_ > 0.0F);
  ASSERT(task_stack_depth > 0U);

  for (auto& decoder : decoders_)
  {
    decoder.Init();
  }

  app.Register(*this);
  hw.Register(LibXR::Entry<Encoder>{*this, {"encoder"}});
  thread_.Create(this, ThreadFunc, "encoder", task_stack_depth,
                 thread_priority);
}

LibXR::Topic& Encoder::DataTopic() { return data_topic_; }

void Encoder::ResetAll()
{
  for (auto& decoder : decoders_)
  {
    decoder.ResetCount();
  }
  previous_counts_.reset();
  last_sample_time_ = {};
  latest_data_ = {};
  has_published_ = false;
}

Encoder::WheelCounts Encoder::ReadCounts() const
{
  WheelCounts counts{};
  for (size_t wheel = 0U; wheel < decoders_.size(); ++wheel)
  {
    counts[wheel] = decoders_[wheel].GetCount();
  }
  return counts;
}

void Encoder::PublishIfDue()
{
  const LibXR::MicrosecondTimestamp now_us =
      LibXR::Timebase::GetMicroseconds();
  if (has_published_ && publish_period_ms_ != 0U &&
      (now_us - last_sample_time_).ToMicrosecond() <
          static_cast<uint64_t>(publish_period_ms_) * 1000ULL)
  {
    return;
  }

  const WheelCounts counts = ReadCounts();
  const float dt_seconds = has_published_
                               ? (now_us - last_sample_time_).ToSecondf()
                               : 0.0F;
  const EncoderKinematics kinematics =
      BuildKinematics(counts, previous_counts_, dt_seconds, counts_per_rev_);

  MotorData data{};
  data.count = counts;
  data.angle_rad = kinematics.angle_rad;
  data.speed_rad_s = kinematics.speed_rad_s;
  data.sample_time_ms = static_cast<uint32_t>(
      static_cast<uint64_t>(now_us) / 1000ULL);
  data.sequence = sequence_++;

  previous_counts_ = counts;
  last_sample_time_ = now_us;
  latest_data_ = data;
  has_published_ = true;
  data_topic_.Publish(latest_data_);
}

void Encoder::ThreadFunc(Encoder* self)
{
  LibXR::MillisecondTimestamp wakeup = LibXR::Thread::GetTime();
  while (true)
  {
    self->PublishIfDue();
    LibXR::Thread::SleepUntil(wakeup, kWorkerPeriodMs);
  }
}
