# Encoder

Thread-owned, platform-independent four-wheel quadrature encoder sampler.

The GPIO callbacks perform four-times quadrature decoding. A dedicated high
priority thread takes a four-wheel snapshot and publishes the
converted motor data without depending on `ApplicationManager::MonitorAll()`.

## Hardware Mapping

The board layer must register eight `LibXR::GPIO` entries in the hardware
container:

- front left: `encoder_fl_a`, `encoder_fl_b`
- front right: `encoder_fr_a`, `encoder_fr_b`
- back left: `encoder_bl_a`, `encoder_bl_b`
- back right: `encoder_br_a`, `encoder_br_b`

The MSPM0 application maps these aliases to the `encoder_jie` SysConfig pins;
other platforms can provide any `LibXR::GPIO` implementation.

## Published Topic

The configured `topic_name` publishes `Encoder::MotorData`:

- `count`: cumulative signed quadrature counts
- `angle_rad`: cumulative shaft angle
- `speed_rad_s`: average angular speed since the previous sample
- `sample_time_ms`: LibXR monotonic sample time
- `sequence`: monotonically increasing frame sequence

Array order is front-left, front-right, back-left, back-right.

## Configuration

```yaml
- id: Encoder_0
  name: Encoder
  constructor_args:
    topic_name: encoder
    publish_period_ms: 5
    counts_per_rev: 1050.0
    task_stack_depth: 768
    thread_priority: LibXR::Thread::Priority::HIGH
```

`counts_per_rev` must be the output-shaft count for four-times decoding. Verify
it against the encoder line count and gearbox ratio before speed-loop tuning.
