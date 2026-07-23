# Encoder

Thread-owned four-wheel quadrature encoder sampler for MSPM0.

The GPIO callbacks perform four-times quadrature decoding. A dedicated high
priority thread takes a coherent four-wheel snapshot and publishes the
converted motor data without depending on `ApplicationManager::MonitorAll()`.

## Hardware Mapping

The module uses the SysConfig pin group `encoder_jie`:

- front left: `FLA`, `FLB`
- front right: `FRA`, `FRB`
- back left: `BLA`, `BLB`
- back right: `BRA`, `BRB`

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
