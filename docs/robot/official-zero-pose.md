# 官方模型的零点姿势（HOME_FRAME）

`DEFAULT_POSITION`（`duck-control/src/model.rs`）就是训练环境的 **`HOME_FRAME`**：
每个策略观测的是**相对这个姿势**的关节角，所以它是复刻机要对齐的“零点”。
`REST_POSITION` 是卸力坐下后稳定下来的姿势，**不是**策略零点，别混用。

导出（代码是唯一来源，不会和固件漂移）：

```bash
cargo run -p duck-control --example official-zero-pose          # markdown
cargo run -p duck-control --example official-zero-pose -- --csv
cargo run -p duck-control --example official-zero-pose -- --json
```

> 先标“所有舵机 2048”的零点姿势：见 [`zero-pose.md`](zero-pose.md)。
> 本文的表是 **HOME**（策略零点），不是舵机零点。

## 表

| joint | ID | home (rad) | home (°) | home counts (2048=0) | rest (rad) | EEPROM homing |
|---|---:|---:|---:|---:|---:|---:|
| left_hip_yaw | 20 | 0.0000 | 0.00 | 2048 | 0.3129 | 0 |
| left_hip_roll | 21 | -0.0873 | -5.00 | 1991 | 0.4050 | 0 |
| left_hip_pitch | 22 | -0.4579 | -26.24 | 1749 | -0.2338 | 0 |
| left_knee | 23 | -0.0049 | -0.28 | 2045 | 1.5493 | **-512** |
| left_ankle | 24 | 0.4530 | 25.95 | 2343 | 0.5522 | 0 |
| neck_pitch | 30 | 0.3491 | 20.00 | 2276 | 0.9572 | 0 |
| head_pitch | 31 | 0.3491 | 20.00 | 2276 | 1.4787 | 0 |
| head_yaw | 32 | 0.0000 | 0.00 | 2048 | -0.0129 | 0 |
| head_roll | 33 | 0.0000 | 0.00 | 2048 | -0.0522 | 0 |
| mouth | 34 | 0.0000 | 0.00 | 2048 | -0.0121 | 0 |
| right_hip_yaw | 10 | 0.0000 | 0.00 | 2048 | 0.0353 | 0 |
| right_hip_roll | 11 | 0.0873 | 5.00 | 2105 | -0.5660 | 0 |
| right_hip_pitch | 12 | 0.4579 | 26.24 | 2347 | 0.2853 | 0 |
| right_knee | 13 | 0.0049 | 0.28 | 2051 | -1.4696 | **-512** |
| right_ankle | 14 | -0.4530 | -25.95 | 1753 | -0.4096 | 0 |

嘴巴：闭合 -5.00°，全开 +30.00°（不在任何策略的 14 维动作里）。

## 怎么用它对齐复刻机

1. **舵机 ID**：与原版一致（左腿 20–24、颈/头/嘴 30–34、右腿 10–14），不需要改
   `JOINT_IDS`。
2. **机械零位**：把机器人摆成 HOME 姿势时，每颗舵机的读数应当是上表的
   `home counts`（Feetech 用 `2048 + rad·4096/2π`）。实测差 `Δ` counts 时：
   - 物理上转舵盘最直接；或
   - 在 `MICRODUCK_FEETECH_OFFSETS` 里给该关节 `Δ·2π/4096` 弧度。
3. **方向**：某个关节正方向和原版相反时，`MICRODUCK_FEETECH_SIGNS` 给 `-1`，
   同时把该关节的 offset 也取反。
4. **膝盖的 -512**：官方膝盖 EEPROM 里 `homing_offset = -512`（-45°），
   也就是 `present = raw + 512`。复刻有两种做法：
   - 按官方装配把膝盖舵盘偏 45°（物理零位），软件 offset 保持 0；
   - 或者装在中位，软件给膝盖 `offset = +0.7854 rad`（`512·2π/4096`）补回来。
   两种只能选一种，别叠加。
5. **验证**：摆到 HOME，跑 `robotd`（`[policy] enabled = false`），读回来的
   `positions` 应接近上表 `home (rad)`；偏差超过 ~0.02 rad 就要重新标。

## 和 Feetech 单位的关系

Feetech HD-1910M 也是 4096 counts/圈、2048 中位，所以 `home counts` 直接可用；
唯一的单位差异是**速度**：HD-1910M 是 0.732 RPM/LSB，原版 XL330 是 0.229。
本分支的 `FeetechIo` 已经按 0.732 换算（见 `docs/robot/feetech-bus.md`）。
