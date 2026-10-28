# Feetech 总线（A733 + 复刻 HAT）

这是 `feat/feetech` 分支：用复刻 HAT 的 **Feetech HD-1910M** 舵机总线替换原版
Microduck 的 Dynamixel XL330 总线，控制环、kinematics 和策略不变。

## 编译

```bash
cargo build -p robotd --features feetech
```

默认（不带 feature）仍是 Dynamixel。

## 关节单位 / 方向 / 零点

**默认就是原版约定**，所以原版训练的策略看到的是同一组数字：

- 位置：`rad = 2π·count/4096 − π`（2048 → 0 rad，与 XL330 相同）；
- 速度：`0.732 RPM/LSB`（HD-1910M 手册值；XL330 是 0.229）；
- 方向：正方向同原版；镜像安装的关节用 `sign = -1`；
- 零点：默认 2048；机械零位不同的关节用 `offset`（弧度）平移。

两个环境变量按 `JOINT_IDS` 顺序给 15 个值：

```bash
# 例：第 1 个关节镜像，第 3 个关节零点 +5°
export MICRODUCK_FEETECH_SIGNS="-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1"
export MICRODUCK_FEETECH_OFFSETS="0,0,0.0873,0,0,0,0,0,0,0,0,0,0,0,0"
```

不设置就是 `sign=+1, offset=0`。

## IMU

- 复刻 HAT 的 IMU 在 **I2C**（LSM6DSV16X / BMI088），本分支还没有 I2C 读取器，
  默认给中性姿态（`quat = [1,0,0,0]`、重力 `[0,0,-1]`）。所以第一步只能
  `[policy] enabled = false` 跑控制环、保持姿势；
- 如果 IMU 板在舵机总线上（原版接线，Feetech ID 200、地址 56、12 字节块），
  设 `MICRODUCK_IMU_ON_BUS=1`，解码器与 Dynamixel 后端完全一致。

## 先上板验证

```bash
# 1) 总线和 ID（用复刻仓库的工具）
python3 tools/imu200/imu200.py demo
python3 tools/servo-web/feetech.py --port /dev/ttyS0 ping 1

# 2) 只读位置，确认方向/零点
robotd --port /dev/ttyS0   # 先 [policy] enabled = false

# 3) 扭矩、增益、限幅
```

## 尚未实现

- `adopt_replacement`（换舵机自动刷 ID）——需要按校准文件里的关节顺序实现；
- I2C IMU 读取器（LSM6DSV16X SFLP / BMI088 融合）——下一个提交；
- 把校准从环境变量搬进 `robotd.toml`。
