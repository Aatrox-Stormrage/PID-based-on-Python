import matplotlib.pyplot as plt
import PID

def simulate_system(current_temp, power, dt):
    """
    模拟系统响应：温度随加热功率和时间的变化
    简单模型：温度变化 = 功率 * 0.1 - 自然散热（假设散热系数0.05）
    """
    heat_gain = power * 0.1  # 加热带来的温度上升
    heat_loss = 0.05 * current_temp  # 自然散热导致的温度下降
    new_temp = current_temp + (heat_gain - heat_loss) * dt
    return new_temp

# 初始化PID控制器（参数需根据实际系统调试）
pid = PID.PIDController(
    kp=2.0,    # 比例系数：快速响应偏差
    ki=0.1,    # 积分系数：消除稳态误差
    kd=0.5,    # 微分系数：抑制超调
    setpoint=50,  # 目标温度50℃
    output_limits=(0, 100)  # 加热功率限制在0~100
)

# 模拟过程
current_temp = 20.0  # 初始温度
history = {
    'time': [],
    'temperature': [],
    'power': []
}

# 模拟30秒，每0.5秒采样一次
for i in range(60):
    t = i * 0.5
    # 计算当前加热功率（控制量）
    power = pid.compute(current_temp, current_time=t)
    # 模拟系统温度变化
    current_temp = simulate_system(current_temp, power, dt=0.5)
    # 记录历史数据
    history['time'].append(t)
    history['temperature'].append(current_temp)
    history['power'].append(power)

# 绘图展示控制效果
plt.figure(figsize=(12, 6))

# 温度曲线
plt.subplot(2, 1, 1)
plt.plot(history['time'], history['temperature'], label='实际温度')
plt.axhline(y=pid.setpoint, color='r', linestyle='--', label='目标温度')
plt.ylabel('温度 (℃)')
plt.legend()

# 控制量（功率）曲线
plt.subplot(2, 1, 2)
plt.plot(history['time'], history['power'], label='加热功率', color='g')
plt.xlabel('时间 (s)')
plt.ylabel('功率')
plt.legend()

plt.tight_layout()
plt.show()