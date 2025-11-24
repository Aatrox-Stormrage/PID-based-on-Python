import time


class PIDController:
    def __init__(self, kp=0, ki=0, kd=0, setpoint=0, output_limits=None):
        """
        初始化PID控制器
        :param kp: 比例系数
        :param ki: 积分系数
        :param kd: 微分系数
        :param setpoint: 目标设定值
        :param output_limits: 控制量输出范围 (min, max)，防止控制量过大
        """
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数
        self.setpoint = setpoint  # 目标值

        # 状态变量：记录历史信息用于积分和微分计算
        self.last_error = 0  # 上一次的偏差
        self.integral = 0  # 积分项累积
        self.last_time = None  # 上一次计算的时间

        # 控制量输出限制（可选）
        self.output_limits = output_limits if output_limits else (-float('inf'), float('inf'))

    def set_setpoint(self, setpoint):
        """更新目标设定值"""
        self.setpoint = setpoint

    def compute(self, process_value, current_time=None):
        """
        计算当前控制量
        :param process_value: 系统当前实际值
        :param current_time: 当前时间（若为None则自动获取）
        :return: 计算得到的控制量
        """
        # 处理时间（首次调用时初始化）
        if current_time is None:
            current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return 0  # 首次调用无历史数据，返回0

        # 计算时间间隔（dt）
        dt = current_time - self.last_time
        if dt <= 0:
            return 0  # 时间间隔无效，返回0

        # 计算当前偏差（目标值 - 实际值）
        error = self.setpoint - process_value

        # 1. 比例项：与当前偏差成正比
        proportional = self.kp * error

        # 2. 积分项：累积历史偏差（消除稳态误差）
        self.integral += self.ki * error * dt
        # 积分限幅（抗积分饱和，防止积分项过大）
        self.integral = max(
            min(self.integral, self.output_limits[1] / self.ki if self.ki != 0 else 0),
            self.output_limits[0] / self.ki if self.ki != 0 else 0
        )

        # 3. 微分项：反映偏差变化率（抑制超调）
        derivative = self.kd * (error - self.last_error) / dt

        # 总控制量 = 比例项 + 积分项 + 微分项
        output = proportional + self.integral + derivative

        # 限制控制量输出范围
        output = max(min(output, self.output_limits[1]), self.output_limits[0])

        # 更新历史状态
        self.last_error = error
        self.last_time = current_time

        return output