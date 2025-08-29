import math
import matplotlib.pyplot as plt
import serial
import time

# 차량 파라미터
L = 2.9
WB = 2.9

# PID 제어 파라미터
Kp = 1.0
Ki = 0.1
Kd = 0.1

k = 0.1
dt = 1.0
Lfc = 1.0
base_target_speed = 0.3
show_animation = True  # 전역변수로 선언

# ==================== 여기부터 기존 코드 ====================

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt

class PIDControl:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = None

    def control(self, target, current):
        error = target - current
        self.integral += error * dt
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class TargetCourse:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        if self.old_nearest_point_index is None:
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = [abs(math.hypot(idx, idy)) for (idx, idy) in zip(dx, dy)]
            ind = d.index(min(d))
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = math.hypot(state.x - self.cx[ind], state.y - self.cy[ind])
            while True:
                if (ind + 1) >= len(self.cx):
                    break
                distance_next_index = math.hypot(state.x - self.cx[ind + 1], state.y - self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind += 1
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc
        while Lf > math.hypot(state.x - self.cx[ind], state.y - self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break
            ind += 1
        return ind

    def calc_curvature(self, ind):
        if ind < 1 or ind > len(self.cx) - 2:
            return 0.0
        x1, y1 = self.cx[ind - 1], self.cy[ind - 1]
        x2, y2 = self.cx[ind], self.cy[ind]
        x3, y3 = self.cx[ind + 1], self.cy[ind + 1]
        a = math.hypot(x2 - x1, y2 - y1)
        b = math.hypot(x3 - x2, y3 - y2)
        c = math.hypot(x3 - x1, y3 - y1)
        if a == 0 or b == 0 or c == 0:
            return 0.0
        s = (a + b + c) / 2
        area = math.sqrt(max(s * (s - a) * (s - b) * (s - c), 0.0))
        try:
            curvature = 4 * area / (a * b * c)
        except ZeroDivisionError:
            curvature = 0.0
        return curvature

class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)

def pure_pursuit_steer_control(state, trajectory, pind):
    ind = trajectory.search_target_index(state)
    if pind >= ind:
        ind = pind
    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
    Lf = k * state.v + Lfc
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
    return delta, ind

def plot_arrow(x, y, yaw, length=1.0, width=0.5):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

# ==================== 여기부터 main 함수 및 수정 부분 ====================

def main():
    global show_animation

    # 시리얼 연결 재시도
    ser = None
    while ser is None:
        try:
            ser = serial.Serial('COM10', 4800, timeout=0.1)
            time.sleep(0.1)
            print("Serial connected")
        except Exception as e:
            print(f"Serial connection error, retrying in 3 sec: {e}")
            time.sleep(3)

    if show_animation:
        plt.figure()
        plt.show(block=False)

    cx = list(range(0, 50))
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    target_course = TargetCourse(cx, cy)

    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    time_ = 0.0
    states = States()
    target_ind = target_course.search_target_index(state)
    pid = PIDControl(Kp, Ki, Kd)

    last_send_time = time.time() - 10.0
    send_interval = 0.1

    # 엔코더 rad/s 저장 변수
    left_rad_per_sec = 0.0
    right_rad_per_sec = 0.0

    while True:
        try:
            # 1) 아두이노로 속도 명령 전송
            curvature = target_course.calc_curvature(target_ind)
            speed_scale = max(0.3, 1.0 - 5.0 * curvature)
            target_speed = base_target_speed * speed_scale

            ai = pid.control(target_speed, state.v)
            di, target_ind = pure_pursuit_steer_control(state, target_course, target_ind)

            state.update(ai, di)
            time_ += dt
            states.append(time_, state)

            linear_velocity = state.v
            angular_velocity = state.v / WB * math.tan(di)

            current_time = time.time()
            if current_time - last_send_time >= send_interval:
                cmd = f"L:{linear_velocity:.3f},A:{angular_velocity:.3f}\n"
                ser.write(cmd.encode())
                ser.flush()
                last_send_time = current_time

            # 2) 아두이노에서 시리얼 데이터 읽기 (엔코더 rad/s)
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                # 예: LRS:0.1234,RRS:0.2345
                if line.startswith("LRS:") and "RRS:" in line:
                    try:
                        parts = line.split(",")
                        left_str = parts[0].split(":")[1]
                        right_str = parts[1].split(":")[1]
                        left_rad_per_sec = float(left_str)
                        right_rad_per_sec = float(right_str)
                        print(f"Left Encoder rad/s: {left_rad_per_sec:.4f}, Right Encoder rad/s: {right_rad_per_sec:.4f}")
                    except Exception as e:
                        print(f"Parsing error: {e}, line: {line}")

            # 3) 그래프 그리기
            if show_animation and int(time_ * 10) % 2 == 0:
                plt.cla()
                plot_arrow(state.x, state.y, state.yaw)
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(states.x, states.y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title(f"Speed [m/s]: {state.v:.2f}")
                plt.pause(0.01)

            time.sleep(0.1)

        except KeyboardInterrupt:
            print("User interrupted")
            break

        except Exception as e:
            print(f"Runtime error caught, continuing loop: {e}")
            time.sleep(0.1)

    ser.close()
    print("Serial port closed")
    print("Done")

    if show_animation:
        plt.figure()
        plt.plot(states.t, states.v, "-r")
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    main()
