
from simple_pid import PID
import pynput,win32con
#import pydirectinput
import win32api
import time
import threading


lock_tag = '0'

# 线程锁
lock = threading.Lock()

# define some global variables
targetRealX, targetRealY = 0, 0
current_mouse_x, current_mouse_y = 0, 0
lock_state = False
dist = False
pid_exit_flag = False



def pid_control():
    print('PID thread started')
    Kp, Ki, Kd = 0.15, 0, 0.2
    move_max = 20 # 鼠标一次最大移动距离
    integral_x = 0
    integral_y = 0
    prev_error_x = 0
    prev_error_y = 0
    local_lock_state = False
    local_pid_exit_flag = False
    while True:
        # update local variables
        with lock:
            local_lock_state = lock_state
            local_pid_exit_flag = pid_exit_flag
        if local_pid_exit_flag:
            break
        # print(f"mouse lock state: {lock_state}")
        if local_lock_state:
            error_X = targetRealX - current_mouse_x
            error_Y = targetRealY - current_mouse_y
            integral_x += error_X
            integral_y += error_Y
            derivative_x = error_X - prev_error_x
            derivative_y = error_Y - prev_error_y
            prev_error_x = error_X
            prev_error_y = error_Y
            output_x = Kp * error_X + Ki * integral_x + Kd * derivative_x
            output_y = Kp * error_Y + Ki * integral_y + Kd * derivative_y
            # 限制输出
            output_x = max(min(output_x, move_max), -move_max)
            output_y = max(min(output_y, move_max), -move_max)
            print(f"Error_X: {error_X}, Error_Y: {error_Y}, 鼠标移动 x:{output_x} y:{output_y}")
            win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, int(output_x), int(output_y))
        else:
            integral_x = 0
            integral_y = 0
            prev_error_x = 0
            prev_error_y = 0
        time.sleep(0.01)
    print('PID thread exited')


class MouseLock:
    def __init__(self, shot_Width, shot_Height) -> None:
        global current_mouse_x, current_mouse_y, dist, targetRealX, targetRealY
        self.screen_width, self.screen_height = (2560,1440) # Screen resolution
        self.target_offset_y = -0.0 # target offset
        dist = False # distance small enough
        self.mouse = pynput.mouse.Controller() # mouse controller
        targetRealX, targetRealY = self.screen_width//2, self.screen_height//2
        current_mouse_x, current_mouse_y = self.screen_width//2, self.screen_height//2
        self.shot_Width, self.shot_Height = shot_Width, shot_Height
        pid_thread = threading.Thread(target=pid_control)
        pid_thread.start()
    def xcyc2xyxy(self, xc, yc):
        # screen_width,screen_height分别是屏幕宽度和高度
        # 截图框的中央在屏幕正中央，shot_Width,shot_Height分别是截图框的宽度和高度
        # xc,xy是目标在截图框中以0到1的浮点数表示的相对坐标
        # 返回目标点在屏幕上的绝对坐标targetRealX,targetRealY
        targetRealX = int(self.screen_width / 2 + float(xc) * self.shot_Width - self.shot_Width / 2)
        targetRealY = int(self.screen_height / 2 + float(yc) * self.shot_Height - self.shot_Height / 2)
        return targetRealX, targetRealY
    def lock(self, aims):
        global targetRealX, targetRealY, current_mouse_x, current_mouse_y
        current_mouse_x,current_mouse_y = self.mouse.position # update mouse position
        aims_copy = aims.copy()
        aims_copy = [x for x in aims_copy if x[0] == lock_tag]
        if(len(aims_copy) ==0):
            return
        dist_list = []
        for det in aims_copy:
            _, x_c, y_c, _, _ = det # tag, x_center, y_center, width, height
            x, y = self.xcyc2xyxy(x_c, y_c)
            dist = (x - current_mouse_x) ** 2 + (y - current_mouse_y) ** 2
            dist_list.append(dist)
        det = aims_copy[dist_list.index(min(dist_list))] # get the nearest target
        tag,target_x,target_y,target_width,target_height=det
        target_x, target_y, target_width, target_height = float(target_x), float(target_y), float(target_width), float(target_height)
        targetRealX, targetRealY = self.xcyc2xyxy(target_x, target_y + self.target_offset_y*target_height) # update target position
        # targetRealY += int(self.target_offset_y*self.shot_Height / 2 * float(target_height))
        dist = (targetRealX - current_mouse_x)**2 + (targetRealY - current_mouse_y)**2
        if dist < 20000:
            self.dist = True
        else:
            self.dist = False
    def set_lock_state(self, state):
        global lock_state
        with lock:
            lock_state = state
    def set_exit_flag(self, flag):
        global pid_exit_flag
        with lock:
            pid_exit_flag = flag