import pynput.mouse
import win32con,win32gui
import time
import win32api


lock_mode = False  #don's edit this
def on_click(x, y, button, pressed):
    global lock_mode, isX2Down
    if button == button.x2: # 使用鼠标上面一个侧键切换锁定模式，需要在apex设置中调整按键避免冲突
        isX2Down = pressed
        lock_mode = pressed
        print(f'isX2Down: {isX2Down}')
        
# ...or, in a non-blocking fashion:
listener = pynput.mouse.Listener(
    on_click=on_click)
listener.start()

while True:
    if lock_mode:
        output_x = 10
        output_y = 10
        print(f"鼠标移动 x:{output_x} y:{output_y}")
        win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, int(output_x), int(output_y))
        time.sleep(0.02)