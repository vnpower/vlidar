import math
import pyautogui

# Di chuyển chuột đến vị trí (x=500, y=300)
pyautogui.moveTo(1920+500, 300)

# Click chuột trái tại vị trí đó
pyautogui.click()

print(math.sin(340 * math.pi / 180.0 ))