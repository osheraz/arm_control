import numpy as np

dx = 20
dy = dx
# Motor specs: [mm]
x = np.linspace(270, 339, dx)  # HDA50 Stroke
y = np.linspace(200, 260, dy)  # P16 Stroke
y_ = 197  # P16 Extracted length
x_ = 246  # HDA50 Extracted length
h = 140
L = 550
l4 = 80
l1 = 285
r = 237
l3 = 46
l2 = 293
lp = 206
l33 = 398
H = 330
l44 = 669.5
minPWM = 20



angle =  0 * np.pi /180
height = -100

PSI = np.arcsin((height - lp * np.sin(angle)) / (l44))
xp = l44 * np.cos(PSI) + lp * np.cos(angle)
alpha = np.arccos((l44 ** 2 + l1 ** 2 - l33 ** 2) / (2 * l44 * l1)) + PSI
PHI = 77.31 * np.pi / 180 - angle - abs(PSI)

x = (h ** 2 + l1 ** 2 + 2 * h * l1 * np.sin(alpha)) ** (1 / 2)
y = (r ** 2 + l3 ** 2 - 2 * r * l3 * np.cos(PHI)) ** (1 / 2)

x_mm = (x - x_)
y_mm = (y - y_)
x_cmd = x_mm * (1023 / 101)
y_cmd = y_mm * (1023 / 150)
