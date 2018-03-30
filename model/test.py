import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure(figsize=(8, 8))
r = 5
points = 35
theta = np.linspace(0.01, np.pi - 0.01, points)  # 0.01 to avoid infinities/zeros
# generate points given thetas
x = np.sin(theta) * r
y = np.cos(theta) * r + r
# y = np.linspace(0.0001, 2 * r - 0.0001, points)
# x = np.sqrt(r**2 - (y - r)**2)

plt.plot(y, x)

c = x + (y - r) / x * y
# print(c)
m = -(y - r) / x

for i in range(points):
    point_x = x[i]
    point_y = y[i]
    _y = np.linspace(0, 2 * r - 0.0001, points)
    _x = m[i] * _y + c[i]

    plt.plot(_y, _x)

t_x = 12
t_y = 20
py1 = (r * t_y ** 2 - r ** 2 * t_y + t_x ** 2 * r - np.sqrt(r ** 2 * t_x ** 4 - 2 * t_y * r ** 3 * t_x ** 2 + t_y ** 2 * r ** 2 * t_x ** 2)) / (
            t_y ** 2 - 2 * r * t_y + t_x ** 2 + r ** 2)
py2 = (r * t_y ** 2 - r ** 2 * t_y + t_x ** 2 * r + np.sqrt(r ** 2 * t_x ** 4 - 2 * t_y * r ** 3 * t_x ** 2 + t_y ** 2 * r ** 2 * t_x ** 2)) / (
            t_y ** 2 - 2 * r * t_y + t_x ** 2 + r ** 2)

# py = (r ** 3 - np.sqrt(
#     r ** 2 * t_x ** 2 * r ** 2 - 2 * r * t_x ** 2 * r ** 2 * t_y + t_x ** 4 * r ** 2 - t_x ** 2 * r ** 4 + t_x ** 2 * r ** 2 * t_y ** 2) - 2 * r ** 2 * t_y + r * t_x ** 2 - r * r ** 2 + r * t_y ** 2 + r ** 2 * t_y) / (
#              r ** 2 - 2 * r * t_y + t_x ** 2 + t_y ** 2)
px1 = np.sqrt(r ** 2 - (py1 - r) ** 2)
px2 = np.sqrt(r ** 2 - (py2 - r) ** 2)

plt.plot(t_y, t_x, 'r.')
plt.plot(py1, px1, 'b.')
plt.plot(py2, px2, 'g.')
print((py1, px1), (py2, px2))

# plt.axis('equal')
plt.xlim((-20, 20))
plt.ylim((0, 40))
plt.show()
