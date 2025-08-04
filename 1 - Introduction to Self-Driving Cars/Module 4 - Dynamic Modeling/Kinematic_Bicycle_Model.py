import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Bicycle():
    max_w = 1.2     #Maximum Turning Rate (rad/s)
    L = 2           #Wheelbase length (m)
    lr = 1.2        #From the rear axle to center of mass (m)

    def __init__(self):
        self._xc = 0
        self._yc = 0
        self._theta = 0
        self._delta = 0
        self._beta = 0

    def reset(self):
        self._xc = 0
        self._yc = 0
        self._theta = 0
        self._delta = 0
        self._beta = 0

    def step(self, v, w):
        # Ensuring that maximum turning rate is not exceeded
        if w < 0:
            w = max (w, -1 * Bicycle.max_w)
        else:
            w = min (w, Bicycle.max_w)

        sample_time = 0.01

        xc_dot = v * np.cos(self._theta + self._beta)
        yc_dot = v * np.sin(self._theta + self._beta)
        theta_dot = 1 / Bicycle.L * v * np.cos(self._beta) * np.tan(self._delta)
        delta_dot = w

        self._xc += xc_dot * sample_time
        self._yc += yc_dot * sample_time
        self._theta += theta_dot * sample_time
        self._delta += delta_dot * sample_time
        self._beta = np.arctan(self.lr * np.tan(self._delta) / self.L)

    def setRadius(self, r):
        # delta = tan^-1(L/r)
        self._delta = np.arctan(Bicycle.L / r)

    def getPosition(self):
        pos = {"xc": self._xc, "yc": self._yc}
        return pos



sample_time = 0.01
time_end = 20 #seconds
bicycle = Bicycle()


# We want the model to traval a circle
# of radious 10m in 20 seconds
bicycle.setRadius(10)

t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)

for i in range (t_data.shape[0]):
    pos = bicycle.getPosition()
    x_data[i] = pos["xc"]
    y_data[i] = pos["yc"]
    bicycle.step(np.pi, 0)

plt.axis('equal')
plt.plot(x_data, y_data,label='My Model')
plt.legend()
plt.show()