import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Vehicle():
    def __init__(self):

        # Throttle to Engine Torque
        self._a0 = 400
        self._a1 = 0.1
        self._a2 = -0.0002

        # Gear Ratio, Effective Radius, Mass + Inertia
        self._GR = 0.35
        self._r_e = 0.3
        self._J_e = 10
        self._m = 2000
        self._g = 9.81

        # Aerodynamic and Friction Coefficient
        self._c_a = 1.36
        self._c_r1 = 0.01

        # Tire Force
        self._c = 10000
        self._F_max = 10000

        # State Variables
        self._x = 0
        self._v = 5
        self._a = 0
        self._w_e = 100
        self._w_e_dot = 0

        self.sample_time = 0.01

    def reset(self):
        self._x = 0
        self._v = 5
        self._a = 0
        self._w_e = 100
        self._w_e_dot = 0

    def step(self, throttle, alpha):
        T_e = throttle * (self._a0 + self._a1 * self._w_e + self._a2 * (self._w_e ** 2))
        F_aero = 1/2 * self._c_a * (self._v ** 2)
        R_x = self._c_r1 * self._v
        F_g = self._m * self._g * np.sin(alpha)
        F_load = F_aero + R_x + F_g

        w_w = self._GR * self._w_e
        s = (w_w * self._r_e - self._v) / self._v
        if abs(s) < 1:
            F_x = self._c * s
        else:
            F_x = self._F_max
        
        #Combined Engine Dynamic Equations
        self._a = (F_x - F_load) / self._m
        self._w_e_dot = (T_e - self._GR * self._r_e * F_load) / self._J_e

        #Update Equations
        self._w_e += self._w_e_dot * self.sample_time
        self._v += self._a * self.sample_time
        self._x += (self._v * self.sample_time) - (0.5 * self._a * (self.sample_time ** 2))

    def getV(self):
        return self._v
    
    def getX(self):
        return self._x
    
    def getW_E(self):
        return self._w_e
    
vehicle = Vehicle()

##########################
# Constant Throttle Test #
##########################
"""
vehicle.reset()
time_end = 100

t_data = np.arange(0, time_end, vehicle.sample_time)
v_data = np.zeros_like(t_data)

throttle = 0.2
alpha = 0

for i in range (t_data.shape[0]):
    v_data[i] = vehicle.getV()
    vehicle.step(throttle, alpha)

plt.plot(t_data, v_data)
plt.show()
"""
####################################
# Driving the Vehicle Over a Slope #
####################################

vehicle.reset()
time_end = 20
t_data = np.arange(0, time_end, vehicle.sample_time)
x_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
w_e_data = np.zeros_like(t_data)
throttle = np.zeros_like(t_data)
alpha = np.zeros_like(t_data)

def calcThrottle(currentTime):
    if currentTime < 5:
        start_time = 0
        m = 0.2
        c = (0.5 - 0.2) / (5 - 0)
    elif currentTime < 15:
        start_time = 5
        m = 0.5
        c = (0.5 - 0.5) / (15 - 5)
    else:
        start_time = 15
        m = 0.5
        c = (0 - 0.5) / (20 - 15)
    return (currentTime - start_time) * c + m

def calcAlpha (x):
    return (x < 60) * np.arctan(3/60) + (60 <= x and x < 150) * np.arctan(9 / 90)

for i in range (t_data.shape[0]):
    # First, we should find the value of alpha based on our position and the given chart
    alpha[i] = calcAlpha(vehicle.getX())

    # Now, we should find the value of throttle based on current time and the given chart
    throttle[i] = calcThrottle(t_data[i])

    vehicle.step(throttle[i], alpha[i])
    x_data[i] = vehicle.getX()
    v_data[i] = vehicle.getV()
    w_e_data[i] = vehicle.getW_E()


# plt.plot(alpha)
# plt.plot(throttle)

plt.title('Distance')
plt.plot(t_data, x_data)
plt.show()

plt.title('Velocity')
plt.plot(t_data, v_data)
plt.show()

plt.title('w_e')
plt.plot(t_data, w_e_data)
plt.show()

plt.title('throttle')
plt.plot(t_data, throttle)
plt.show()

plt.title('alpha')
plt.plot(t_data, alpha)
plt.show()