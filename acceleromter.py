'''
DESCRIPTION:
    The following code is a script to collect,
    process, and visualize accelerometer data
    from a MPU6050 sensor using a Kalman filter.
    
CODE DEPENDENCIES:
    smbus:      sudo apt install python3-smbus
    mpu6050:    pip install mpu6050-raspberrypi
    matplotlib: pip install matplotlib
    
PINOUT: (from MPU6050 to Raspberry Pi)
    VCC -> VCC (5V)
    GND -> GND
    SCL -> SCL (Pin5)
    SDA -> SDA (Pin3)
'''

from mpu6050 import mpu6050
import time
import matplotlib.pyplot as plt 
import matplotlib.animation as animation
import numpy as np

# Creating figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

# Software Specific Code
t = []
x_measured = []
y_measured = []
z_measured = []
x_kalman = []
y_kalman = []
z_kalman = []

SAMPLE_PERIOD = 0.5

# Kalman Filter Code
measure_variance_x = 0.05
measure_variance_y = 0.05
measure_variance_z = 0.05
process_variance_x = 0.125
process_variance_y = 0.125
process_variance_z = 0.125

X = np.matrix([[0], [0], [9.81]])
P = np.matrix([
               [process_variance_x**2, 0, 0],
               [0, process_variance_y**2, 0],
               [0, 0, process_variance_z**2],
              ])
A = np.identity(3)
A_T = np.transpose(A)
B = np.identity(3)
u = np.matrix([[0], [0], [0]])
w = np.matrix([[0], [0], [0]])
Q = np.matrix([[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]])
H = np.identity(3)
H_T = np.transpose(H)
R = np.matrix([
               [measure_variance_x**2, 0, 0],
               [0, measure_variance_y**2, 0],
               [0, 0, measure_variance_z**2],
              ])
I = np.identity(3)

# Initialize MPU6050 sensor
mpu = mpu6050(0x68)

# Here Y is a 3x1 matrix of the collected accelerometer data
def kalman(Y):
    # predict state and process matrix for current iteration
    X_predict = A*X + B*u + w
    P_predict = A*P*A_T + Q
    
    # find kalman gain
    K = P_predict*H_T*(np.linalg.inv(H*P_predict*H_T + R))

    # estimate new state and process matricies
    X_estimate = X_predict + K*(Y-H*X_predict)
    P_estimate = (I - K*H)*P
    return X_estimate, P_estimate


def animate(i, t, x_measured, y_measured, z_measured, x_kalman, y_kalman, z_kalman):
    global X, P
    
    # collects accelerometer data
    accel_data = mpu.get_accel_data()
    accel_x = accel_data['x']
    accel_y = accel_data['y']
    accel_z = accel_data['z']

    # processes accelerometer data to kalman filter
    Y = np.matrix([[accel_x],[accel_y],[accel_z]])
    X, P = kalman(Y)    
    kal_x = X[0,0]
    kal_y = X[1,0]
    kal_z = X[2,0]

    # appends acceleromter data to lists 
    x_measured.append(accel_x)
    y_measured.append(accel_y)
    z_measured.append(accel_z)
    x_kalman.append(kal_x)
    y_kalman.append(kal_y)
    z_kalman.append(kal_z)
    t.append(i*SAMPLE_PERIOD) 

    # limits each list to 20 items
    x_measured = x_measured[-20:]
    y_measured = y_measured[-20:]
    z_measured = z_measured[-20:]
    x_kalman = x_kalman[-20:]
    y_kalman = y_kalman[-20:]
    z_kalman = z_kalman[-20:]
    t = t[-20:]

    # graphs data
    ax.clear()
    ax.plot(t, x_measured, color='g', label='measured X')
    ax.plot(t, x_kalman, linestyle='dashed', color='g', label='kalman X')
    ax.plot(t, y_measured, color='r', label='measured Y')
    ax.plot(t, y_kalman, linestyle='dashed', color='r', label='kalman Y')
    ax.plot(t, z_measured, color='b', label='measured Z')
    ax.plot(t, z_kalman, linestyle='dashed', color='b', label='kalman Z')
    ax.legend(loc = "upper left")
    plt.title("Kalman Filter for Accelerometer on MPU6050")
    plt.ylabel("Acceleration (m/s^2)")
    plt.xlabel("Time (s)")

try:
    ani = animation.FuncAnimation(fig, animate, fargs=(t, x_measured, y_measured, z_measured, x_kalman, y_kalman, z_kalman), interval=int(SAMPLE_PERIOD*1000)) 
    plt.show()
except KeyboardInterrupt:
    print("\nEnd of Kalman Filter for MPU6050!")
    plt.close()
