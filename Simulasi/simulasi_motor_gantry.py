import numpy as np
import matplotlib.pyplot as plt
import math


def sign_matrix(X):
    Y = np.zeros(X.shape)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            if X[i, j] > 0:
                Y[i, j] = 1
            elif X[i, j] < 0:
                Y[i, j] = -1
            else:
                Y[i, j] = 0
    return Y


# Physical Parameter
mc = 5.0
mt = 5.0
bt = 10.0
br = 10.0
g = 9.81

# Motor 1
L1 = 0.0120
R1 = 15.0
J1 = 0.00200
b1 = 0.004
rp1 = 0.04
Ke1 = 0.2
Kt1 = 0.2
K1 = 1 / (Kt1 * rp1)

# Motor 2
L2 = 0.0015
R2 = 0.5
J2 = 0.00025
b2 = 0.0001
rp2 = 0.01
Ke2 = 0.05
Kt2 = 0.05
K2 = 1 / (Kt2 * rp2)

control_limit = 24 # Volt

# Control Parameter
# Parameter for theta
lambda1 = 15.0
lambda2 = 0.0
matrix_lambda = np.matrix([[lambda1], [lambda2]])

# Parameter for x and l
alpha1 = 0.30
alpha2 = 0.9
matrix_alpha = np.matrix([[alpha1, 0.0], [0.0, alpha2]])

# Parameter for x_dot and l_dot
beta1 = 1.50
beta2 = 5.0
matrix_beta = np.matrix([[beta1, 0.0], [0.0, beta2]])

# K must be > 0
k1 = 0.003
k2 = 0.00013
k = [[k1], [k2]]
# k = 0.0005

# print("matrix lambda: \n", matrix_lambda)
# print("matrix alpha: \n", matrix_alpha)
# print("matrix beta: \n", matrix_beta)
# print("k: \n", k)

# Simulation Parameter
dt = 0.005
timeout_duration = 60.0
steady_state_checking_duration_window = 1.0



y_desired = np.matrix([[1.0], [0.5], [0.0]])
y_initial = np.matrix([[0.0], [1.5], [0.0]])

x = [y_initial[0, 0]]
x_dot = [0.0]
x_dot_dot = [0.0]
l = [y_initial[1, 0]]
l_dot = [0.0]
l_dot_dot = [0.0]
theta = [y_initial[2, 0]]
theta_dot = [0.0]
theta_dot_dot = [0.0]

Ux = [0.0]
Ul = [0.0]

Sx = [0.0]
Sl = [0.0]


matrix_A = np.matrix([[0.0, 0.0], [0.0, 0.0]])
matrix_B = np.matrix([[0.0, 0.0], [0.0, 0.0]])
matrix_C = np.matrix([[0.0, 0.0], [0.0, 0.0]])
matrix_D = np.matrix([[0.0], [0.0]])
matrix_E = np.matrix([[0.0], [0.0]])
matrix_F = np.matrix([[0.0], [0.0]])

show_result = True
i = 0
while i < int(timeout_duration / dt):
    # print("i: ", i)
    # Update matrix A
    matrix_A[0, 0] = (
        -K1 * L1 * (rp1**2 * np.cos(theta[i]) ** 2 * mc - (mc + mt) * rp1**2 - J1)
    )
    matrix_A[0, 1] = -K1 * L1 * rp1**2 * mc * np.sin(theta[i])
    matrix_A[1, 0] = -K2 * L2 * rp2**2 * mc * np.sin(theta[i])
    matrix_A[1, 1] = K2 * L2 * (mc * rp2**2 + J2)

    # Update matrix B
    matrix_B[0, 0] = K1 * (
        2 * rp1**2 * np.cos(theta[i]) * np.sin(theta[i]) * L1 * mc * theta_dot[i]
        - rp1**2 * np.cos(theta[i])**2  * R1 * mc
        + (L1 * b1 + R1 * mc + R1 * mt) * rp1**2
        + R1 * J1
        + L1 * b1
    )
    matrix_B[0, 1] = (
        -K1
        * rp1**2
        * mc
        * (np.cos(theta[i]) * L1 * theta_dot[i] + np.sin(theta[i]) * R1)
    )
    matrix_B[1, 0] = (
        -K2
        * rp2**2
        * mc
        * (np.cos(theta[i]) * L2 * theta_dot[i] + np.sin(theta[i]) * R2)
    )
    matrix_B[1, 1] = K2 * (rp2**2 * (L2 * br + R2 * mc) + R2 * J2 + L2 * b2)

    # Update matrix C
    matrix_C[0, 0] = K1 * (R1 * bt * rp1**2 + Ke1 * Kt1 + R1 * b1)
    matrix_C[1, 1] = K2 * (R2 * br * rp2**2 + Ke2 * Kt2 + K2 * b2)

    # Update matrix D
    matrix_D[0, 0] = (
        K1 * 2 * L1 * rp1**2 * mc * np.sin(theta[i]) * l[i] * theta_dot[i]
    )
    matrix_D[1, 0] = -K2 * 2 * L2 * rp2**2 * mc * l[i] * theta_dot[i]

    # Update matrix E
    matrix_E[0, 0] = K1 * (
        L1 * rp1**2 * mc * np.cos(theta[i]) * l[i] * theta_dot[i] ** 2
        + rp1**2 * np.sin(theta[i]) * mc * (L1 * l_dot[i] + R1 * l[i]) * theta_dot[i]
        + 2 * L1 * rp1**2 * (np.cos(theta[i]) ** 2 - 0.5) * mc * g
    )

    matrix_E[1, 0] = K2 * (
        -(rp2**2) * mc * (L2 * l_dot[i] + R2 * l[i]) * theta_dot[i]
        + L2 * rp2**2 * mc * np.sin(theta[i]) * g
    )

    # Update matrix F
    matrix_F[0, 0] = K1 * np.cos(theta[i]) * np.sin(theta[i]) * R1 * mc * rp1**2 * g
    matrix_F[1, 0] = -K2 * np.cos(theta[i]) * R2 * mc * rp2**2 * g

    

    y_now = np.matrix([[x[i]], [l[i]]])
    y_dot_now = np.matrix([[x_dot[i]], [l_dot[i]]])
    y_dot_dot_now = np.matrix([[x_dot_dot[i]], [l_dot_dot[i]]])
    y_target = np.matrix([[y_desired[0, 0]], [y_desired[1, 0]]])
    

    sliding_surface_now = np.matrix([[Sx[i]], [Sl[i]]])
    sliding_surface_now = (
        matrix_lambda * theta[i]
        + np.matmul(matrix_alpha, (y_now - y_target))
        + np.matmul(matrix_beta, y_dot_now)
        + y_dot_dot_now
    )
    
    control_now = np.matrix([[0], [0]])
    control_now = (
        np.matmul((matrix_B - np.matmul(matrix_A, matrix_beta)), y_dot_dot_now)
        + np.matmul((matrix_C - np.matmul(matrix_A, matrix_alpha)), y_dot_now)
        + matrix_D * theta_dot_dot[i]
        + (matrix_E - np.matmul(matrix_A, matrix_lambda)) * theta_dot[i]
        + matrix_F
    ) - k * sign_matrix(sliding_surface_now)
    # print(-k@sign_matrix(sliding_surface_now))

    control_now = np.clip(control_now, -control_limit, control_limit)
    

    y_triple_dot_now = np.matmul(np.linalg.inv(matrix_A), (
        control_now
        - matrix_B * y_dot_dot_now
        - matrix_C * y_dot_now
        - matrix_D * theta_dot_dot[i]
        - matrix_E * theta[i]
        - matrix_F
    ))
    

    x_dot_dot.append(x_dot_dot[i] + y_triple_dot_now[0, 0] * dt)
    l_dot_dot.append(l_dot_dot[i] + y_triple_dot_now[1, 0] * dt)
    temp_ = (
        np.cos(theta[i]) * x_dot_dot[i]
        - 2 * l_dot[i] * theta_dot[i]
        - np.sin(theta[i]) * g
    ) / l[i]
    theta_dot_dot.append(temp_)
    x_dot.append(x_dot[i] + x_dot_dot[i + 1] * dt)
    l_dot.append(l_dot[i] + l_dot_dot[i + 1] * dt)
    theta_dot.append(theta_dot[i] + theta_dot_dot[i + 1] * dt)
    x.append(x[i] + x_dot[i + 1] * dt)
    l.append(l[i] + l_dot[i + 1] * dt)
    theta.append(theta[i] + theta_dot[i + 1] * dt)

    Sx.append(sliding_surface_now[0, 0])
    Sl.append(sliding_surface_now[1, 0])

    Ux.append(control_now[0, 0])
    Ul.append(control_now[1, 0])

    # print("matrix A: \n", matrix_A)
    # print("matrix B: \n", matrix_B)
    # print("matrix C: \n", matrix_C)
    # print("matrix D: \n", matrix_D)
    # print("matrix E: \n", matrix_E)
    # print("matrix F: \n", matrix_F)

    # print("y_now: \n", y_now)
    # print("y_target: \n", y_target)

    # print("control_now: \n", control_now)

    # print("Inversed matrix A: \n", np.linalg.inv(matrix_A))
    # print("y_triple_dot_now: \n", y_triple_dot_now)

    # print("x: ", x[i+1])
    # print("l: ", l[i+1])
    # print("theta: ", theta[i+1])
    # print("x_dot: ", x_dot[i+1])
    # print("l_dot: ", l_dot[i+1])
    # print("theta_dot: ", theta_dot[i+1])
    # print("x_dot_dot: ", x_dot_dot[i+1])
    # print("l_dot_dot: ", l_dot_dot[i+1])
    # print("theta_dot_dot: ", theta_dot_dot[i+1])

    # print("Sx: ", Sx[i+1])
    # print("Sl: ", Sl[i+1])

    # print("Ux: ", Ux[i+1])
    # print("Ul: ", Ul[i+1])

    

    if i % 100 == 0:
        print("Progress: ", round(i / (timeout_duration / dt) * 100, 1), "%", end="\r")

    i += 1
    if (abs(x[i]-y_desired[0,0]) > 2**16 or abs(l[i]-y_desired[1,0]) > 2**16 or abs(theta[i]-abs(y_desired[2,0])) > 2**16):
        print("Simulation Failed! Divergence detected!")
        show_result = False
        break

if (show_result):
    print("Simulation Completed!")

    time = np.arange(0, timeout_duration + dt, dt)
    # print(len(x))
    # print(len(time))

    # print(theta_dot_dot)

    plot_folder_path = "Simulasi/Gambar/Simulasi Gantry Motor/"

    # Plotting
    plt.figure("X Displacement")
    plt.plot(time, x_dot, "b--", label="x_dot (m/s)", alpha=0.5)
    plt.plot(time, x, "r", label="x (m)")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("x")
    plt.title("x vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "x vs time.png")

    plt.figure(2)
    plt.plot(time, l_dot, "g--", label="l_dot (m/s)", alpha=0.5)
    plt.plot(time, l, "b", label="l (m)")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("l")
    plt.title("l vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "l vs time.png")

    theta = [math.degrees(i) for i in theta]
    theta_dot = [math.degrees(i) for i in theta_dot]
    plt.figure(3)
    plt.plot(time, theta_dot, "r--", label="theta_dot (degree/s)", alpha=0.2)
    plt.plot(time, theta, "g", label="theta (degree)")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("theta")
    plt.title("theta vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "theta vs time.png")

    plt.figure(4)
    plt.plot(time, Ux, "r", label="Ux (volt)")
    plt.plot(time, Sx, "b--", label="Sx")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("Ux Response")
    plt.title("Ux vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "Ux vs time.png")

    plt.figure(5)
    plt.plot(time, Ul, "r", label="Uy (volt)")
    plt.plot(time, Sl, "b--", label="Sl")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("Ul Response")
    plt.title("Ul vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "Ul vs time.png")

    plt.figure(6)
    plt.plot(time, x, "r", label="x (m)")
    plt.plot(time, l, "b", label="l (m)")
    plt.plot(time, theta, "g", label="theta (degree)")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("State")
    plt.title("State vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "State vs time.png")


    # plt.show()
