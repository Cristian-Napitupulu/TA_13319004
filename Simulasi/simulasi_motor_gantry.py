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
L1 = 0.00120
R1 = 10.0
J1 = 0.00200
b1 = 0.004
rp1 = 0.04
Ke1 = 0.05
Kt1 = 0.05
K1 = 1 / (Kt1 * rp1)

# Motor 2
L2 = 0.0015
R2 = 1.0
J2 = 0.00025
b2 = 0.0001
rp2 = 0.01
Ke2 = 0.05
Kt2 = 0.05
K2 = 1 / (Kt2 * rp2)

control_limit = 24  # Volt

# Control Parameter
# Parameter for theta
lambda1 = 15.0
lambda2 = 0.0
matrix_lambda = np.matrix([[lambda1], [lambda2]])

# Parameter for x and l
alpha1 = 0.5
alpha2 = 1.2
matrix_alpha = np.matrix([[alpha1, 0.0], [0.0, alpha2]])

# Parameter for x_dot and l_dot
beta1 = 2.0
beta2 = 5.0
matrix_beta = np.matrix([[beta1, 0.0], [0.0, beta2]])

# K must be > 0
k1 = 0.01
k2 = 0.02
k = [[k1], [k2]]
# k = 0.0005

# print("matrix lambda: \n", matrix_lambda)
# print("matrix alpha: \n", matrix_alpha)
# print("matrix beta: \n", matrix_beta)
# print("k: \n", k)

# Simulation Parameter
dt = 0.001
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
        L1 * J1 / (Kt1 * rp1) + L1 * rp1 * (mt + mc * np.sin(theta[i]) ** 2) / Kt1
    )
    matrix_A[0, 1] = -L1 * rp1 * mc * np.sin(theta[i]) / Kt1
    matrix_A[1, 0] = -L2 * rp2 * mc * np.sin(theta[i]) / Kt2
    matrix_A[1, 1] = L2 * J2 / (Kt2 * rp2) + L2 * rp2 * mc / Kt2

    # Update matrix B
    matrix_B[0, 0] = (
        (L1 * b1 + R1 * J1) / (Kt1 * rp1)
        + L1 * rp1 * mc * np.sin(2*theta[i]) * theta_dot[i] / Kt1
        + L1 * rp1 * bt / Kt1
        + R1 * rp1 * (mt + mc * np.sin(theta[i]) ** 2) / Kt1
    )
    matrix_B[0, 1] = (
        -L1 * rp1 * mc * np.cos(theta[i]) * theta_dot[i] / Kt1
        - R1 * rp1 * mc * np.sin(theta[i]) / Kt1
    )
    matrix_B[1, 0] = (
        -L2 * rp2 * mc * np.cos(theta[i]) * theta_dot[i] / Kt2
        - R2 * rp2 * mc * np.sin(theta[i]) / Kt2
    )
    matrix_B[1, 1] = (
        (L2 * b2 + R2 * J2) / (Kt2 * rp2) + L2 * rp2 * br / Kt2 + R2 * rp2 * mc / Kt2
    )

    # Update matrix C
    matrix_C[0, 0] = R1 * b1 / (Kt1 * rp1) + Ke1 / rp1 + R1 * rp1 * bt / Kt1
    matrix_C[1, 1] = R2 * b2 / (Kt2 * rp2) + Ke2 / rp2 + R2 * rp2 * br / Kt2

    # Update matrix D
    matrix_D[0, 0] = 2 * L1 * rp1 * mc * l[i] * np.sin(theta[i]) * theta_dot[i] / Kt1
    matrix_D[1, 0] = -2 * L2 * rp2 * mc * l[i] * theta_dot[i] / Kt2

    # Update matrix E
    matrix_E[0, 0] = (
        L1 * rp1 * mc * l[i] * np.cos(theta[i]) * theta_dot[i] ** 2 / Kt1
        + L1 * rp1 * mc * np.sin(theta[i]) * l_dot[i] * theta_dot[i] / Kt1
        + R1 * rp1 * mc * l[i] * np.sin(theta[i]) * theta_dot[i] / Kt1
        + L1 * rp1 * mc * g * np.cos(2* theta[i]) ** 2 / Kt1
    )

    matrix_E[1, 0] = (
        -L2 * rp2 * mc * l_dot[i] * theta_dot[i] / Kt2
        - R2 * rp2 * mc * l[i] * theta_dot[i] / Kt2
        + L2 * rp2 * mc * g * np.sin(theta[i]) / Kt2
    )

    # Update matrix F
    matrix_F[0, 0] = R1 * rp1 * mc * g * np.sin(theta[i]) * np.cos(theta[i]) / Kt1
    matrix_F[1, 0] = -R2 * rp2 * mc * g * np.cos(theta[i]) / Kt2

    q_now = np.matrix([[x[i]], [l[i]]])
    q_dot_now = np.matrix([[x_dot[i]], [l_dot[i]]])
    q_dot_dot_now = np.matrix([[x_dot_dot[i]], [l_dot_dot[i]]])
    q_desired = np.matrix([[y_desired[0, 0]], [y_desired[1, 0]]])

    sliding_surface_now = np.matrix([[Sx[i]], [Sl[i]]])
    sliding_surface_now = (
        np.matmul(matrix_alpha, (q_now - q_desired))
        + np.matmul(matrix_beta, q_dot_now)
        + q_dot_dot_now
        + matrix_lambda * theta[i]
    )

    control_now = np.matrix([[0], [0]])
    control_now = (
        np.matmul((matrix_B - np.matmul(matrix_A, matrix_beta)), q_dot_dot_now)
        + np.matmul((matrix_C - np.matmul(matrix_A, matrix_alpha)), q_dot_now)
        + matrix_D * theta_dot_dot[i]
        + (matrix_E - np.matmul(matrix_A, matrix_lambda)) * theta_dot[i]
        + matrix_F
        - k * sign_matrix(sliding_surface_now)
    )

    control_now = np.clip(control_now, -control_limit, control_limit)

    q_triple_dot_now = np.matmul(
        np.linalg.inv(matrix_A),
        (
            control_now
            - (
                matrix_B * q_dot_dot_now
                + matrix_C * q_dot_now
                + matrix_D * theta_dot_dot[i]
                + matrix_E * theta[i]
                + matrix_F
            )
        ),
    )

    x_dot_dot.append(x_dot_dot[i] + q_triple_dot_now[0, 0] * dt)
    l_dot_dot.append(l_dot_dot[i] + q_triple_dot_now[1, 0] * dt)
    theta_dot_dot_temp_ = (
        np.cos(theta[i]) * x_dot_dot[i]
        - 2 * l_dot[i] * theta_dot[i]
        - np.sin(theta[i]) * g
    ) / l[i]
    theta_dot_dot.append(theta_dot_dot_temp_)
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

    # print("q_now: \n", q_now)
    # print("q_desired: \n", q_desired)

    # print("control_now: \n", control_now)

    # print("Inversed matrix A: \n", np.linalg.inv(matrix_A))
    # print("q_triple_dot_now: \n", q_triple_dot_now)

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
    if (
        abs(x[i] - y_desired[0, 0]) > 2**16
        or abs(l[i] - y_desired[1, 0]) > 2**16
        or abs(theta[i] - abs(y_desired[2, 0])) > 2**16
    ):
        print("Simulation Failed! Divergence detected!")
        show_result = False
        break

if show_result:
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
    plt.savefig(plot_folder_path + "x vs time.svg", format='svg', transparent=True)

    plt.figure(2)
    plt.plot(time, l_dot, "g--", label="l_dot (m/s)", alpha=0.5)
    plt.plot(time, l, "b", label="l (m)")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("l")
    plt.title("l vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "l vs time.svg", format='svg', transparent=True)

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
    plt.savefig(plot_folder_path + "theta vs time.svg", format='svg', transparent=True)

    plt.figure(4)
    plt.plot(time, Ux, "r", label="Ux (volt)")
    plt.plot(time, Sx, "b--", label="Sx")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("Ux Response")
    plt.title("Ux vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "Ux vs time.svg", format='svg', transparent=True)

    plt.figure(5)
    plt.plot(time, Ul, "r", label="Uy (volt)")
    plt.plot(time, Sl, "b--", label="Sl")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("Ul Response")
    plt.title("Ul vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "Ul vs time.svg", format='svg', transparent=True)

    plt.figure(6)
    plt.plot(time, x, "r", label="x (m)")
    plt.plot(time, l, "b", label="l (m)")
    plt.plot(time, theta, "g", label="theta (degree)")
    plt.legend(loc="upper right")
    plt.xlabel("time (s)")
    plt.ylabel("State")
    plt.title("State vs time")
    plt.grid(True)
    plt.savefig(plot_folder_path + "State vs time.svg", format='svg', transparent=True)

    # plt.show()
