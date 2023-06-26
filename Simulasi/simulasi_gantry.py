import numpy as np
import matplotlib.pyplot as plt

def moving_average_cumsum(data, window_size):
    cumsum = np.cumsum(data, dtype=float)
    cumsum[window_size:] = cumsum[window_size:] - cumsum[:-window_size]
    return cumsum[window_size - 1:] / window_size

class state:
    def __init__(self, dt_, duration_, x0=0.0, x_dot0=0.0):
        self.dt = dt_
        self.duration = duration_
        self._x = np.zeros(int(self.duration / self.dt))
        self._x_dot = np.zeros(int(self.duration / self.dt))
        self._x[0] = x0
        self._x_dot[0] = x_dot0


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
mc = 2
mt = 2
bt = 2
br = 2
g = 9.81

# Control Parameter
# K must be > 0
k = 1.0
lambda1 = 0.75
lambda2 = 0.7
alpha1 = 1.5
alpha2 = 0.0

matrix_lambda = np.matrix([[lambda1, 0.0, 0.0], [0.0, lambda2, 0.0]])
matrix_alpha = np.matrix([[alpha1], [alpha2]])
matrix_I = np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

# Simulation Parameter
dt = 0.005
duration = 80.0
steady_state_checking_duration_window = 1.0

y_initial = np.matrix([[0.0], [6.0], [0.0]])
y_desired = np.matrix([[5.0], [1.0], [0.0]])

x = state(dt, duration, x0=y_initial[0,0])
x_dot = state(dt, duration)
l = state(dt, duration, x0=y_initial[1,0])
l_dot = state(dt, duration)
theta = state(dt, duration, x0=y_initial[2,0])
theta_dot = state(dt, duration)

Fx = state(dt, duration)
Fl = state(dt, duration)
control_limit = 2 * mc * g 

sliding_surface1 = state(dt, duration)
sliding_surface2 = state(dt, duration)

time = np.arange(0, duration, dt)

all_sliding_surface = [sliding_surface1, sliding_surface2]
all_state = [x, x_dot, l, l_dot, theta, theta_dot]
all_control = [Fx, Fl]

matrix_A = np.matrix(
    [
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
)

matrix_B = np.matrix(
    [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
)

matrix_G = np.matrix([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])


def update_Matrix_A(mc, mt, bt, br, l, l_dot, theta, theta_dot, matrix_A, j):
    matrix_A[1, 1] = -bt / mt
    matrix_A[1, 3] = -br * np.sin(theta._x[j]) / mt
    matrix_A[3, 1] = -bt * np.sin(theta._x[j]) / mt
    matrix_A[3, 3] = -(mc * (np.sin(theta._x[j])) ** 2 / mt + 1) * (br / mt)
    matrix_A[3, 5] = l._x[j] * theta_dot._x[j]
    matrix_A[5, 1] = -bt * np.cos(theta._x[j]) / (mt * l._x[j])
    matrix_A[5, 3] = -br * np.sin(theta._x[j]) * np.cos(theta._x[j]) / (mt * l._x[j])
    matrix_A[5, 5] = -2 * l_dot._x[j] / l._x[j]


def update_Matrix_B(mc, mt, l, theta, matrix_B, j):
    matrix_B[1, 0] = 1 / mt
    matrix_B[1, 1] = np.sin(theta._x[j]) / mt
    matrix_B[3, 0] = np.sin(theta._x[j]) / mt
    matrix_B[3, 1] = (mc * (np.sin(theta._x[j])) ** 2 / mt + 1) / mc
    matrix_B[5, 0] = np.cos(theta._x[j]) / (mt * l._x[j])
    matrix_B[5, 1] = np.sin(theta._x[j]) * np.cos(theta._x[j]) / (mt * l._x[j])


def update_Matrix_G(g, l, theta, matrix_G, j):
    matrix_G[1, 0] = 0.0
    matrix_G[3, 0] = g * np.cos(theta._x[j])
    matrix_G[5, 0] = -g * np.sin(theta._x[j]) / l._x[j]


for j in range(int(duration / dt) - 1):
    # Update Matrix A
    update_Matrix_A(mc, mt, bt, br, l, l_dot, theta, theta_dot, matrix_A, j)

    # Update Matrix B
    update_Matrix_B(mc, mt, l, theta, matrix_B, j)

    # Update Matrix G
    update_Matrix_G(g, l, theta, matrix_G, j)

    # Matrix A, B, G cap
    matrix_A_cap = np.matrix(
        [
            [matrix_A[1, 1], matrix_A[1, 3], matrix_A[1, 5]],
            [matrix_A[3, 1], matrix_A[3, 3], matrix_A[3, 5]],
        ]
    )

    matrix_B_cap = np.matrix(
        [[matrix_B[1, 0], matrix_B[1, 1]], [matrix_B[3, 0], matrix_B[3, 1]]]
    )

    matrix_G_cap = np.matrix([[matrix_G[1, 0]], [matrix_G[3, 0]]])

    y = np.matrix([[x._x[j]], [l._x[j]], [theta._x[j]]])
    y_dot = np.matrix([[x_dot._x[j]], [l_dot._x[j]], [theta_dot._x[j]]])

    # Create dummy matrix for sliding surface
    sliding_surface_now = (
        matrix_lambda * (y_desired - y) - matrix_I * y_dot - matrix_alpha * theta._x[j]
    )

    # Create dummy matrix for control input
    control_now = np.matrix([[Fx._x[j]], [Fl._x[j]]])
    control_now = np.linalg.inv(matrix_B_cap) * (
        -np.matmul((matrix_A_cap + matrix_lambda), y_dot)
        - matrix_G_cap
        - matrix_alpha * theta_dot._x[j]
    ) + k * sign_matrix(sliding_surface_now)
    control_now = np.clip(control_now, -control_limit, control_limit)

    # Create dummy matrix for state
    state_now = np.matrix(
        [
            [x._x[j]],
            [x_dot._x[j]],
            [l._x[j]],
            [l_dot._x[j]],
            [theta._x[j]],
            [theta_dot._x[j]],
        ]
    )
    # print(state_now)

    # Update State
    state_now = state_now + dt * (
        matrix_A * state_now + matrix_B * control_now + matrix_G
    )

    # Update State
    x._x[j + 1] = state_now[0, 0]
    x_dot._x[j + 1] = state_now[1, 0]
    l._x[j + 1] = state_now[2, 0]
    l_dot._x[j + 1] = state_now[3, 0]
    theta._x[j + 1] = state_now[4, 0]
    theta_dot._x[j + 1] = state_now[5, 0]

    # Update Sliding Surface
    sliding_surface1._x[j + 1] = sliding_surface_now[0, 0]
    sliding_surface2._x[j + 1] = sliding_surface_now[1, 0]

    # Update Control
    Fx._x[j + 1] = control_now[0, 0]
    Fl._x[j + 1] = control_now[1, 0]

    if j % 50 == 0:
        print("Progress: ", round(j / (duration / dt) * 100, 1), "%", end="\r")

print("Simulation Completed!")

plot_folder_path = "Simulasi/Gambar/Simulasi Gantry/"

# Plotting
plt.figure(1)
plt.plot(time, x._x, "r", label="x (m)")
plt.plot(time, x_dot._x, "b--", label="x_dot (m/s)", alpha=0.8)
plt.legend(loc="upper right")
plt.xlabel("time (s)")
plt.ylabel("x")
plt.title("x vs time")
plt.grid(True)
plt.savefig(plot_folder_path + "x vs time.png")

plt.figure(2)
plt.plot(time, l._x, "b", label="l (m)")
plt.plot(time, l_dot._x, "g--", label="l_dot (m/s)", alpha=0.8)
plt.legend(loc="upper right")
plt.xlabel("time (s)")
plt.ylabel("l")
plt.title("l vs time")
plt.grid(True)
plt.savefig(plot_folder_path + "l vs time.png")

plt.figure(3)
plt.plot(time, theta._x * 180 / np.pi, "b", label="theta (degree)")
plt.plot(time, theta_dot._x  * 180 / np.pi, "g--", label="theta_dot (degree/s)", alpha=0.8)
plt.legend(loc="upper right")
plt.xlabel("time (s)")
plt.ylabel("theta")
plt.title("theta vs time")
plt.grid(True)
plt.savefig(plot_folder_path + "theta vs time.png")

plt.figure(4)
plt.plot(time, Fx._x, "r", label="Fx (N)")
plt.plot(time, sliding_surface1._x, "b--", label="sliding_surface1", alpha=0.8)
plt.legend(loc="upper right")
plt.xlabel("time (s)")
plt.ylabel("Control Parameter")
plt.title("Control Parameter vs time")
plt.grid(True)
plt.savefig(plot_folder_path + "Fx vs time.png")

plt.figure(5)
plt.plot(time, Fl._x, "b", label="Fy (N)")
plt.plot(time, sliding_surface2._x, "g--", label="sliding_surface2", alpha=0.8)
plt.legend(loc="upper right")
plt.xlabel("time (s)")
plt.ylabel("Control Parameter")
plt.title("Control Parameter vs time")
plt.grid(True)
plt.savefig(plot_folder_path + "Fl vs time.png")

plt.figure(6)
plt.plot(time, x._x, "r", label="x (m)")
plt.plot(time, l._x, "b", label="l (m)")
plt.plot(time, theta._x * 180 / np.pi, "g", label="theta (degree)")
plt.legend(loc="upper right")
plt.xlabel("time (s)")
plt.ylabel("State")
plt.title("State vs time")
plt.grid(True)
plt.savefig(plot_folder_path + "State vs time.png")


# plt.show()