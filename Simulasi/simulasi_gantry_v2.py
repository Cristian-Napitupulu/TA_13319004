import numpy as np
import matplotlib.pyplot as plt


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

    cumsum = np.cumsum(data, dtype=float)
    cumsum[window_size:] = cumsum[window_size:] - cumsum[:-window_size]
    return cumsum[window_size - 1 :] / window_size


def find_settling_index(data, target, threshold):
    for i in range(len(data) - 1, 0, -1):
        if np.abs(data[i] - target) < threshold:
            if np.abs(data[i] - target) >= threshold:
                return i
    return np.nan


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


def update_Matrix_A(mc, mt, bt, br, l, l_dot, theta, theta_dot, matrix_A):
    matrix_A[1, 1] = -bt / mt
    matrix_A[1, 3] = -br * np.sin(theta) / mt
    matrix_A[3, 1] = -bt * np.sin(theta) / mt
    matrix_A[3, 3] = -(mc * (np.sin(theta)) ** 2 / mt + 1) * (br / mt)
    matrix_A[3, 5] = l * theta_dot
    matrix_A[5, 1] = -bt * np.cos(theta) / (mt * l)
    matrix_A[5, 3] = -br * np.sin(theta) * np.cos(theta) / (mt * l)
    matrix_A[5, 5] = -2 * l_dot / l


def update_Matrix_B(mc, mt, l, theta, matrix_B):
    matrix_B[1, 0] = 1 / mt
    matrix_B[1, 1] = np.sin(theta) / mt
    matrix_B[3, 0] = np.sin(theta) / mt
    matrix_B[3, 1] = (mc * (np.sin(theta)) ** 2 / mt + 1) / mc
    matrix_B[5, 0] = np.cos(theta) / (mt * l)
    matrix_B[5, 1] = np.sin(theta) * np.cos(theta) / (mt * l)


def update_Matrix_G(g, l, theta, matrix_G):
    matrix_G[1, 0] = 0.0
    matrix_G[3, 0] = g * np.cos(theta)
    matrix_G[5, 0] = -g * np.sin(theta) / l


def matrix_Lambda(lambda1, lambda2):
    matrix_Lambda = np.matrix([[lambda1, 0.0, 0.0], [0.0, lambda2, 0.0]])
    return matrix_Lambda


def matrix_Alpha(alpha1, alpha2):
    matrix_alpha = np.matrix([[alpha1], [alpha2]])
    return matrix_alpha


# Physical Parameter
mc = 2
mt = 2
bt = 2
br = 2
g = 9.81

# Control Parameter
k = np.arange(0.01, 1.0, 0.01)
lambda1 = np.arange(0.01, 1.0, 0.01)
lambda2 = np.arange(0.01, 1.0, 0.01)
alpha1 = np.arange(0.5, 10, 0.5)
alpha2 = 0
# Simulation Parameter
dt = 0.005

y_desired = np.matrix([[5.0], [1.0], [0.0]])
y_initial = np.matrix([[0.0], [10.0], [0.0]])

control_limit = 2 * mc * g

x_max_error = 0.01
x_dot_max_steady_state = 0.01

l_max_error = 0.01
l_dot_max_steady_state = 0.01

theta_max_error = 0.01
theta_dot_max_steady_state = 0.01

steady_state_checking_duration_window = 1.0
timeout_duration = 180.0

# Searching Result = [[theta_max, theta_settling_time, x_settling_time, l_settling_time, lambda1, k, lambda2, alpha1]]
searching_result = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0]]

i = 0
for alpha1_ in alpha1:
    for lambda2_ in lambda2:
        for k_ in k:
            for lambda1_ in lambda1:
                i += 1
                print(
                    "Iteration:",
                    i,
                    "out of",
                    len(alpha1) * len(lambda2) * len(k) * len(lambda1),
                )
                print(
                    "lambda 1:",
                    lambda1_,
                    ";  k:",
                    k_,
                    ";  lambda 2:",
                    lambda2_,
                    ";  alpha 1:",
                    alpha1_,
                )
                matrix_lambda = matrix_Lambda(lambda1_, lambda2_)
                matrix_alpha = matrix_Alpha(alpha1_, alpha2)
                matrix_I = np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

                x = [y_initial[0, 0]]
                x_dot = [0.0]
                l = [y_initial[1, 0]]
                l_dot = [0.0]
                theta = [y_initial[2, 0]]
                theta_dot = [0.0]

                Fx = [0.0]
                Fl = [0.0]

                Sx = [0.0]
                Sl = [0.0]

                x_steady_state = False
                l_steady_state = False
                theta_steady_state = False

                j = 0
                while (
                    (not x_steady_state) or (not l_steady_state) or (theta_steady_state)
                ) and (j * dt < timeout_duration):
                    print("Simulation Elapsed:", round(j * dt, 1), "s", end="\r")
                    update_Matrix_A(
                        mc, mt, bt, br, l[j], l_dot[j], theta[j], theta_dot[j], matrix_A
                    )
                    update_Matrix_B(mc, mt, l[j], theta[j], matrix_B)
                    update_Matrix_G(g, l[j], theta[j], matrix_G)
                    matrix_A_cap = np.matrix(
                        [
                            [matrix_A[1, 1], matrix_A[1, 3], matrix_A[1, 5]],
                            [matrix_A[3, 1], matrix_A[3, 3], matrix_A[3, 5]],
                        ]
                    )

                    matrix_B_cap = np.matrix(
                        [
                            [matrix_B[1, 0], matrix_B[1, 1]],
                            [matrix_B[3, 0], matrix_B[3, 1]],
                        ]
                    )

                    matrix_G_cap = np.matrix([[matrix_G[1, 0]], [matrix_G[3, 0]]])

                    y = np.matrix([[x[j]], [l[j]], [theta[j]]])
                    y_dot = np.matrix([[x_dot[j]], [l_dot[j]], [theta_dot[j]]])

                    sliding_surface_now = np.matrix([[Sx[j]], [Sl[j]]])
                    sliding_surface_now = (
                        matrix_lambda * (y_desired - y)
                        - matrix_I * y_dot
                        - matrix_alpha * theta[j]
                    )

                    control_now = np.matrix([[Fx[j]], [Fl[j]]])
                    control_now = np.linalg.inv(matrix_B_cap) * (
                        -np.matmul((matrix_A_cap + matrix_lambda), y_dot)
                        - matrix_G_cap
                        - matrix_alpha * theta_dot[j]
                    ) + k * sign_matrix(sliding_surface_now)
                    control_now = np.clip(control_now, -control_limit, control_limit)

                    state_now = np.matrix(
                        [
                            [x[j]],
                            [x_dot[j]],
                            [l[j]],
                            [l_dot[j]],
                            [theta[j]],
                            [theta_dot[j]],
                        ]
                    )
                    state_now = state_now + dt * (
                        matrix_A * state_now + matrix_B * control_now + matrix_G
                    )
                    x.append(state_now[0, 0])
                    x_dot.append(state_now[1, 0])
                    l.append(state_now[2, 0])
                    l_dot.append(state_now[3, 0])
                    theta.append(state_now[4, 0])
                    theta_dot.append(state_now[5, 0])

                    Fx.append(control_now[0, 0])
                    Fl.append(control_now[1, 0])

                    Sx.append(sliding_surface_now[0, 0])
                    Sl.append(sliding_surface_now[1, 0])

                    if (
                        (j > steady_state_checking_duration_window / dt)
                        and (not x_steady_state)
                        and (abs(x[j] - y_desired[0, 0]) < x_max_error)
                        and (abs(x_dot[j]) < x_dot_max_steady_state)
                    ):
                        x_steady_state = True

                    if (
                        (j > steady_state_checking_duration_window / dt)
                        and (not l_steady_state)
                        and (abs(l[j] - y_desired[1, 0]) < l_max_error)
                        and (abs(l_dot[j]) < l_dot_max_steady_state)
                    ):
                        l_steady_state = True

                    if (
                        (j > steady_state_checking_duration_window / dt)
                        and (not theta_steady_state)
                        and (abs(theta[j] - y_desired[2, 0]) < theta_max_error)
                        and (abs(theta_dot[j]) < theta_dot_max_steady_state)
                    ):
                        theta_steady_state = True

                    j += 1

                x_settling_time = (
                    find_settling_index(x, y_desired[0, 0], x_max_error) * dt
                )
                l_settling_time = (
                    find_settling_index(l, y_desired[1, 0], l_max_error) * dt
                )
                theta_settling_time = (
                    find_settling_index(theta, y_desired[2, 0], theta_max_error) * dt
                )

                searching_result.append(
                    [
                        np.max(np.abs(theta))*180/np.pi,
                        theta_settling_time,
                        x_settling_time,
                        l_settling_time,
                        lambda1_,
                        k_,
                        lambda2_,
                        alpha1_,
                    ]
                )
                print("")
                print("Result: ")
                print(
                    "Max. Theta(deg) | Theta Settl.Time(s) | X Settl.Time(s) | L Settl.Time(s) | Lambda1 | k | Lambda2 | Alpha1"
                )
                print(
                    searching_result[i][0],'|',
                    searching_result[i][1],'|',
                    searching_result[i][2],'|',
                    searching_result[i][3],'|',
                    searching_result[i][4],'|',
                    searching_result[i][5],'|',
                    searching_result[i][6],'|',
                    searching_result[i][7],'|',
                )
                print("")
