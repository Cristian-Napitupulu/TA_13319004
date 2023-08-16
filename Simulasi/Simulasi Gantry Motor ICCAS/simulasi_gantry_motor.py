# Simulasi Gantry Motor ICCAS
# Oleh: Cristian MP Napitupulu

# "Simulasi" pada file ini merujuk pada while-loop di bawah
# Pada proses simulasi x dan l memiliki satuan meter
# Pada proses simulasi theta memiliki satuan radian
# Namun, theta pada proses analisis respon (rise time, settling time, dll)...
# ...theta memiliki satuan derajat

import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd
import os

absolute_folder_path = os.path.abspath(os.path.dirname(__file__))
relative_folder_path = "Gambar/"
folder_path = os.path.join(absolute_folder_path, relative_folder_path)

# Fungsi untuk mendapatkan nilai sign dari matriks S (sliding surface)
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

settle_percentage = 0.02 # 2%
begin_rise_percentage = 0.1 # 0%
end_rise_percentage = 0.9 # 95%

# Fungsi untuk mendapatkan nilai rise time
def rise_time(data, setpoint):
    begin_rise_time_percentage = begin_rise_percentage
    end_rise_time_percentage = end_rise_percentage
    begin_rise_time = 0.0
    end_rise_time = 0.0
    amplitude = abs(data[0] - setpoint)
    for i in range(len(data)):
        if abs(data[i] - setpoint) <= (1 - begin_rise_time_percentage) * amplitude:
            begin_rise_time = i * dt
            break
    for i in range(len(data)):
        if abs(data[i] - setpoint) <= (1 - end_rise_time_percentage) * amplitude:
            end_rise_time = i * dt
            break
    return round(end_rise_time - begin_rise_time, 3)

# Fungsi untuk mendapatkan nilai settling time
def settling_time(data, setpoint, error_max=0.0):
    settling_time_percentage = settle_percentage
    amplitude = abs(data[0] - setpoint) * settling_time_percentage
    if error_max != 0.0:
        amplitude = error_max
        
    settling_time = 0.0
    for i in range(len(data) - 1, 0, -1):
        if abs(data[i] - setpoint) >= amplitude:
            settling_time = i * dt
            break
    return round(settling_time, 3)

# Fungsi untuk mendapatkan nilai RMSE steady state
def rmse_steady_state(data, setpoint, error_max=0.0):
    settling_time_percentage = settle_percentage
    amplitude = abs(data[0] - setpoint) * settling_time_percentage
    if error_max != 0.0:
        amplitude = error_max
        
    settling_index = 0
    for i in range(len(data) - 1, 0, -1):
        if abs(data[i] - setpoint) >= amplitude:
            settling_index = i
            break
    
    sum_squared_error = 0.0
    for i in range (len(data)-1, settling_index, -1):
        sum_squared_error += (data[i] - setpoint) ** 2
    return round(np.sqrt(sum_squared_error / (len(data) - settling_index)), 3)

# Physical Parameter
# Gantry
mc = 2.0
mt = 2.0
bt = 10.0
br = 10.0
g = 9.81

# Motor 1
L1 = 0.0015
R1 = 10.0
J1 = 0.0005
b1 = 0.0004
rp1 = 0.01
Ke1 = 0.05
Kt1 = 0.05

# Motor 2
L2 = 0.0025
R2 = 1.20
J2 = 0.0007
b2 = 0.0004
rp2 = 0.01
Ke2 = 0.08
Kt2 = 0.08

control_limit = 4.2  # Volt
constrained = not True
scenario_name = "unconstrained"

# Control Parameter
# Parameter for theta
lambda1 = 20.0
lambda2 = 1.0
matrix_lambda = np.matrix([[lambda1], [lambda2]])

# Parameter for x and l
alpha1 = 10.0
alpha2 = 5.0
matrix_alpha = np.matrix([[alpha1, 0.0], [0.0, alpha2]])

# Parameter for x_dot and l_dot
beta1 = 10.0
beta2 = 5.0
matrix_beta = np.matrix([[beta1, 0.0], [0.0, beta2]])

# K must be > 0
k1 = 0.01
k2 = 0.01
k = [[k1], [k2]]
# k = 0.0005

# print("matrix lambda: \n", matrix_lambda)
# print("matrix alpha: \n", matrix_alpha)
# print("matrix beta: \n", matrix_beta)
# print("k: \n", k)

# Simulation Parameter
dt = 0.0001
timeout_duration = 15.0
steady_state_checking_duration_window = 1.0


y_desired = np.matrix([[1.0], [0.5], [0.0]])
y_initial = np.matrix([[0.0], [1.5], [0.0]])

result = []

variation_number = 2
show_result = True
# j adalah variasi yang akan diuji
# j = 0 -> unconstrained
# j = 1 -> constrained
for j in range (variation_number):
    # deklarasi variabel untuk initial condition
    x = [y_initial[0, 0]]
    x_dot = [0.0]
    x_dot_dot = [0.0]
    l = [y_initial[1, 0]]
    l_dot = [0.0]
    l_dot_dot = [0.0]
    theta = [y_initial[2, 0]]
    theta_dot = [0.0]
    theta_dot_dot = [0.0]

    theta_max_error = 0.001 * 180 / np.pi # degree

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

    # Mulai simulasi
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

        # Buat variabel untuk q, q_dot, q_dot_dot, dan q_desired yang...
        # ...menyimpan nilai 2 state utama: x dan l serta turunannya 
        q_now = np.matrix([[x[i]], [l[i]]])
        q_dot_now = np.matrix([[x_dot[i]], [l_dot[i]]])
        q_dot_dot_now = np.matrix([[x_dot_dot[i]], [l_dot_dot[i]]])
        q_desired = np.matrix([[y_desired[0, 0]], [y_desired[1, 0]]])

        # Buat variabel untuk sliding surface dan control
        sliding_surface_now = np.matrix([[Sx[i]], [Sl[i]]])
        sliding_surface_now = (
            np.matmul(matrix_alpha, np.subtract(q_now, q_desired))
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

        if j==1:    # jika sedang melakukan constrained scenario ...
            constrained = True
            scenario_name = "constrained"
            # ... maka control yang dihasilkan akan dibatasi oleh control limit
            control_now = np.clip(control_now, -control_limit, control_limit)

        # Update x_dot_dot, l_dot_dot, theta_dot_dot...
        # ... menggunakan persamaan state space
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

        # Bagian ini hanya untuk menampilkan progress
        if i % 100 == 0:
            print("Progress: ", round(i / (timeout_duration / dt) * 100, 1), "%", end="\r")

        # Bagian ini untuk menghentikan simulasi jika terjadi divergensi
        if (
            abs(x[i] - y_desired[0, 0]) > 2**16
            or abs(l[i] - y_desired[1, 0]) > 2**16
            or abs(theta[i] - abs(y_desired[2, 0])) > 2**16
        ):
            if abs(x[i] - y_desired[0, 0]) > 2**16:
                print("x diverged at time: ", i * dt, "s", "x: ", x[i], "y_desired: ", y_desired[0, 0])
            if abs(l[i] - y_desired[1, 0]) > 2**16:
                print("l diverged at time: ", i * dt, "s", "l: ", l[i], "y_desired: ", y_desired[1, 0])
            if abs(theta[i] - abs(y_desired[2, 0])) > 2**16:
                print("theta diverged at time: ", i * dt, "s", "theta: ", theta[i], "y_desired: ", y_desired[2, 0])
            
            print("Simulation Failed!")
            show_result = False
            break

        # Tambahkan increment pada while-loop untuk simulasi
        i += 1

    # Simulasi selesai
    print("Simulation Completed!")

    # Konversi dari radian ke derajat untuk keperluan plotting
    theta = [math.degrees(i) for i in theta]
    theta_dot = [math.degrees(i) for i in theta_dot]

    # Lakukan analisis untuk mendapatkan rise time, settling time, dan RMSE...
    # ...untuk setiap variabel kemudian simpan ke dalam list
    result.append(
        {
            "Variation": scenario_name,
            "Rise time x (s)": rise_time(x, y_desired[0, 0]),
            "Settling time x (s)": settling_time(x, y_desired[0, 0]),
            "RMSE x (m)": rmse_steady_state(x, y_desired[0, 0]),
            "Rise time l (s)": rise_time(l, y_desired[1, 0]),
            "Settling time l (s)": settling_time(l, y_desired[1, 0]),
            "RMSE l (m)": rmse_steady_state(l, y_desired[1, 0]),
            "Settling time theta (s)": settling_time(theta, y_desired[2, 0], error_max= theta_max_error),
            "RMSE theta (degree)": rmse_steady_state(theta, y_desired[2, 0], error_max= theta_max_error),
        }
    )

    # print(result)
    
    # Bagian ini untuk melakukan plotting hasil simulasi (jika diinginkan)
    if show_result:
        print ("Plotting " +scenario_name+ " scenario...")

        # Buat array untuk waktu
        time = np.arange(0, timeout_duration + dt, dt)
        # print(len(x))
        # print(len(time))
        # print(theta_dot_dot)

        # Plotting
        plt.figure(scenario_name + " x vs time")
        plt.plot(time, x_dot, "b--", label="x_dot (m/s)")
        plt.plot(time, x, "r", label="x (m)")
        plt.legend(loc="upper right")
        plt.xlabel("time (s)")
        plt.ylabel("x")
        plt.title("x vs time")
        plt.grid(True)
        plt.savefig(folder_path + scenario_name + " x vs time.svg", format='svg', transparent=True)
        plt.savefig(folder_path + scenario_name + " x vs time.png")

        plt.figure(scenario_name + " l vs time")
        plt.plot(time, l_dot, "g--", label="l_dot (m/s)")
        plt.plot(time, l, "b", label="l (m)")
        plt.legend(loc="upper right")
        plt.xlabel("time (s)")
        plt.ylabel("l")
        plt.title("l vs time")
        plt.grid(True)
        plt.savefig(folder_path + scenario_name + " l vs time.svg", format='svg', transparent=True)
        plt.savefig(folder_path + scenario_name + " l vs time.png")

        plt.figure(scenario_name + " theta vs time")
        plt.plot(time, theta_dot, "r--", label="theta_dot (degree/s)")
        plt.plot(time, theta, "g", label="theta (degree)")
        plt.legend(loc="upper right")
        plt.xlabel("time (s)")
        plt.ylabel("theta")
        plt.title("theta vs time")
        plt.grid(True)
        plt.savefig(folder_path + scenario_name + " theta vs time.svg", format='svg', transparent=True)
        plt.savefig(folder_path + scenario_name + " theta vs time.png")

        plt.figure(scenario_name + " u1 vs time")
        plt.plot(time, Ux, "r", label="u1 (volt)")
        plt.plot(time, Sx, "b--", label="s1")
        plt.legend(loc="upper right")
        plt.xlabel("time (s)")
        plt.ylabel("u1 Response")
        plt.title("u1 vs time")
        plt.grid(True)
        plt.savefig(folder_path + scenario_name + " u1 vs time.svg", format='svg', transparent=True)
        plt.savefig(folder_path + scenario_name + " u1 vs time.png")

        plt.figure(scenario_name + " u2 vs time")
        plt.plot(time, Ul, "r", label="u2 (volt)")
        plt.plot(time, Sl, "b--", label="s2")
        plt.legend(loc="upper right")
        plt.xlabel("time (s)")
        plt.ylabel("u2 Response")
        plt.title("u2 vs time")
        plt.grid(True)
        plt.savefig(folder_path + scenario_name + " u2 vs time.svg", format='svg', transparent=True)
        plt.savefig(folder_path + scenario_name + " u2 vs time.png")

        plt.figure(scenario_name + " state vs time")
        plt.plot(time, x, "r", label="x (m)")
        plt.plot(time, l, "b", label="l (m)")
        plt.plot(time, theta, "g", label="theta (degree)")
        plt.legend(loc="upper right")
        plt.xlabel("time (s)")
        plt.ylabel("State")
        plt.title("State vs time")
        plt.grid(True)
        plt.savefig(folder_path + scenario_name + " state vs time.svg", format='svg', transparent=True)
        plt.savefig(folder_path + scenario_name + " state vs time.png")

        print ("Plotting Done!")

    # plt.show()

# Simpan hasil analisis ke dalam file excel
print ("Saving result to excel...")
filename = "result.xlsx"
save_path = os.path.join(absolute_folder_path, filename)
df = pd.DataFrame(result)
df.to_excel(save_path, index=False)
    
