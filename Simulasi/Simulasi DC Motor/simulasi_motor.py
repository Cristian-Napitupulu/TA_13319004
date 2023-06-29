import numpy as np
import matplotlib.pyplot as plt

K = 0.0325
L = 2.7
J = 0.0004
R = 0.4
b = 0.0022
K2 = -(R*b + K*K)/(L*J)
K3 = -(L*b+R*J)/(L*J)

x1ref = 1

a = 15
b = 7
u_max = 1
ks = (K/(L*J))*u_max

dt = 0.0005
duration = 10

x1 = np.zeros(int(duration/dt))
x2 = np.zeros(int(duration/dt))
x3 = np.zeros(int(duration/dt))
u = np.zeros(int(duration/dt))
s = np.zeros(int(duration/dt))
time = np.arange(0,duration,dt)
error = x1ref - x1

for i in range(0,int(duration/dt)-1):
    error[i+1] = x1ref - x1[i]
    s[i+1] = a*(x1ref - x1[i]) - b*x2[i] - x3[i]
    temp_ = ((L*J)/K)*(-(K2+a)*x2[i] - (K3+b)*x3[i] + ks*np.sign(s[i]))
    
    if (abs(temp_) > u_max):
        u[i+1] = u_max*np.sign(temp_)
    else:
        u[i+1] = temp_
    

    #u[i+1] = temp_

    x1[i+1] = x1[i] + dt*x2[i]
    x2[i+1] = x2[i] + dt*x3[i]
    x3[i+1] = x3[i] + dt*(K2*x2[i] + K3*x3[i] + K/(L*J)*u[i])

    time[i+1] = time[i] + dt

plt.figure('State')
plt.plot(time,x1)
plt.plot(time,x2)
plt.plot(time,x3)
plt.legend(['x1 (rad)','x2 (rad/s)','x3 (rad/s^2))'])
plt.xlabel('Time (s)')
plt.grid(True)

plt.figure('Control')
plt.plot(time,u)
plt.plot(time,s)
plt.legend(['u','s'])
plt.xlabel('Time (s)')
plt.grid(True)
plt.show()
