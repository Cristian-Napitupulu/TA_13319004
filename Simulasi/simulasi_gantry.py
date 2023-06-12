import numpy as np
import matplotlib.pyplot as plt

# Parameter
m1 = 1
m2 = 1.5
l = 0.5
g = 9.8
D = 12.32
R = 0.5
Kt = 0.0071619
Ke = 0.0071619
rp = 0.012
r = 1

# V = ax'' + bx' + ctheta''
a = (R*rp/(Kt*r))*(m1+m2)
b = ((D*R*rp)/(Kt*r) + (Ke*r)/rp)
c = (m1*l*R*rp)/(Kt*r)