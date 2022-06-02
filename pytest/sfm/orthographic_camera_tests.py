import numpy as np
from scipy.spatial.transform import Rotation as R

p_o = np.array([[-0.01,-0.01,0]]).T
R_o_c = R.from_rotvec([0,0,0]).as_matrix()
t_o_c = np.array([[0,0,0]],dtype=np.float32).T

s_x = 2*1e-6
s_y = 2*1e-6

m = 0.08

w = 2590
h = 1920

c_x = 1180
c_y = 1010

p_c = R_o_c @ p_o + t_o_c

x_u = m * p_c

xi = 1/s_x * x_u + np.array([[c_x,c_y,1]]).T

print(xi)


xd = s_x * (xi-np.array([[c_x,c_y,1]]).T)

p_c = 1/m * xd

print(p_c)
