import numpy as np
import matplotlib.pyplot as plt

x_distortedFinal = 476.5
y_distortedFinal = 1585.5

x_c = 1244.66
y_c = 1047.35

x = np.linspace(start=450, stop=500, num=1000)
y = np.linspace(start=1550,stop=1600,num=1000)

xx,yy = np.meshgrid(x,y)

k = -117.177


x_distorted = np.cos(np.tan((y-y_c)/(x-x_c)))/(k*((x-x_c)**2 + (y-y_c)**2)+1)+x_c

y_distorted = np.sin(np.tan((y-y_c)/(x-x_c)))/(k*((x-x_c)**2 + (y-y_c)**2)+1)+y_c


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x,y,x_distorted)
fig.show()
