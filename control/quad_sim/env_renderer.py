import numpy as np
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import sys

from .utils import get_rotation_mtx


class EnvRenderer(object):
  def __init__(self, quad_model):
    self.fig = plt.figure()
    self.ax = Axes3D.Axes3D(self.fig)
    self.ax.set_xlim3d([-5.0, 5.0])
    self.ax.set_xlabel('X')
    self.ax.set_ylim3d([-5.0, 5.0])
    self.ax.set_ylabel('Y')
    self.ax.set_zlim3d([0, 5.0])
    self.ax.set_zlabel('Z')
    self.ax.set_title('Quadcopter Simulation')
    self.quad_plot = {}
    self.quad_model = quad_model
    self.init_plot()
    self.fig.canvas.mpl_connect('key_press_event', self.keypress_routine)

  def init_plot(self):
    self.quad_plot['l1'], = self.ax.plot([], [], [], color='blue', linewidth=3, antialiased=False)
    self.quad_plot['l2'], = self.ax.plot([], [], [], color='red', linewidth=3, antialiased=False)
    self.quad_plot['hub'], = self.ax.plot([], [], [], marker='o', color='green', markersize=6, antialiased=False)
    plt.ion()
    plt.show()

  def update(self, states):
    euler_angles = states[3:6]
    position = states[0:3]
    rot_mtx = get_rotation_mtx(euler_angles)
    L = self.quad_model.L
    points = np.array([[-L, 0, 0], [L, 0, 0], [0, -L, 0], [0, L, 0], [0, 0, 0], [0, 0, 0]]).T
    points = np.dot(rot_mtx, points)
    points[0, :] += position[0]
    points[1, :] += position[1]
    points[2, :] += position[2]
    self.quad_plot['l1'].set_data(points[0, 0:2], points[1, 0:2])
    self.quad_plot['l1'].set_3d_properties(points[2, 0:2])
    self.quad_plot['l2'].set_data(points[0, 2:4], points[1, 2:4])
    self.quad_plot['l2'].set_3d_properties(points[2, 2:4])
    self.quad_plot['hub'].set_data(points[0, 5], points[1, 5])
    self.quad_plot['hub'].set_3d_properties(points[2, 5])
    plt.draw()
    plt.pause(0.0001)

  def keypress_routine(self, event):
    sys.stdout.flush()
    if event.key == 'x':
      y = list(self.ax.get_ylim3d())
      y[0] += 0.2
      y[1] += 0.2
      self.ax.set_ylim3d(y)
    elif event.key == 'w':
      y = list(self.ax.get_ylim3d())
      y[0] -= 0.2
      y[1] -= 0.2
      self.ax.set_ylim3d(y)
    elif event.key == 'd':
      x = list(self.ax.get_xlim3d())
      x[0] += 0.2
      x[1] += 0.2
      self.ax.set_xlim3d(x)
    elif event.key == 'a':
      x = list(self.ax.get_xlim3d())
      x[0] -= 0.2
      x[1] -= 0.2
      self.ax.set_xlim3d(x)

  def cleanup(self):
    plt.close(self.fig)
