import numpy as np


# Modified https://github.com/zziz/kalman-filter

class AltitudeKalmanFilter(object):
  def __init__(self, dt, x0=None):
    var_rangefinger = 0.009
    var_acc = 0.005

    self.F = np.array([[1, dt], [0, 1]])
    self.B = np.array([[0.5 * dt ** 2], [dt]])
    self.H = np.array([[1, 0], [0, 0]])
    self.n = self.F.shape[1]
    self.m = self.H.shape[1]

    self.Q = np.array(
      [[1 / 4 * var_acc * dt ** 4, 1 / 2 * var_acc * dt ** 3], [1 / 2 * var_acc * dt ** 3, var_acc * dt ** 2]])
    self.R = np.array([[var_rangefinger, 0], [0, 0.001]])

    self.P = np.eye(self.n)  # initialize as identity
    self.x = np.zeros((self.n, 1)) if x0 is None else x0

  def predict_altitude(self, u=0):
    self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
    self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
    x_pred = np.dot(self.H, self.x)
    alt_pred = x_pred[0]
    return alt_pred

  def update(self, z):
    y = z - np.dot(self.H, self.x)
    S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
    K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
    self.x = self.x + np.dot(K, y)
    I = np.eye(self.n)
    self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P),
                    (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)


def example():
  dt = 1.0 / 60
  F = np.array([[1, dt], [0, 1]])
  B = np.array([[0.5 * dt ** 2], [dt]])
  H = np.array([[1, 0], [0, 0]])
  var_acc = 0.005
  Q = np.array([[1 / 4 * var_acc * dt ** 4, 1 / 2 * var_acc * dt ** 3], [1 / 2 * var_acc * dt ** 3, var_acc * dt ** 2]])
  var_rangefinger = 0.009
  R = np.array([[var_rangefinger, 0], [0, 0.001]])

  x = np.linspace(-0, 1, 100)
  measurements = - (1 * x ** 2 + 2 * x - 2) + np.random.normal(0, 2, 100)

  kf = AltitudeKalmanFilter(dt=dt)
  # kf = AltitudeKalmanFilter(dt)
  print(kf)
  predictions = []

  for z in measurements:
    print(kf.predict())
    predictions.append(np.dot(H, kf.predict())[0])
    kf.update(z)

  import matplotlib.pyplot as plt
  plt.plot(range(len(measurements)), measurements, label='Measurements')
  plt.plot(range(len(predictions)), np.array(predictions), label='Kalman Filter Prediction')
  plt.legend()
  plt.show()


if __name__ == '__main__':
  example()
