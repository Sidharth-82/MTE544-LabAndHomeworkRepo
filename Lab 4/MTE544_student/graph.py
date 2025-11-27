import numpy as np
import matplotlib.pyplot as plt

# parameters
K = 20
theta_adj_max = 0.1

# error range
e = np.linspace(-0.2, 0.2, 400)

# function
theta_adj = theta_adj_max / (1 + K * np.abs(e))

# plot
plt.figure()
plt.plot(e, theta_adj)
plt.xlabel("Ball Error  e_ball  (m)")
plt.ylabel("Theta Adjustment  θ_adj  (rad)")
plt.title("Nonlinear Theta Adjustment Function\nθ_adj = θ_adj_max / (1 + K|e_ball|)")
plt.grid(True)

plt.show()