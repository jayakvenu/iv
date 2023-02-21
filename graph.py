import matplotlib.pyplot as plt

import pandas as pd
traj_data = pd.read_csv('trajectory.txt', header=None)
car_data = pd.read_csv('carpose.txt', header=None)

ax = traj_data.plot(x=0,y=1)
ax.plot(car_data, x=0,y=1)
plt.show()
