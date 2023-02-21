import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
  
trax = []
tray = []
cx = []
cy = []
  
f = open('trajectory.txt','r')
g = open('carpose.txt','r')


for row in f:
    row = row.split(' ')
    trax.append(float(row[0]))
    tray.append(float(row[1]))

for row in g:
    row = row.split(' ')
    cx.append(float(row[0]))
    cy.append(float(row[1]))
  
#plt.bar(names, marks, color = 'g', label = 'File Data')
plt.plot(trax, tray, color='r', label='Planned Trajectory')
plt.plot(cx, cy, color='g', label='Path followed by the car')

plt.gca().add_patch(Rectangle((20-1.92/2,0-0.96/2),1.92,0.96, 
                    edgecolor='red',
                    facecolor='none',
                    lw=2))

plt.gca().add_patch(Rectangle((35-1.92/2,2-0.96/2),1.92,0.96, 
                    edgecolor='red',
                    facecolor='none',
                    lw=2))


plt.xlabel('X', fontsize = 12)
plt.ylabel('Y', fontsize = 12)
plt.ylim(-1, 10)
plt.title('Planner', fontsize = 20)
plt.legend()
plt.show()