import numpy as np
import matplotlib.pyplot as plt

plt.figure()

data = np.loadtxt(f'../data_files/control_data/integratorImRk5.txt')

times = data[:,0]


plt.scatter(times, data[:, 1], label=f'\nКоличество узлов {data.shape[0]}')
plt.scatter(times, data[:, 2], label=f'\nКоличество узлов {data.shape[0]}')

# plt.scatter(times, goal, label=f'\nЦелевое значение')
plt.legend()
plt.title('Зависимость расстояния до \nцелевого положения от времени')
plt.grid()
plt.xlabel('t')
plt.ylabel(r'$\theta$')
plt.savefig('theta2PID.png')
plt.show()