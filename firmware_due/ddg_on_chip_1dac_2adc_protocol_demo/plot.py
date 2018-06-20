import matplotlib.pyplot as plt
data = []
with open("2-data.txt") as f:
    data.append(f.readlines())
plt.autoscale(enable=True,axis='both')
plt.plot(data)
plt.show()
