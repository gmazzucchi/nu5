import matplotlib.pyplot as plt
import math

# maybe TAU=L/15 is a good compromesz

L=150
TAU=L/15

ys=[math.exp(-(i/TAU)) for i in range(0, L-1)]
plt.plot(ys)

TAU=L/5
ys1=[math.exp(-(i/TAU)) for i in range(0, L-1)]
plt.plot(ys1)

TAU=L/30
ys2=[math.exp(-(i/TAU)) for i in range(0, L-1)]
plt.plot(ys2)

plt.show()





