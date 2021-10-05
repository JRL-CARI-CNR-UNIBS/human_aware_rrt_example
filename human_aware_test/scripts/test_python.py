from scipy.stats import ks_2samp
import numpy as np

np.random.seed(123456)
x = np.random.normal(0, 1, 1000)
y = np.random.normal(0, 1, 1000)
z = np.random.normal(1.1, 0.9, 1000)

print(ks_2samp(x, y))
