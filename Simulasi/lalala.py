import numpy as np

def moving_average(data, window):
    weights = np.ones(window) / window
    return np.convolve(data, weights, mode='same')

# Example usage
data = np.array([10, 20, 30, 40, 50, 60, 70, 80, 90])
window_size = 9

result = moving_average(data, window_size)
print(result)

print(np.where(result == np.amax(result)))
