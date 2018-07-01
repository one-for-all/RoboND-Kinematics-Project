import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":
    with open('errors.npy', 'r') as f:
        errors = np.load(f)
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
        f.tight_layout()
        ax1.plot(errors[0])
        ax1.set_title("position error")
        ax2.plot(errors[1])
        ax2.set_title("orientation error")
