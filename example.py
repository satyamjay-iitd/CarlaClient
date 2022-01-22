from LaneDetection import lane_detector
import numpy as np
import matplotlib.pyplot as plt

ld = lane_detector.LaneDetector()
out = ld("_out/001850.png")

x = np.linspace(0, 60)
yl = out[0](x)
yr = out[1](x)
plt.plot(x, yl, label="yl")
plt.plot(x, yr, label="yr")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.legend()
plt.axis("equal")
plt.show()
