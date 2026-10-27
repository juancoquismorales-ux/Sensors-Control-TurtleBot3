import numpy as np
import control as ct

# week 5 lab part D1
B_e = np.array([[0.0], [1.0]])
print("  v0      det(C)     rank(C)")
for v0 in [1.0, 0.5, 0.1, 0.01, 0.0]:
    A_e = np.array([[0.0, v0], [0.0, 0.0]])
    Co_e = ct.ctrb(A_e, B_e)
    print("%5.2f   %8.3f   %d" % (v0, np.linalg.det(Co_e), np.linalg.matrix_rank(Co_e)))