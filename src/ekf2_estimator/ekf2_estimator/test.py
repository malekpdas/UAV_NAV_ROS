from utils import rotate_vector, quat_to_euler, quat_to_rot_mat
import numpy as np

acc_body = [0.8775146 ,  0.42228073, -10.62237817]
acc_ned  = [0.53828742,  0.38923796, -1.33783161]
q = [-0.41348496,  0.08165342,  0.05430039, 0.90521508]

euler = np.degrees(quat_to_euler(q))
DCM = quat_to_rot_mat(q)

new_acc_ned = rotate_vector(acc_body, q) + np.array([0,0,9.8066])


print(euler)
print(DCM)
print(new_acc_ned)