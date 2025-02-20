import numpy as np
from scipy.spatial.transform import Rotation


# Tmat_worldhat
Tmat_world_head = np.eye(4)
rmat = Tmat_world_head[:3, :3]
euler = Rotation.from_matrix(Tmat_world_head).as_euler('xyz', degrees=False)
euler[:2] = 0.
Tmat_world_foot = np.eye(4)
Tmat_world_foot[:3,:3] = Rotation.from_euler(euler).as_matrix()
Tmat_world_foot[:2,3] = Tmat_world_foot[:2,3] # z 0

# Tmat_world_lftend, Tmat_world_rgtend

# Tmat_foot_lftend
# Tmat_foot_rgtend
