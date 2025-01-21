import numpy as np
from scipy import linalg

if __name__ == "__main__":
    #plane_normal = np.array([[fit[0]], [fit[1]], [-1]])
    #plane_normal = plane_normal / linalg.norm(plane_normal)
    plane_normal = [[ 0.00427268],[-0.00701908],[-0.99996624]]
    plane_normal=np.array(plane_normal)
    up_normal = np.array([[0],[0],[1]], dtype=np.float32)
    print("plane_normal="+str(plane_normal))
    print("shape="+str(np.shape(plane_normal)))
    '''[[ 0.00427268]
    [-0.00701908]
    [-0.99996624]]'''
    print("up normal="+str(up_normal))
    print("shape="+str(np.shape(up_normal)))
    
    '''Debugging comments:
    A=plane_normal.T
    B=up_normal
    '''
    # https://math.stackexchange.com/a/897677/1012327    
    '''
    G = np.array([
        [np.dot(plane_normal.T,up_normal)[0][0], -linalg.norm(np.cross(plane_normal.T[0],up_normal.T[0])), 0],
        [linalg.norm(np.cross(plane_normal.T[0],up_normal.T[0])), np.dot(plane_normal.T,up_normal)[0][0], 0],
        [0, 0, 1]
    ])
    print("G=")
    print(G)
    print("##########")
    '''
    A=plane_normal.T[0]
    B=up_normal.T[0]
    G = np.array([
        [np.dot(A,B), -linalg.norm(np.cross(A,B)), 0],
        [linalg.norm(np.cross(A,B)), np.dot(A,B), 0],
        [0, 0, 1]
    ])
    print("new G=")
    print(G)
    
    
    F = np.array([plane_normal.T[0], ((up_normal-np.dot(plane_normal.T,up_normal)[0][0]*plane_normal)/linalg.norm((up_normal-np.dot(plane_normal.T,up_normal)[0][0]*plane_normal))).T[0], np.cross(up_normal.T[0],plane_normal.T[0])]).T
    R = F @ G @ linalg.inv(F)
    print("F=")
    print(F)
    
    print("R=")
    print(R)
    
    
    print("#####")
    F = linalg.inv(np.array([plane_normal.T[0], ((up_normal-np.dot(plane_normal.T,up_normal)[0][0]*plane_normal)/linalg.norm((up_normal-np.dot(plane_normal.T,up_normal)[0][0]*plane_normal))).T[0], np.cross(up_normal.T[0],plane_normal.T[0])]))
    R = linalg.inv(F) @ G @ F
    print("new F=")
    print(F)
    
    print("new R=")
    print(R)
    
    exit()

    R = R @ [[1,0,0],[0,-1,0],[0,0,1]] # i dont fucking know why
    print("R2=")
    print(R)

    to_world_coords_matrix = np.array(np.vstack((np.c_[R, [0,0,0]], [[0,0,0,1]])))
    
    print("to_world_coords_matrix")
    print(to_world_coords_matrix)
    
    
    '''vorher:
    [[ 0.45928318 -0.88827964 -0.00427268  0.        ]
    [ 0.88827964  0.45924942  0.00701908  0.        ]
    [ 0.00427268  0.00701908 -0.99996624  0.        ]
    [ 0.          0.          0.          1.        ]]'''