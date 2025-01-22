import numpy as np
from scipy import linalg



def main():
    #plane_normal = np.array([[fit[0]], [fit[1]], [-1]])
    #plane_normal = plane_normal / linalg.norm(plane_normal)
    plane_normal = [[ 0.00427268],[-0.00701908],[-0.99996624]]  #from calibration
    #plane_normal = [[ 0],[1],[0]]
    plane_normal = plane_normal / linalg.norm(plane_normal)
    plane_normal=np.array(plane_normal)
    up_normal = np.array([[0],[0],[1]], dtype=np.float32)
    print("plane_normal="+str(plane_normal))
    print("shape="+str(np.shape(plane_normal)))
    '''[[ 0.00427268]
    [-0.00701908]
    [-0.99996624]]'''
    print("up normal="+str(up_normal))
    print("shape="+str(np.shape(up_normal)))

    whichFunction="newImplementation"

    if whichFunction =="startingCode":
        to_world_coords_matrix= createMatrix_startingCode(plane_normal,up_normal)
        print("createMatrix_startingCode:")
        print(to_world_coords_matrix)
    
    if whichFunction =="prettifiedCode":
        to_world_coords_matrix= createMatrix_prettifiedCode(plane_normal,up_normal)
        print("createMatrix_prettifiedCode:")
        print(to_world_coords_matrix)

    if whichFunction =="newImplementation":
        to_world_coords_matrix= createMatrix_newImplementation(plane_normal,up_normal)
        print("createMatrix_newImplementation:")
        print(to_world_coords_matrix)


    #Test rotation
    print("### Testing ###")
    test_plane_normal = plane_normal.T[0]
    print("test_plane_normal="+str(test_plane_normal))
    test_plane_normal = np.concatenate((test_plane_normal, [1]))
    rotated_plane_normal = np.array(to_world_coords_matrix) @ test_plane_normal
    #rotated_plane_normal = np.array(to_world_coords_matrix).dot(test_plane_normal)
    rotated_plane_normal=rotated_plane_normal[:3]
    rotated_plane_normal=np.round(rotated_plane_normal,2)
    print("rotated_plane_normal="+str(rotated_plane_normal)+" should equal "+str(up_normal))


    

def createMatrix_startingCode(plane_normal, up_normal):
    # https://math.stackexchange.com/a/897677/1012327    
    
    G = np.array([
        [np.dot(plane_normal.T,up_normal)[0][0], -linalg.norm(np.cross(plane_normal.T[0],up_normal.T[0])), 0],
        [linalg.norm(np.cross(plane_normal.T[0],up_normal.T[0])), np.dot(plane_normal.T,up_normal)[0][0], 0],
        [0, 0, 1]
    ])
    print("G=")
    print(G)
    print("##########")
    
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
    

    R = R @ [[1,0,0],[0,-1,0],[0,0,1]] # i dont fucking know why
    print("R2=")
    print(R)

    to_world_coords_matrix = np.array(np.vstack((np.c_[R, [0,0,0]], [[0,0,0,1]])))
    
    '''vorher:
    [[ 0.45928318 -0.88827964 -0.00427268  0.        ]
    [ 0.88827964  0.45924942  0.00701908  0.        ]
    [ 0.00427268  0.00701908 -0.99996624  0.        ]
    [ 0.          0.          0.          1.        ]]'''

    return to_world_coords_matrix


def createMatrix_prettifiedCode(plane_normal, up_normal):
    #same as starting code, but closer to the stack exchange example # https://math.stackexchange.com/a/897677/1012327  

    A=plane_normal.T[0]
    B=up_normal.T[0]
    G = np.array([
        [np.dot(A,B), -linalg.norm(np.cross(A,B)), 0],
        [linalg.norm(np.cross(A,B)), np.dot(A,B), 0],
        [0, 0, 1]
    ])
    print("new G=")
    print(G)
    
    
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

    return to_world_coords_matrix




def get_rotation_axis_and_angle(A, B):
    
    print("## get_rotation_axis_and_angle")
    print("A="+str(A))
    print("B="+str(B))
    # Calculate the rotation axis
    axis = np.cross(A, B)
    
    # Calculate the rotation angle
    dot_product = np.dot(A, B)
    theta = np.arccos(dot_product)
    
    # Normalize the axis if it's not zero length
    if np.linalg.norm(axis) < 1e-10:
        return np.zeros(3), 0.0
    
    normalized_axis = axis / np.linalg.norm(axis)
    return normalized_axis, theta

def create_affine_matrix(axis, angle):  #https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    print("## create_affine_matrix")
    print("axis="+str(axis))
    print("angle="+str(angle))
    kx=axis[0]
    ky=axis[1]
    kz=axis[2]
    K = np.array([\
        [0, -kz, ky],\
        [kz,0,-kx],\
        [-ky,kx,0]])
    I = np.eye(3)

    R = I + np.sin(angle)*K + (1-np.cos(angle))*np.matmul(K,K)
    print("R=")
    print(R)

    affine_matrix = np.zeros((4, 4))
    affine_matrix[:3, :3] = R
    affine_matrix[3, 3] = 1.0
    return affine_matrix
    

def createMatrix_newImplementation(plane_normal, up_normal):
    

    A=plane_normal.T[0]
    B=up_normal.T[0]

    axis, angle = get_rotation_axis_and_angle(A, B)
    
    affine_matrix = create_affine_matrix(axis, angle)   

    return affine_matrix




    

if __name__ == "__main__":
    # Example usage:
    
    
    main()



