from math import sin,cos
import math
import numpy as np

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0 ,0] * R[0 ,0] +  R[1 ,0] * R[1 ,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2 ,1] , R[2 ,2])
        y = math.atan2(-R[2 ,0], sy)
        z = math.atan2(R[1 ,0], R[0 ,0])
    else :
        x = math.atan2(-R[1 ,2], R[1 ,1])
        y = math.atan2(-R[2 ,0], sy)
        z = 0

    return np.array([x, y, z])


def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


#Theta (q),a,alpha,d
def getTransformMatrix(q, d, a, al):
    A = [[cos(q), -sin(q)*cos(al), sin(q)*sin(al), a*cos(q)],
         [sin(q), cos(q)*cos(al), -cos(q)*sin(al), a*sin(q)],
         [0, sin(al), cos(al), d],
         [0,0,0,1]]
    return A

def fromRTtoTrans(R,t):
    tr = np.concatenate((R, t), axis=1)
    zeros = np.asarray([[0, 0, 0, 1]])
    tr = np.concatenate((tr,zeros), axis=0)
    return tr

def getRotationMatrixZ(q):
    Rz = np.asarray([[cos(q), -sin(q), 0],
                    [sin(q), cos(q), 0],
                    [0, 0, 1]])
    return Rz


def inverseTransform(T):
    Rmat = T[0:3, 0:3]
    Rmat = np.transpose(Rmat)
    t = -np.matmul(Rmat,T[0:3,3])
    t = t.reshape((-1, 1))
    return fromRTtoTrans(Rmat,t)

def normalize_angle(x):
    x = x % (2 * np.pi) # force in range [0, 2 pi)
    if x > np.pi: # move to [-pi, pi)
        x -= 2 * np.pi
    return x

if __name__ == '__main__':
    x = 1-359
    x = np.radians(x)
    print(np.degrees(normalize_angle(x)))