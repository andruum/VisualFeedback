from scipy.optimize import minimize
from numpy import *
import math


#Theta (q),a,alpha,d
def getTransformMatrix(q, d, a, al):
    A = [[cos(q), -sin(q)*cos(al), sin(q)*sin(al), a*cos(q)],
         [sin(q), cos(q)*cos(al), -cos(q)*sin(al), a*sin(q)],
         [0, sin(al), cos(al), d],
         [0,0,0,1]]
    return A

def fromRTtoTrans(R,t):
    tr = np.concatenate((R, t), axis=1)
    tr = np.concatenate((tr, [0, 0, 0, 1]), axis=0)
    return tr

def minimize_error_all(q,*args):
    parts = args[0]
    configuration = args[1]

    rotation_base, pos_base = configuration.getBasePosition()
    tr_base = fromRTtoTrans(rotation_base,pos_base)

    error = 0
    for i,qi in enumerate(q):
        d, a, al = configuration.getLink(i)
        tr_part = fromRTtoTrans(parts(i))
        error += tr_part -tr_base*getTransformMatrix(qi,d,a,al)

    return error

class Estimator:


    def __init__(self, robot_configuration):
        self.robot_configuration = robot_configuration

    def senseParts(self,parts):
        q0 = np.array(0, 0)

        cons = ({'type': 'ineq', 'fun': lambda q: math.pi/2 - abs(q[0]) },
                {'type': 'ineq', 'fun': lambda q: math.pi/2 - abs(q[1]) })

        res = minimize(minimize_error_all, q0, args=(parts,self.robot_configuration),
                       method='COBYLA',
                       tol=1e-6,
                       constraints=cons)
        return res
