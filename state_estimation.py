from scipy.optimize import minimize
import numpy as np
from numpy import cos,sin
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
    zeros = np.asarray([[0, 0, 0, 1]])
    tr = np.concatenate((tr,zeros), axis=0)
    return tr

def minimize_error_all(q,*args):
    parts = args[0]
    configuration = args[1]

    rotation_base, pos_base = configuration.getBasePosition()
    tr_base = fromRTtoTrans(rotation_base,pos_base)

    error = 0
    for i,qi in enumerate(q):
        d, a, al = configuration.getLink(i)

        tr_part = fromRTtoTrans(parts[i][0],parts[i][1])
        error_mat =  abs(np.matmul(tr_base,getTransformMatrix(qi,d,a,al)) - tr_part)
        error += np.mean(error_mat)

    return error

class Estimator:


    def __init__(self, robot_configuration):
        self.robot_configuration = robot_configuration

    def senseParts(self,parts):
        qinit = []
        for i,RT in parts.items():
            if len(RT) == 0:
                break
            else:
                qinit.append(0.0)

        if len(qinit)==0:
            return []
        q0 = np.asarray(qinit)

        #
        # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])},
                #         {'type': 'ineq', 'fun': lambda q: math.pi/2 - abs(q[1]) })

        cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])})
        res = minimize(minimize_error_all, q0, args=(parts,self.robot_configuration),
                       method='COBYLA',
                       tol=1e-6,
                       constraints = cons)
        return res.x
