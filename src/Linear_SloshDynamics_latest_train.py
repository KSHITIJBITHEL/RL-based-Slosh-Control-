
 
#For ARTPARK


import numpy as np
from numpy import sin, cos, power
import matplotlib.pyplot as plt
import random

from scipy.integrate import odeint

sign = lambda z: abs(z)/z if z!=0 else 0  # One line equivalent of the signum function



class SloshEnv:
    '''
    This is a Open-AI gym compatible environment for the underactuated
    Slosh-Container System 'https://ieeexplore.ieee.org/document/7921790'
    '''

    def __init__(self,k1= 1.776,k2= 4.084):

        self.k1 = k1
        self.k2 = k2

        # self.stateHist = np.array([])
        # self.actionHist = np.array([])
        # self.controllerGainHist = np.array([k1,k2])

        self.ms = 0.375 # in kg
        self.M = 2.7 # M=Ml+Mc+Mb+m=10.82
        self.g = 9.8
        self.c1 = 1
        self.c2 = 11
        self.cd = 0.000279 # damping coefficient given in equation (1b)
        self.l = 0.0328    #in meter
        self.d = 0       # d is external disturbance in input channel equation 1(a)


    def surface(self, zeta1, zeta2):
        return self.c1 * zeta2 + self.c2 *zeta1


    def dynamics(self, state, t,u1):
        # equation 4 decomposition of states

        zeta1 = state[0]
        sigman = state[1]
        zeta3 = state[2]
        zeta4 = state[3]
        z = state[4]

        # Using self.[variables] to facilitate future dynamical parameter updates
        # Decompose back to normal variable names for ease of writing code
        k1 = self.k1
        k2 = self.k2
        ms = self.ms
        M = self.M
        g = self.g
        c1 = self.c1
        c2 = self.c2
        cd = self.cd
        l = self.l
        d = self.d
        D = M - ms * power(cos(zeta3),2)


        # __________ EQUATION 5 ____________

        f1_term1 = ms * g * sin(zeta3) * cos(zeta3)
        f1_term2 = (cd/l) * zeta4 * cos(zeta3)
        f1_term3 = ms * l * power(zeta4,2) * sin(zeta3)

    
        f1 = (f1_term1 + f1_term2 + f1_term3)/D
        b1 = 1/D


#        f2_term1 = (g/l) * sin(zeta3)
#        f2_term2 = (cd/(ms * power(l,2))) * zeta4
#        f2_term3 = (cos(zeta3)/l) * f1


#        f2 = -(f2_term1 + f2_term2 + f2_term3)
#        b2 = np.cos(zeta3)/(l * D)


        # print(f1,b1,f2,b2)        


        b1Nominal = 1/(M-(ms/2))
        deltaB1 = b1 - b1Nominal

        wl_bar = (c2*(sigman-c2*zeta1))/(c1) 
        

        vl = -k1 * power(abs(sigman),1/2) * sign(sigman) + z


        u = (vl - wl_bar)/(c1 * b1Nominal)
        u1.append(u)

        # de = c1 * f1 + c1 * b1 * 2*sin(5*t) + c1 * deltaB1 * u   #with disturbence given in ref. paper
        de = c1 * f1 + c1 * b1 * d + c1 * deltaB1 * u

        
        zeta1Dot = (sigman - (c2*zeta1))/(c1)
        sigmanDot = -k1 * power(abs(sigman),1/2) * sign(sigman) + z + de
        # print(sigmanDot)
        zeta3Dot = zeta4 

        bigStuff = vl + de - (((c2)/(c1)) * (sigman - (c2*zeta1)))
        term1 = -(cos(zeta3)/(l*c1)) * bigStuff
        term2 =  - (g/l) * sin(zeta3) - (cd/(ms * power(l,2))) * zeta4

        zeta4Dot = term1 + term2
        zDot = -k2 * sign(sigman) 

#        print([zeta1Dot,sigmanDot,zeta3Dot,zeta4Dot,zDot])
        return [zeta1Dot,sigmanDot,zeta3Dot,zeta4Dot,zDot]


    def simulate(self, initStates, T,action,plot):
        x = initStates[0]
        xDot = initStates[1]
        phi = initStates[2]
        phiDot = initStates[3]
        self.c2 = action[0]
        self.k1 = action[1]
        self.k2 = action[2]
        xDes = 1.2
        xDotDes = 0
        phiDes = 0
        phiDotDes = 0

        ex = x - xDes
        exDot = xDot - xDotDes
        ephi = phi - phiDes
        ephiDot = phiDot - phiDotDes

        initCon = [ex,self.surface(ex,exDot),ephi,ephiDot,0]
        
        
        dT = 0.0001          # this is h from RK4 method
        x = np.array(initCon)
        sol = []
        reward = 0
        u1=[]   # control input
        # cost = []
        for t in np.arange(0,T,dT):
            sol.append(x)
            xDot = self.dynamics(x,t,u1)
            xDot = np.array(xDot)
#            print(xDot)
            x = x + xDot*dT
            coost = round(-(4*abs(x[0])+0.3*abs(x[2])+0.3*abs(x[3])),3);
            # coost = round(-(0.25*abs(x[2])),3);
            # cost.append(coost);
            #cost.append(-(0.1*(x[0])**2+0.45*x[2]**2+0.45*x[3]**2));
            reward += coost
        t = np.arange(0,T,dT)
        sol = np.array(sol)
        if abs(reward)< 100:
            done = True
            reward += 100
        else:
            done = False
            
#        t = np.linspace(0,T,10000)
#        sol = odeint(self.dynamics,initCon,t,full_output=1)
        return_state = [0,0,0,0]
        # return_state[0] = sol[-1][0]+xDes
        # return_state[1] = xDot[0]+xDotDes
        # return_state[2] = sol[-1][2] + phiDes
        # return_state[3] = sol[-1][3] + phiDotDes
        ''' taking error as states '''
        return_state[0] = sol[-1][0]
        return_state[1] = xDot[0]
        return_state[2] = sol[-1][2] 
        return_state[3] = sol[-1][3] 
       
#        plt.plot(t,sol[:,0])
#        plt.show()
#        plt.figure()
#        plt.plot(t,sol[:,1])
#        plt.show()
#        plt.figure()
        # print(np.size(u1), np.size(sol),np.size(t))
# uncomment this to plot the values at every call of this function
        if(plot):
            self.plot(t,sol,u1)
        return return_state,reward,done

    def reset(self):
        x0 = round(random.uniform(0.00, 0.250), 4)
        x1 = round(random.uniform(-0.150, 0.400), 4)
        x2 = round(random.uniform(-0.25, 0.25), 4)
        x3 = round(random.uniform(-2.50, 2.50), 4)
        return [x0,x1,x2,x3]
    def plot(self,t,sol,u1):
        
        # plt.plot(t,reward)
        # plt.xlabel("time (sec)")
        # plt.ylabel("reward")
        # plt.grid()
        # plt.figure()
        plt.plot(t,sol[:,0])        # ex=x-xDes 
        # plt.plot(t,sol[:,0]+0.175)   # actual distace of cart
        plt.xlabel("time (sec)")
        plt.ylabel("x (mm)")
        plt.grid()
        plt.figure()
        plt.plot(t,sol[:,2])
        plt.xlabel("time (sec)")
        plt.ylabel("phi (rad)")
        plt.grid()
        plt.figure()
        plt.plot(t,sol[:,3])
        plt.xlabel("time (sec)")
        plt.ylabel("phiDot (rad/sec)")
        plt.grid()
        plt.figure()
        plt.plot(t,u1)               #control input
        plt.xlabel("time (sec)")
        plt.ylabel("f control input")
        plt.grid()

    
# env = SloshEnv(0.1,0.1)    #0.9, 0.9 best gains 
# env.dynamics([-0.175,0,0,0,0],0)
# print(env.surface(-0.175,0))for i in range(10):
    # next_state,reward,done  = env.simulate([0,0,0,0],8,[3.8489 ,1.0265, 0.1165],False)
# 3.8489 ,1.0265, 0.1165
#[3.5626 0.8711 0.1002]