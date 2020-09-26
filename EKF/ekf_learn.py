"""
EKF example
"""
import numpy as np
import math
import matplotlib.pyplot as plt

show_animation = True

SIM_TIME = 50.0 #[s]
delta_t = 0.1   #[s]

# input vector
v = 1.0
w = 0.1
u = np.array([[v, w]]).T # v=1.0m/s, w=0.1rad/s

Q = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0])**2
R = np.diag([1.0, 1.0])**2 # the smaller R is, the more dependent on measurement
# 0.1 not good result--too scattered

def motion_model(x, u):
    x[0,0] = x[0,0] + u[0,0]*delta_t*math.cos(x[2,0])
    x[1,0] = x[1,0] + u[0,0]*delta_t*math.sin(x[2,0])
    x[2,0] = x[2,0] + u[1,0]*delta_t
    x[3,0] = v
    return x

def observation_model(x):
    H = np.array([
        [1,0,0,0],
        [0,1,0,0]
    ])
    z = H.dot(x)
    return z

def observation(xTrue, xDR, u):
    xTrue = motion_model(xTrue, u)

    # add noise to v and w
    vn = u[0,0] + np.random.randn() * (1.0**2)
    wn = u[1,0] + np.random.randn() * (np.deg2rad(30.0)**2)
    un = np.array([[vn, wn]]).T

    xDR = motion_model(xDR, un)

    # use noise-added groundtruth as gps data 
    zx = xTrue[0,0] + np.random.randn() * (0.5**2)
    zy = xTrue[1,0] + np.random.randn() * (0.5**2)
    z = np.array([[zx, zy]]).T 
    
    return xTrue, z, xDR, un

def jacobF(x, u):
    jF = np.array([
        [1.0, 0.0, -u[0,0]*delta_t*math.sin(x[2,0]), delta_t*math.cos(x[2,0])],
        [0.0, 1.0,  u[0,0]*delta_t*math.cos(x[2,0]), delta_t*math.sin(x[2,0])],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return jF

def jacobH():
    jH = np.array([
        [1,0,0,0],
        [0,1,0,0]
    ])
    return jH

def ekf_estimation(xEst, PEst, z, un):
    # Predict
    xPred = motion_model(xEst, un)
    jF = jacobF(xPred, un)
    PPred = jF.dot(PEst).dot(jF.T) + Q

    # Update
    jH = jacobH()
    S = jH.dot(PPred).dot(jH.T) + R
    K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    zPred = observation_model(xPred)
    delta_y = z - zPred
    xEst = xPred + K.dot(delta_y)
    PEst = (np.eye(len(xEst))-K.dot(jH)).dot(PPred)

    return xEst, PEst

def main():
    print(__file__ + "start!!!")
    time = 0.0

    # [x,y,yaw,v]T
    xTrue = np.zeros((4,1))
    xDR = np.zeros((4,1))

    xEst = np.zeros((4,1))
    PEst = np.eye(4)

    # save history
    hxTrue = xTrue
    hxDR = xDR
    hxEst = xEst
    hz = np.zeros((2,1))

    while time <= SIM_TIME:
        time += delta_t

        # get groundtruth and dead reckoning results
        xTrue, z, xDR, un = observation(xTrue, xDR, u)

        xEst, PEst = ekf_estimation(xEst, PEst, z, un)

        # save history data
        hxTrue = np.hstack((hxTrue, xTrue))
        hxDR = np.hstack((hxDR, xDR))
        hxEst = np.hstack((hxEst, xEst))
        hz = np.hstack((hz, z))

        if show_animation:
            plt.cla()
            plt.plot(hz[0,:].flatten(), hz[1,:].flatten(), ".g")
            plt.plot(hxTrue[0,:].flatten(), hxTrue[1,:].flatten(), "-b")
            plt.plot(hxDR[0,:].flatten(), hxDR[1,:].flatten(), "-k")
            plt.plot(hxEst[0,:].flatten(), hxEst[1,:].flatten(), "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

    plt.savefig("ekf.png")

if __name__ == "__main__":
    main()