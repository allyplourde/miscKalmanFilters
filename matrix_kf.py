# Multi-Dimensional Kalman Filter
# for use of determining a single value from a
# set of measurements using kalman filtering
# author: aplourde

import numpy as np

def kalman_gain(predicted_convariance_matrix, obersvation_error):
    H = np.identity(6)
    num = predicted_convariance_matrix.dot(H) 
    dom = (H.dot(predicted_convariance_matrix).dot(H) + obersvation_error)
    print(num)
    print(dom)
    K = predicted_convariance_matrix.dot(H) / (H.dot(predicted_convariance_matrix).dot(H) + obersvation_error)
    if K.any() < 0 or K.any() > 1:
        print("Error! KGAIN outside of desired range")
    return K

def calculateNewError(kgain, previousEstimateError):
    newError = (1 - kgain) * previousEstimateError
    return newError

def current_estimate(kgain, previousEstimate, measuredVal, previousEstimateError):
    currentEstimate = previousEstimate + kgain*(measuredVal - previousEstimate)
    newError = calculateNewError(kgain, previousEstimateError)
    return currentEstimate, newError

def matrix_kf(est, est_err, measurements, mea_err):
    for i in range(len(measurements)):
        kgain = kalman_gain(est_err, mea_err[i])
        est, est_err = current_estimate(kgain, est, measurements[i], est_err)
        print(kgain, est, est_err)
    return kgain, est, est_err

def predicted_state(state_matrix, control_matrix, process_covariance_matrix):
    A = np.array([[1, 0, 0, dT, 0, 0],    
                  [0, 1, 0, 0, dT, 0],
                  [0, 0, 1, 0, 0, dT],
                  [0, 0, 0, 1, 0 ,0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])

    B = np.array([[0.5*dT**2, 0, 0],
                  [0, 0.5*dT**2, 0],
                  [0, 0, 0.5*dT**2],
                  [dT, 0, 0],
                  [0, dT, 0],
                  [0, 0, dT]])

    X_predicted = A.dot(state_matrix) + B.dot(control_matrix) # + w
    P_predicted = A.dot(process_covariance_matrix).dot(A.transpose()) # + Q

    print("X_predicted")
    print(X_predicted)
    print("P_predicted")
    print(P_predicted)
    return X_predicted, P_predicted


if __name__ == "__main__":

    dT = 1 # time for 1 cycle
    #Initial State
    initial_state = np.array([[4000], #pos_x
                              [0], #pos_y
                              [0], #pos_z
                              [280], #vel_x
                              [0], #vel_y
                              [0]  #vel_z
                             ])

    control_matrix = np.array([[2], #acc_x
                               [0], #acc_y
                               [0]  #acc_z
                             ])
    
    Samples = np.array([[4000, 0, 0, 280, 0, 0],
                        [4260, 0, 0, 282, 0, 0],
                        [4550, 0, 0, 285, 0, 0],
                        [4860, 0, 0, 286, 0, 0],
                        [5110, 0, 0, 290, 0, 0]])
    
    n_samp = len(Samples)

    print(np.ones([n_samp, n_samp]))

    deviation_matrix = Samples - np.ones([n_samp,n_samp]).dot(Samples)*(1/len(Samples))
    covariance_matrix = deviation_matrix.transpose().dot(deviation_matrix)
    print(covariance_matrix)
    X = initial_state # state matrix (position and velocity of object being tracked)
    P = np.array([[400, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 25, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0]]) # process covariance matrix (error in estimate/processing)
    Q = np.array([]) # process noise covariance matrix
    R = np.array([[625, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 36, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0]]) # sensor noise convariance matrix
    u = np.array(control_matrix) # control variable matrix (eg acceleration)
    w = np.array([]) # predicted state noise matrix

    #Previous State

    #New (Predicted) State
    X_p, P_p = predicted_state(X, u, P)

    #Calculate Kalman Gain
    K = kalman_gain(P_p, R)
    print(K)

    #Update with New Measurement
    C = np.identity(6) # 
    M = np.array([Samples[1]]).transpose()# measured state
    Y = C.dot(M) # + z

    X_new = X_p + K*(Y-np.identity(6).dot(X_p))
    print(X_new)

    #Update Process Covariance Matrix
    
    P_new = np.nan_to_num(np.identity(6)-K).dot(np.nan_to_num(P_p))
    print(P_new)

    




    


    """
    initial_estimate = 68
    initial_estimate_error = 2
    measurements = [75, 71, 70, 74]
    measurement_error = [4, 4, 4, 4]

    kgain, est, err = matrix_kf(
                        initial_estimate, 
                        initial_estimate_error, 
                        measurements, 
                        measurement_error)


    # State Matrix Example
    # kinematic equations
    x = xo + dx*t + 0.5*d2x*t^2
    """