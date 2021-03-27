# Single Measurement Kalman Filter
# for use of determining a single value from a
# set of measurements using kalman filtering
# author: aplourde

def kalman_gain(estimatedError, dataError):
    kgain = estimatedError / (estimatedError + dataError)
    if kgain < 0 or kgain > 1:
        print("Error! KGAIN outside of desired range")
    return kgain

def calculateNewError(kgain, previousEstimateError):
    newError = (1 - kgain) * previousEstimateError
    return newError

def current_estimate(kgain, previousEstimate, measuredVal, previousEstimateError):
    currentEstimate = previousEstimate + kgain*(measuredVal - previousEstimate)
    newError = calculateNewError(kgain, previousEstimateError)
    return currentEstimate, newError

def single_measurement_kf(est, est_err, measurements, mea_err):
    for i in range(len(measurements)):
        kgain = kalman_gain(est_err, mea_err[i])
        est, est_err = current_estimate(kgain, est, measurements[i], est_err)
        print(kgain, est, est_err)
    return kgain, est, est_err

if __name__ == "__main__":

    initial_estimate = 68
    initial_estimate_error = 2
    measurements = [75, 71, 70, 74]
    measurement_error = [4, 4, 4, 4]

    kgain, est, err = single_measurement_kf(
                        initial_estimate, 
                        initial_estimate_error, 
                        measurements, 
                        measurement_error)
    

