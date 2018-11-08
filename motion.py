import numpy as np

# State vector is state[rPose landmarks]

# Initial number of landmarks is zero
nLandmarks = 0

# rPose: Robot's pose [x, y, alpha]
rPose = np.array([0, 0, 0]) 

# stateMean: State vector's mean
stateMean = np.zeros(3)

# stateCov: State vector's covariance matrix
stateCov = np.zeros(rMean.size, rMean.size)

# noise: Robot's Noise (to be defined)
rNoise = np.zeros(2)

# noiseCov: Covariance of noise vector
rNoiseCov = np.cov(noise)


while True:
	
	# noiseNow: Current robot's perturbation
	rNoiseNow = np.multiply(noise, np.random.randn(2))
	 