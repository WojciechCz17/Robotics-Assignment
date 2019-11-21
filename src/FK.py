import image1.py

def transformation(DH_params):
    # DH_params = [alpha, a, d, theta]

    trans_d = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, DH_params[2]],
                        [0, 0, 0, 1]])

    trans_a = np.array([[1, 0, 0, theta[1]],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0,0, 0, 1]])
    
    theta = DH_params[3]
    trans_theta = np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                            [math.sin(theta), math.cos(theta), 0, 0],
                            [0, 0, 1, 0],
                            [0, 0,0, 1]])

    alpha = DH_params[0]
    trans_alpha = np.array([[1, 0, 0, 0],
                            [0, math.cos(alpha), -math.sin(alpha), 0],
                            [0, math.sin(alpha), math.cos(alpha), 0],
                            [0, 0, 0, 1]])

    T = np.dot(trans_d, np.dot(trans_theta, np.dot(trans_a, trans_alpha)))
    
    return T

def detect_end_effector(self,image):
    a = self.pixel2meter(image)
    endPos = a * (self.detect_yellow(image) - self.detect_red(image))
    return endPos

def main():

    

    