import numpy as np


def invertMatrix(M):
    #steps to invert the matrix
    invRotM = M[0:3,0:3].T #take the transpose of the rotation matrix
    negTranM = -M[0:3,-1].reshape(3,1) #take the negative of the translation vector in the transformation matrix
    

    #create the entire inverted transformation matrix

    #rotation component
    invRotMWhole = np.eye(4)    
    invRotMWhole[0:3,0:3] = invRotM
    
    #translation component
    invTranMWhole = np.eye(4)
    invTranMWhole[0:3,3:] = negTranM

    #multiply the two matrices together
    invM = np.matmul(invRotMWhole,invTranMWhole)

    return invM

def buildRotationMatrix ():
    #transform from camera into measurement plane

    c1 = np.cos(np.deg2rad(yaw)) #cos(yaw)
    c2 = np.cos(np.deg2rad(p)) #cos(yaw)
    c3 = np.cos(np.deg2rad(r)) #cos(yaw)

    s1 = np.sin(np.deg2rad(yaw)) #cos(yaw)
    s2 = np.sin(np.deg2rad(p)) #cos(yaw)
    s3 = np.sin(np.deg2rad(r)) #cos(yaw)

    #create transformation matrix - based on PLC code
    #M = np.array([[c1*c2*c3 - s1*s3,    -c3*s1 - c1*c2*s3,  c1*s2,  x],
    #            [c1*s3 + c2*c3*s1,      c1*c3 - c2*s1*s3,   s1*s2,  y],
    #            [-c3*s2,                s2*s3,              c2,     z ],
    #            [0.0,                   0.0,                0.0,    1.0]])

    #create transformation matrix - based on Wikipedia
    M = np.array([[c1*c2,      c1*s2*s3-s1*c3,  c1*s2*c3+s1*s3,  x],
                  [s1*c2,      s1*s2*s3+c1*c3,  s1*s2*c3-c1*s3,  y],
                  [-s2,                 c2*s3,           c2*c3,  z],
                  [0.0,                   0.0,             0.0,1.0]])

    return M

def cameraMatrix():
    #information about the area scan camera
    focus = 0.0276199
    kappa = -12.8906
    Sx = 5.00038e-6
    Sy = 5e-6
    Cx = 1286.48
    Cy = 997.563
    ImageWidth = 2560
    ImageHeight = 2048




if __name__ == "__main__":



    frame = 2

    if frame == 1:
        #Transformation from robot coordinates into tote frame. This is the
        #position of the tote in the robot coordinate frame.
        
        x = 553 #mm
        y = -485
        z = 0
        yaw = 0
        p = 180 #deg
        r = 0
    elif frame == 2:
        #transform from area scan camera coordinates into robot base coordinates
        x = 363 #mm
        y = -354
        z = 839
        r = 179.3
        p = 0.3735 #deg
        yaw = 0.5229

        #measurement plane in cam pose
        x2 = -3.4
        y2 = -2.4 
        z2 = 839
        r2 = 0.689
        p2 = 359
        yaw2 = 270





    """
    information received from area scan in Sheet (Stack)

    This information appears to be in area scan camera coordinate frame! 
    NOT given in robot base coordinate frame.
    """
    width_as = 1683
    length_as = 1310
    theta_as = 3.87
    centerX_as = 1322
    centerY_as = 1051
    
    pxToMm = 6.43 #pixel to mm conversion

    centerXmm_as = centerX_as/pxToMm
    centerYmm_as = centerY_as/pxToMm
    theta = 3.87 #degrees
    thetaX_as = np.sin(np.deg2rad(theta))
    thetaY_as = np.cos(np.deg2rad(theta))

    # robot given coordinates. (originally populated from GVL_VisionServer) This is used to validate the transformation
    # into tote frame coordinates. This is achieved by using the invM matrix.
    centerX = 239
    centerY = -453

    centerPt_as = np.array([[centerXmm_as],
                            [centerYmm_as],
                            [0],
                            [1]])
    theta_as = np.array([[thetaX_as],
                        [thetaY_as],
                        [0],
                        [1]])
    centerPt_robot = np.matmul(invM,centerPt_as)
    theta_robot = np.matmul(invM,theta_as)
    width_robot = width_as/pxToMm
    length_robot = length_as/pxToMm

    print(centerPt_robot)
    print(theta_robot)
