import numpy as np

def invertMat3x3(M):
    '''
    Takes a 3x3 matrix and inverts it
    '''
    invRotM = M[0:2,0:2].T
    negTranM = -M[0:2,-1].reshape(2,1)

    #create the entire inverted matrix

    #rotation component
    invRotMWhole = np.eye(3)
    invRotMWhole[0:2,0:2] = invRotM

    invTranMWhole = np.eye(3)
    invTranMWhole[0:2,2:] = negTranM

    invM = np.matmul(invRotMWhole,invTranMWhole)

    return invM

def invertMat4x4(M):
    '''
    Takes a 4x4 matrix and inverts it
    '''
    #steps to invert the matrix
    invRotM = M[0:3,0:3].T #take the transpose of the rotation matrix
    negTranM = -M[0:3,-1].reshape(3,1) #take the negative of the translation vector in the transformation matrix
    
    #create the entire inverted matrix

    #rotation component
    invRotMWhole = np.eye(4)    
    invRotMWhole[0:3,0:3] = invRotM
    
    #translation component
    invTranMWhole = np.eye(4)
    invTranMWhole[0:3,3:] = negTranM

    #multiply the two matrices together
    invM = np.matmul(invRotMWhole,invTranMWhole)

    return invM

def buildTransformMat (x,y,z,yaw,pitch,roll):
    '''
    Takes x, y, z in mm
    Takes yaw, pitch, roll in deg
    '''
    #transform from camera into measurement plane

    c1 = np.cos(np.deg2rad(yaw)) #cos(yaw)
    c2 = np.cos(np.deg2rad(pitch)) #cos(yaw)
    c3 = np.cos(np.deg2rad(roll)) #cos(yaw)

    s1 = np.sin(np.deg2rad(yaw)) #cos(yaw)
    s2 = np.sin(np.deg2rad(pitch)) #cos(yaw)
    s3 = np.sin(np.deg2rad(roll)) #cos(yaw)

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
    #information about the area scan camera at ADT13
    focus = 0.0261026
    kappa = -117.177
    Sx = 5.00051e-6 #m per pixel
    Sy = 5e-6
    Cx = 1244.66
    Cy = 1047.35
    ImageWidth = 2560
    ImageHeight = 2048

    #pxToMm = 6.43 #pixel to mm conversion

    fx = focus#/Sx
    fy = focus#/Sy

    M = np.array([[fx,  0, Cx],
                  [ 0, fy, Cy],
                  [ 0,  0,  1]])

    return M


if __name__ == "__main__":

    '''
    https://ripcord.atlassian.net/wiki/spaces/DHA/pages/2517467211/Robot+Communication+Interface+Design+Detail
    
    '''
    np.set_printoptions(suppress=True)
    frame = 2

    if frame == 1:
        '''Transformation from robot coordinates into tote frame. 
        This is the position of the point in the robot coordinate frame.'''

        # Transformation information
        x = 553 #mm
        y = -485
        z = 0
        yaw = 0
        pitch = 180 #deg
        roll = 0
        M = buildTransformMat(x,y,z,yaw,pitch,roll)

        # Point we want to transform
        point_x = 239
        point_y = -453

        point = np.array([[point_x],
                        [point_y],
                        [0],
                        [1]])

        invM = invertMat4x4(M)
        pointToteFrame = np.matmul(invM,point)

        print("original point:")
        print(point)
        print("point in tote frame coordinates")
        print(pointToteFrame)

    elif frame == 2:
        #pose transform from area scan camera 
        #   coordinates into robot base coordinates
        x = 0.361341 #m
        y = -0.360911
        z = 1.15466
        roll =  179.825
        pitch = 359.935 #deg
        yaw = 359.622

        #measurement plane in cam pose
        x2 = -0.0054729
        y2 = -0.00110888
        z2 = 0.835834
        roll2 = 0.499958
        pitch2 = 0.0188596
        yaw2 = 359.778

        """
        information received from area scan in Sheet (Stack)

        This information appears to be in area scan camera coordinate frame! 
        NOT given in robot base coordinate frame.
        """
        #width_as = 1683
        #length_as = 1310

        #orientation of staple
        theta_as = 40.7

        #center of rectangle coordinate info
        centerX_as = 476.5
        centerY_as = 1585.5


        focus = 0.0261026

        #undistort the point
        kappa = -117.177
        centerX_asUndistort = centerX_as/(kappa*(centerX_as**2 + centerY_as**2) + 1)
        centerY_asUndistort = centerY_as/(kappa*(centerX_as**2 + centerY_as**2) + 1)

        p_as = np.array([[centerX_as*z2],
                  [centerY_as*z2],
                  [z2],
                  [1]])

        invK = invertMat3x3(cameraMatrix())

        #calculate the projected 3d point in the focal plane of the camera
        #center_proj = np.matmul(invK,p_as)

        #Form the 4x1 matrix for the 
        # center_proj = np.vstack([center_proj, 1])
        # print('Projected into image plane Center Point')
        # print(center_proj)

        T_camInBase = buildTransformMat(x,y,z,roll,pitch,yaw)
        T_baseInCam = invertMat4x4(T_camInBase)


        # apply the camera matrix to the transformation matrix
        rotation_WIP = np.matmul(T_camInBase[0:3,0:3],invK)
        translation_WIP = -np.matmul(rotation_WIP,T_camInBase[0:3,-1].reshape(3,1))

        T = np.eye(4,4)
        T[0:3,0:3] = rotation_WIP
        T[0:3,-1:] = translation_WIP

        center_robot = np.matmul(T,p_as)

        print('Robot coordinate frame Center Point')
        print(center_robot)

        centerPt_as = np.array([[centerX_as],
                                [centerX_as],
                                [0],
                                [1]])

        thetaX_as = np.sin(np.deg2rad(theta_as))
        thetaY_as = np.cos(np.deg2rad(theta_as))

        theta_as = np.array([[thetaX_as],
                            [thetaY_as],
                            [0],
                            [1]])

    #pixel to mm conversion. Used for length and width and nowhere else!
    pxToMm = 6.43 

    """EXPECTED RESULTS
    Robot given coordinates. (originally populated from GVL_VisionServer) 
    We can transform into tote frame coordinates if we use frame = 1.
    This is achieved by using the invM matrix.
    """
    #width_rs = 202 #mm
    #length_rs = 259
    centerX_rs = 0.2377 
    centerY_rs = -0.4507
    theta_rs = -40.7 #degrees
