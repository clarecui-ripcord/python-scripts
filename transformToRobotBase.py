import numpy as np

def invertMat3x3(M):
    '''
    Takes a 3x3 matrix and inverts it
    '''
    invRotM = np.linalg.inv(M[0:2,0:2])
    negTranM = -M[0:2,-1].reshape(2,1)

    #create the entire inverted matrix
    #translation component
    invTranM = np.matmul(invRotM,negTranM)

    #populate the final matrix
    invM = np.eye(3)
    invM[0:2,0:2] = invRotM
    invM[0:2,2:] = invTranM

    return invM

def invertMat4x4(M):
    '''
    Takes a 4x4 matrix and inverts it
    '''
    #steps to invert the matrix
    invRotM = np.linalg.inv(M[0:3,0:3]) #take the inversion of the rotation matrix
    negTranM = -M[0:3,-1].reshape(3,1) #take the negative of the translation vector in the transformation matrix
    
    #create the entire inverted matrix
    #translation component
    invTranM = np.matmul(invRotM,negTranM)

    #populate the final matrix
    invM = np.eye(4)    
    invM[0:3,0:3] = invRotM
    invM[0:3,-1:] = invTranM

    return invM

def buildTransformMat (x,y,z,roll,pitch,yaw):
    '''
    Takes x, y, z in m
    Takes yaw, pitch, roll in deg
    '''
    #transform from camera into measurement plane

    c1 = np.cos(np.deg2rad(yaw)) 
    c2 = np.cos(np.deg2rad(pitch)) 
    c3 = np.cos(np.deg2rad(roll)) 

    s1 = np.sin(np.deg2rad(yaw)) 
    s2 = np.sin(np.deg2rad(pitch)) 
    s3 = np.sin(np.deg2rad(roll)) 

    #create transformation matrix - based on PLC code
    #M2 = np.array([[c1*c2*c3 - s1*s3,    -c3*s1 - c1*c2*s3,  c1*s2,  x],
    #            [c1*s3 + c2*c3*s1,      c1*c3 - c2*s1*s3,   s1*s2,  y],
    #            [-c3*s2,                s2*s3,              c2,     z ],
    #            [0.0,                   0.0,                0.0,    1.0]])

    #create transformation matrix - based on Wikipedia
    #M = np.array([[c1*c2,      c1*s2*s3-s1*c3,  c1*s2*c3+s1*s3,  x],
    #              [s1*c2,      s1*s2*s3+c1*c3,  s1*s2*c3-c1*s3,  y],
    #              [-s2,                 c2*s3,           c2*c3,  z],
    #              [0.0,                   0.0,             0.0,1.0]])

    Rx = np.array([[1, 0, 0],
                [0, c3, -s3],
                [0, s3, c3]])

    Ry = np.array([[c2,0,s2],
                [0,1,0],
                [-s2,0,c2]])    

    Rz = np.array([[c1,-s1,0],
                [s1, c1, 0],
                [0, 0, 1]])

    R = np.matmul(Rx,np.matmul(Ry,Rz))

    M = np.eye(4,4)
    M[0:3,0:3] = R
    M[0,3] = x
    M[1,3] = y
    M[2,3] = z
    #import ipdb; ipdb.set_trace()

    return M

def cameraMatrix():
    #information about the area scan camera at ADT13
    
    '''
    #ADT13
    focus = 0.0261026 #m
    kappa = -117.177
    Sx = 5e-6#.00051e-6 #m per pixel
    Sy = 5e-6
    Cx = 1244.66
    Cy = 1047.35
    ImageWidth = 2560
    ImageHeight = 2048
    
    '''
    #ADT2
     #with a distortion coefficient
    focus = 0.0257868 #m
    kappa = -111.63
    Sx = 5.00363e-6 #m per pixel
    Sy = 5e-6
    Cx = 1318.76
    Cy = 1077.48
    ImageWidth = 2560
    ImageHeight = 2048
    
    '''
    #without distortion coefficient (provided by Halcon)
    focus = 0.0257868 #m
    kappa = 0
    Sx = 5.02671e-6 #m per pixel
    Sy = 5.0148e-6
    Cx = 1319.12
    Cy = 1077.8
    ImageWidth = 2560
    ImageHeight = 2048
    '''
    fx = focus/Sx
    fy = focus/Sy

    M = np.array([[fx,  0, Cx],
                  [ 0, fy, Cy],
                  [ 0,  0,  1]])

    inverseM = 1/(fx*fy)*np.array([[fy,0,-Cx*fy],
                            [0,fx,-Cy*fx],
                            [0,0,fx*fy]])

    return M, inverseM


if __name__ == "__main__":

    '''
    https://ripcord.atlassian.net/wiki/spaces/DHA/pages/2517467211/Robot+Communication+Interface+Design+Detail
    
    '''
    np.set_printoptions(suppress=True)
    frame = 3

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
        M = buildTransformMat(x,y,z,roll,pitch,yaw)

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

    elif frame == 2: #ADT13
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

        #undistort the point
        kappa = -117.177
        Cx = 1244.66
        Cy = 1047.35

        centerX_asUndistort = centerX_as/(kappa*((centerX_as-Cx)**2 + (centerY_as-Cy)**2) + 1)
        centerY_asUndistort = centerY_as/(kappa*((centerX_as-Cx)**2 + (centerY_as-Cy)**2) + 1)

        #centerX_asUndistort = centerX_as/(kappa*((centerX_as)**2 + (centerY_as)**2) + 1)
        #centerY_asUndistort = centerY_as/(kappa*((centerX_as)**2 + (centerY_as)**2) + 1)


        p_as = np.array([[centerX_as*z2],
                  [centerY_as*z2],
                  [z2],
                  [1]])


        K,invK2 = cameraMatrix()
        invK = invertMat3x3(K)
        invK_homo = np.eye(4,4)
        invK_homo[0:3,0:3] = invK
        
        focus = 0.0261026


        ####### Comparing with Halcon results
        p_as2 = np.array([[centerX_as],
                        [centerY_as],
                        [1],
                        [1]])
        projected = np.matmul(invK_homo,p_as2)*focus

        q = np.array([[-.00385112],
                        [0.0026977],
                        [0.0261026],
                        [1]])

        ########
        
        #calculate the projected 3d point in the focal plane of the camera
        #center_proj = np.matmul(invK,p_as)

        #Form the 4x1 matrix for the 
        # center_proj = np.vstack([center_proj, 1])
        # print('Projected into image plane Center Point')
        # print(center_proj)

        T_camInBase = buildTransformMat(x,y,z,roll,pitch,yaw)
        T_baseInCam = invertMat4x4(T_camInBase)

        p_img = np.matmul(invK_homo,p_as)
        point_robot = np.matmul(T_camInBase,p_img)

        # apply the camera matrix to the transformation matrix
        #rotation_WIP = np.matmul(T_camInBase[0:3,0:3],invK2)

        #T = np.eye(4,4)
        #T[0:3,0:3] = rotation_WIP
        #T[0:3,-1:] = T_camInBase[0:3,-1].reshape(3,1)
        #point_robot = np.matmul(T,p_as)

        point_VisionServer = np.array([[0.237653],
                                    [-0.450652]])

    elif frame == 3: #ADT2
        #pose transform from area scan camera 
        #   coordinates into robot base coordinates

        x = 0.36329   #m
        y = -0.35457
        z = 1.17731

        roll =  179.335
        pitch =  0.373498 #deg
        yaw =  0.522852

        #measurement plane in cam Z
        z2 = 0.838596

        #center of rectangle coordinate info
        centerX_as = 1195.240
        centerY_as = 1078.109

        point_VisionServer = np.array([[0.34866],
                                    [-0.36423]])


        K,invK2 = cameraMatrix()
        invK = invertMat3x3(K)
        invK_homo = np.eye(4,4)
        invK_homo[0:3,0:3] = invK


        ####### Comparing with Halcon results
        focus = 0.0257868
        p_as2 = np.array([[centerX_as],
                        [centerY_as],
                        [1],
                        [1]])


        #IMPORTANT!!!
        distorted_projected = np.matmul(invK_homo,p_as2)*focus

        #undistort the point
        kappa = -111.63

        centerX_asUndistort = distorted_projected[0]/(kappa*((distorted_projected[0])**2 + (distorted_projected[1])**2) + 1)
        centerY_asUndistort = distorted_projected[1]/(kappa*((distorted_projected[0])**2 + (distorted_projected[1])**2) + 1)

        p_img2 = np.array([[centerX_asUndistort[0]/focus*z2],
                                    [centerY_asUndistort[0]/focus*z2],
                                    [z2],
                                    [1]])
        
        
        q = np.array([[0.00336335],
                    [-0.000321669],
                    [0.0257868],
                    [1]])

        ########
        

        T_camInBase = buildTransformMat(x,y,z,roll,pitch,yaw)
        T_baseInCam = invertMat4x4(T_camInBase)

        #p_img = np.matmul(invK_homo,p_as)
        point_robot = np.matmul(T_camInBase,p_img2)


        
        
    ########## PRINT EVERYTHING ##########

    print('Original point')
    print(centerX_as)
    print(centerY_as)
    print('\n')

    #print("Area Scan Point with the measurement plane applied")
    #print(p_as)
    #print('\n')

    print("Distorted Projected Point")
    print(distorted_projected)
    print('\n')

    print("Undistorted point")
    print(centerX_asUndistort)
    print(centerY_asUndistort)
    print('\n')

    print("Matrices: Camera Matrix")
    print(K)
    print('\n')

    print("Matrices: Inverse Camera Matrix created by inverting a transformation matrix normally")
    print(invK)
    print('\n')

    #print("Matrices: Inverse Camera Matrix created by inverting according to imatest.com")
    #print(invK2)
    #print('\n')

    print('Matrices: Cam In the Base Frame')
    print(T_camInBase)
    print('\n')

    #print('Matrices: Base in Cam Frame - need to determine which to use')
    #print(T_baseInCam)
    #print('\n')

    #print('Matrices: Transform multiplied to Area Scan Point')
    #print(T)
    #print('\n')

    print('3D Image point')
    print(p_img2)
    print('\n')

    print('ANSWER: Robot coordinate frame Center Point')
    print(point_robot)
    print('\n')

    print("Expected Result")

    print(point_VisionServer)

    """
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
    """

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
