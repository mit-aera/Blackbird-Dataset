#!/usr/bin/env python 

import fire
import numpy as np
import yaml


def correctTransform(array_arg):
    x,y,z,yaw = array_arg
    # IN Blackbird dataset, the transform order is reversed.
    # Proper transform is T.R (rotation before translation).
    # BlackBird Dataset has R.T (translation before rotation)
    # To fix, keep rotation as is, but fix the translation
    # T_a = R.T_b


    # Convert yaw to radians
    theta = (yaw/180.0)*np.pi

    # Normalize yaw to be positive (clockwise)
    if (theta < 0.0): theta += 2.0*np.pi

    R = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0,0,1]])
    x = np.array([[x],[y],[z]])
    x_prime = R.dot(x)
    
    np.set_printoptions(precision=6, suppress=True)

    # Convert yaw to quaternion of form (qx qy qz qw)
    quat = np.array([0, 0, np.sin(theta/2.0), np.cos(theta/2.0)])
    #print list(x_prime.flat) + list(quat) 
    print "          ros_offset: " + np.array2string(np.hstack([x_prime.flat, quat]), separator=', ') + " # x,y,z,qx,qy,qz,qw"
    #print quat
    
    


if __name__ == '__main__':
    fire.Fire(correctTransform)



