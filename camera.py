import cv2
import numpy as np


def calculate_robot_XYZ(pixel, z_obj=0, gripper='2f85', robot_capturing_coord=np.array([0,0,-37]), offset_valid=True):
    
    default_robot_hight = 37
    camera_hight = 50
    if gripper == '2f85':
        max_z_end_effector = 65.75
    elif gripper == 'Ehand':
        max_z_end_effector = 70
    else:
        raise Exception('Invalid gripper type')
        

    if offset_valid:
        robot_homming_offset = np.load('./delta_manager/parameters/homming_offset.npy')
    else:
        robot_homming_offset = np.array([0,0,0])
    
    robot_homming_offset[2] = default_robot_hight
    
    values_tr00, values_tr01, values_tr10, values_tr11, values_off0, values_off1 = np.load('./delta_manager/parameters/values.npy')
    
    H = camera_hight - z_obj + default_robot_hight + robot_capturing_coord[2]

    p00, p01, p10, p11 = np.poly1d(values_tr00), np.poly1d(values_tr01), np.poly1d(values_tr10), np.poly1d(values_tr11)
    tr_hight = np.array([[p00(H), p01(H), 0], [p10(H), p11(H), 0], [0, 0, 0]])
    offp0, offp1 = np.poly1d(values_off0),  np.poly1d(values_off1)
    offset_hight = np.array([offp0(H), offp1(H), -max_z_end_effector +camera_hight -H]) + (robot_capturing_coord + robot_homming_offset)

    robot_coordinates = np.dot([pixel[0],pixel[1],0], tr_hight) + offset_hight

    return robot_coordinates


def undistort(frame):
    
    newcameramtx = np.load('./delta_manager/parameters/newcameramtx.npy')
    mtx = np.load('./delta_manager/parameters/mtx.npy')
    dist = np.load('./delta_manager/parameters/dist.npy')

    undistorted_image = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    return undistorted_image

