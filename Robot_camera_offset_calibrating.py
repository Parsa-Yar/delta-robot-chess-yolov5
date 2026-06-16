import cv2
import numpy as np

##%% camera calibration func
# you can use this part in your code:


newcameramtx = np.load('newcameramtx.npy')

mtx = np.array([ [1.2142863702506017e+03, 0., 6.3050700218912561e+02], 
                [0., 1.2214452241857912e+03, 3.9328877906750563e+02],
                [0., 0., 1.] ])

dist = np.array([ 1.2074614147117023e-01, -6.7600512594494089e-01,
        1.0726076577371024e-03, 3.0635846427998991e-03,
        1.9499668186604049e+00 ])

robot_capturing_coord = np.array([0,0,-37])


def calculate_XYZ(u,v , z_obj,offset , robot_capturing_coord = np.array([0,0,-37])):
    robot_capturing_coord_default = np.array([0,0,-37])

    Values_tr00,Values_tr01,Values_tr10,Values_tr11,Values_off0,Values_off1 = np.load('values.npy')
    
    H = 50 - z_obj + 37 + robot_capturing_coord[2]

    p00,p01,p10,p11 = np.poly1d(Values_tr00),np.poly1d(Values_tr01),np.poly1d(Values_tr10),np.poly1d(Values_tr11)
    tr_Hight = np.array([[p00(H),p01(H),0],[p10(H),p11(H),0],[0,0,0]])
    offp0,offp1 = np.poly1d(Values_off0), np.poly1d(Values_off1)
    offset_Hight = np.array([offp0(H),offp1(H),-71 +50 -H]) + (robot_capturing_coord - robot_capturing_coord_default) + offset

    regenerated_output_centered = np.dot([u,v,0],tr_Hight) + offset_Hight

    return regenerated_output_centered


##%% finding camera offset

offset = np.array([0,0,0])

cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

calibration_flag = input('Calibration or test? (C/T)')

robot_coord_flag = input('are you in [0,0,-37] position? (y/n)')
if robot_coord_flag.upper() == 'Y': input_x, input_y, input_z = 0, 0, -37
elif robot_coord_flag.upper() == 'N':
    sys_coord_input = input('enter capturing coord: (in shape x,y,z)')
    arr = sys_coord_input.split(sep=',')
    input_x, input_y, input_z = np.array([float(arr[0]),float(arr[1]),float(arr[2])])
    print('robot is in:',input_x, input_y, input_z)


def click_event(event, u, v, flags, params):
    global calibration_flag,offset
    if event == cv2.EVENT_LBUTTONDOWN:
        if calibration_flag.upper()== 'T':    offset = np.load('offset.npy')
        elif calibration_flag.upper()== 'C':    offset = np.array([0,0,0])
        # print(f'Pixel: ({u}, {v})')
        [x, y, z] = calculate_XYZ(u, v, z_obj,offset, robot_capturing_coord)
        print(f'Robot coord for clicked point: ({x:.2f}, {y:.2f}, {z:.2f})')
        if calibration_flag.upper()== 'C':
            print('should be (-14,9) make sure clicking on right spot')
            offset = [-14,9,0] - np.array([x,y,0])
            np.save('offset',offset)
            print(f'new Robot coord for the point you clicked on: ({x + offset[0]:.2f}, {y + offset[1]:.2f}, {z:.2f})')
            calibration_flag = input('Do you want to test? (T for test C for calibration)')
while True:
    _, frame = cap.read()
    # Resized_frame = cv2.resize(frame,(640,640))

    # Undistort the frame
    undist_frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    # cv2.circle(undist_frame,(300,int(400*640/480)),5,(255,0,0),-1)
    # print(np.shape(frame))
    # Set object x, y, z coordinates from GUI input
    z_obj  = 0
    robot_capturing_coord = np.array([input_x, input_y, input_z])

    # Image show
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 1080, 950)
    cv2.setMouseCallback('image', click_event)
    cv2.imshow('image', undist_frame)

    # Image save
    key_pressed = cv2.waitKey(1)
    if key_pressed == 27:   # Esc key 
        break
    elif key_pressed == ord('s'): 
        cv2.imwrite('test2.png', undist_frame)
        print('Image saved')

cap.release()
cv2.destroyAllWindows()
