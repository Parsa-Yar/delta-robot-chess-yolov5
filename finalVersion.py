
# import torch
import serial
import cv2
import numpy as np
# from ultralytics import YOLO
import client
import serial
import time
import pandas as pd
import torch
from delta_manager import DeltaManager

import camera

placed_counters = {
    'WR': 0, 'BR': 0, 'WKN': 0, 'BKN': 0, 'WB': 0, 'BB': 0,
    'WQ': 0, 'BQ': 0, 'WK': 0, 'BK': 0, 'WP': 0, 'BP': 0
}

PIECE_HEIGHTS = {
    'WR': 3.9, 'BR': 3.9,
    'WKN': 4.4, 'BKN': 4.4,
    'WP': 3.2, 'BP': 3.2,
    'WB': 4.8, 'BB': 4.8,
    'WQ': 6.4, 'BQ': 6.4,
    'WK': 6.4, 'BK': 6.4,
}

PIECE_PLACEMENTS = {
    'WR': ['a1', 'h1'],
    'BR': ['a8', 'h8'],
    'WKN': ['b1', 'g1'],
    'BKN': ['b8', 'g8'],
    'WB': ['c1', 'f1'],
    'BB': ['c8', 'f8'],
    'WQ': ['d1'],
    'BQ': ['d8'],
    'WK': ['e1'],
    'BK': ['e8'],
    'WP': ['a2', 'b2', 'c2', 'd2', 'e2', 'f2', 'g2', 'h2'],
    'BP': ['a7', 'b7', 'c7', 'd7', 'e7', 'f7', 'g7', 'h7'],
}


def findBoard():
    #Getting the initial frame
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while (cap.isOpened()):

        _ , captured_frame = cap.read()
        Resized_frame = cv2.resize(captured_frame,(640,640))
        cv2.imshow("Initial Frame", Resized_frame)
        key = cv2.waitKey(0) & 0xFF

        if key == ord('q') or key == ord('Q'):
            break

    firstImage = Resized_frame.copy()
    cv2.imshow("Initial Frame", Resized_frame)
    print("Click 's' to save the photo and continue")

    key = cv2.waitKey(0)


    if key == ord('s') or key == ord('S'): 
        cv2.imwrite('initialFrame.jpg', captured_frame)
    
    cv2.destroyAllWindows()
    
    #Cropping image 
    height, width , _ = np.shape(firstImage)
    mask0 = np.zeros(firstImage.shape[:2], np.uint8)
    mask0[:, 0 : int (0.6*width) ] = 255
    cropped_img = cv2.bitwise_and(firstImage, firstImage, mask = mask0)
    
    cv2.imshow("cropped image", cropped_img)
    print("Click 's' to save the photo and continue")
    key = cv2.waitKey(0)

    

    if key == ord('s') or key == ord('S'): 
        cv2.imwrite('cropped_img.jpg', cropped_img)
    
    cv2.destroyAllWindows()


    #Color Filtering
    lower = np.array([125, 0, 0])
    upper = np.array([255, 113, 139])
    mask = cv2.inRange(cropped_img, lower, upper)
    contours, hierarchies = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    x,y = np.shape(np.where(mask!=0))
    temp = np.array(np.where(mask!=0))
    temp.reshape(y,x)
    circle_corner = []
    ## finding circles
    
    for i in contours:
        M = cv2.moments(i)
        
        if M['m00'] > 100:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if (cx and cy):
                circle_corner.append([cx,cy])
                
    ## sorting cricle centers
    circle_corner = np.array(circle_corner)
    circle_corner2 = np.unique(circle_corner,axis=0)
    if (len(circle_corner2) !=4) :
        print("Did not detect 4 circles")
        exit()
    temp = circle_corner2[:,1].argsort()
    sorted_cricles = circle_corner2[temp]
    upper_part_cricles = sorted_cricles[[0, 1]]
    lower_part_cricles = sorted_cricles[[2, 3]]
    temp = upper_part_cricles[:, 0].argsort()

    upper_part_cricles = upper_part_cricles[temp]
    upper_left = upper_part_cricles[0]
    upper_right = upper_part_cricles[-1]

    temp = lower_part_cricles[:,0].argsort()
    lower_part_cricles = lower_part_cricles[temp]
    lower_left = lower_part_cricles[0]
    lower_right = lower_part_cricles[-1]

    ##Wraping perpective
    
    dest_width = 640
    dest_height = 640
    dstPts = [[0, 0], [dest_width, 0], [dest_width, dest_height], [0, dest_height]]
    intersect_pts = np.array([upper_left, upper_right, lower_right, lower_left])

    intersect_pts = np.float32(intersect_pts)
    matrix = cv2.getPerspectiveTransform(np.float32(intersect_pts), np.float32(dstPts))

    ##finding corner of the chess board
    wrapped_pic = cv2.warpPerspective(cropped_img, matrix, (int(dest_width), int(dest_height)))
    gray_wrapped = cv2.cvtColor(wrapped_pic, cv2.COLOR_BGR2GRAY)
    points = cv2.goodFeaturesToTrack(gray_wrapped, 150, 0.0001, 10)
    points = np.int0(points)
    points = points.reshape(150, 2)
    ## finding each point distance from x = 0 , y = 0
    point_distances = (points[:, 0] **2 + points[:, 1]**2)**0.5
    ## finding the location of the nearest and furthest point from x = 0, y = 0
    x_min, y_min = points[point_distances == point_distances.min()][0]
    x_max, y_max = points[point_distances == point_distances.max()][0]
    ## calculating diagonal length and cell length
    diag = ( (x_max - x_min)**2 + (y_max - y_min)**2 )**0.5
    cell_length = (diag/np.sqrt(2)) / 8

    return(matrix,cell_length)

def reversePerspective(mat,*cord):
    _, IM = cv2.invert(mat)
    x1 = cord[0][0]
    y1 = cord[0][1]
    cord = [x1, y1] + [1]
    P = np.float32(cord)

    x, y, z = np.dot(IM, P)
    new_x = int(x/z)
    new_y = int(y/z)
    
    return (new_x,new_y)


def cell2Cord(cell_Name,cell_Length):
    x_cord = ord(cell_Name[0].lower()) - 96 - 0.5
    y_cord = 9 - int(cell_Name[1]) - 0.5
    return np.array([round(x_cord*cell_Length), round(y_cord*cell_Length)])    

def graspObjects(objects, matrix, cell_Length):## [[names], [[x1,y1],[x2,y2]],[angles]]
    
    delta = DeltaManager()
    delta.connect_gripper()

    names = objects[0]
    centers = objects[1]
    delta.home_gripper()
    delta.wait_till_done()

    client.order("command", "forward")

    robot_capturing_coord = [ client.Result[1], client.Result[2], client.Result[3] ]
    Gripper = '2f85'
    for i in range(0,len(names)):
        piece_name = names[i]
        piece_height = PIECE_HEIGHTS.get(piece_name)

        if piece_height is None:
            print(f"Warning: Unknown piece '{piece_name}' with no defined height. Skipping.")
            continue

        [x_w, y_w, z_w] = camera.calculate_robot_XYZ((centers[i][0], centers[i][1]), piece_height, gripper = Gripper, robot_capturing_coord = robot_capturing_coord)

        client.order("move", f"{x_w},{y_w},{z_w+15}")
        if client.Result == "success":
            delta.home_gripper()
            delta.wait_till_done()

        client.order("move", f"{x_w},{y_w},{z_w}")
        if client.Result == "success":
            delta.close_gripper()
            delta.wait_till_done()

        client.order("move", f"{x_w},{y_w},{z_w+15}")
        if client.Result == "success":
            target_squares = PIECE_PLACEMENTS.get(piece_name)
            if not target_squares:
                print(f"Warning: No placement rule for piece '{piece_name}'. Dropping piece and continuing.")
                delta.open_gripper()
                delta.wait_till_done()
                continue

            piece_count = placed_counters.get(piece_name, 0)
            if piece_count >= len(target_squares):
                print(f"Warning: Already placed all {len(target_squares)} pieces of type '{piece_name}'. Dropping extra piece.")
                delta.open_gripper()
                delta.wait_till_done()
                continue

            target_cell_name = target_squares[piece_count]
            u,v = reversePerspective(matrix,cell2Cord(target_cell_name, cell_Length))
            [x_w_target,y_w_target,z_w_target] = camera.calculate_robot_XYZ((u, v), piece_height, gripper = Gripper, robot_capturing_coord = robot_capturing_coord)

            client.order("move", f"{x_w_target},{y_w_target},{z_w_target+15}")
            if client.Result == "success":
                client.order("move", f"{x_w_target},{y_w_target},{z_w_target}")
                placed_counters[piece_name] += 1

        if client.Result == "success":
            delta.open_gripper()
            delta.wait_till_done()

        client.order("move", f"{x_w_target},{y_w_target},{z_w_target + 15}")
        if client.Result == "success":
            delta.home_gripper()
            delta.wait_till_done()

def CheckPieces(res,isWhite):
    check = True

    if isWhite:
        if (res['WR'] != 2) or (res['WKN'] != 2) or (res['WB'] != 2) or (res['WP'] != 2) or (res['WK'] != 1) or (res['WQ'] != 1) :
            check = False
        return check
    
    if not(isWhite):
        if (res['BR'] != 2) or (res['BKN'] != 2) or (res['BB'] != 2) or (res['BP'] != 8) or (res['BK'] != 1) or (res['BQ'] != 1) :
            check = False
        return(check)



def CaptureWhitePieces(DetectionModel):

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print(" Click 'q' to exit detection and continue")

    DetectionModel.conf = 0.5
    while (cap.isOpened()):
        _ , WhitPieces_frame = cap.read()
        Resized_frame = cv2.resize(WhitPieces_frame,(640,640))
        
        DetectionModel.conf = 0.5
        results = DetectionModel(Resized_frame)
        cv2.imshow("Detection",np.squeeze(results.render()))
        key = cv2.waitKey(1) & 0xFF
        
        PiecesNum = results.pandas().xyxy[0].value_counts('name')
        if CheckPieces(PiecesNum, isWhite = True):
            break
        
        if key == ord('q') or key == ord('Q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    pieces_df = results.pandas().xyxy[0]

    # pieces_df = pd.DataFrame(results[0].boxes.data.cpu().detach().numpy(), columns=["x1", 'y1', 'x2', 'y2', 'precision', 'typeNo'])
    ClassList = list(pieces_df['name'])

    # x1_y1_List = [ [x1,y1] for x1,y1 in list(zip((pieces_df['x1']),(pieces_df['y1'])))]
    # x2_y2_List = [ [x2, y2] for x2, y2 in list(zip((pieces_df['x2']),(pieces_df['y2'])))]

    x_centers = (pieces_df['xmin'] + pieces_df['xmax'])/2
    y_centers = (pieces_df['ymin'] + pieces_df['ymax'])/2
    x_y_List = [ [x1,y1] for x1,y1 in list(zip((x_centers),(y_centers)))]
    return (ClassList, x_y_List)


def CaptureBlackPieces(DetectionModel):
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_GAMMA,229)
    cap.set(cv2.CAP_PROP_BRIGHTNESS,40)  
    print(" Click 'q' to exit detection and continue")
    

    while (cap.isOpened()):
        _ , BlackPieces_frame = cap.read()
        
        Resized_frame = cv2.resize(BlackPieces_frame,(640,640))
        
        
        DetectionModel.conf = 0.5
        results = DetectionModel(Resized_frame)
        cv2.imshow("Detection",np.squeeze(results.render()))

        key = cv2.waitKey(1) & 0xFF

        PiecesNum = results.pandas().xyxy[0].value_counts('name')

        if CheckPieces(PiecesNum, isWhite = False):
            break

        if key == ord('q') or key == ord('Q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    # pieces_df = pd.DataFrame(results[0].boxes.data.cpu().detach().numpy(), columns=["x1", 'y1', 'x2', 'y2', 'precision', 'typeNo'])
    # pieces_df["ClassName"] = pieces_df['typeNo'].map(results[0].names)
    pieces_df = results.pandas().xyxy[0]

    # pieces_df = pd.DataFrame(results[0].boxes.data.cpu().detach().numpy(), columns=["x1", 'y1', 'x2', 'y2', 'precision', 'typeNo'])
    ClassList = list(pieces_df['name'])

    # x1_y1_List = [ [x1,y1] for x1,y1 in list(zip((pieces_df['x1']),(pieces_df['y1'])))]
    # x2_y2_List = [ [x2, y2] for x2, y2 in list(zip((pieces_df['x2']),(pieces_df['y2'])))]

    x_centers = (pieces_df['xmin'] + pieces_df['xmax'])/2
    y_centers = (pieces_df['ymin'] + pieces_df['ymax'])/2
    x_y_List = [ [x1,y1] for x1,y1 in list(zip((x_centers),(y_centers)))]

    return (ClassList, x_y_List)




def main():
    
    

    print("Finding chess board: ")
    matrix, cell_Length = findBoard()


    print("Capturing photo for white Pieces: ")
    # model = YOLO('detect/train5/weights/best.pt')
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='Yolov5LatestRun/best.pt', force_reload=True)


    W_ClassList, W_x_y_List = CaptureWhitePieces(model)

    graspObjects([W_ClassList, W_x_y_List], matrix ,cell_Length)

    # grasp and arrange
    ## robot goes to Home position
    command = input("place the black pieces and press enter: ")

    CaptureBlackPieces()
    B_ClassList, B_x_y_List = CaptureBlackPieces(model)
    graspObjects([B_ClassList, B_x_y_List], matrix ,cell_Length)

    #grasp and arrange
if __name__ == '__main__':
    main()