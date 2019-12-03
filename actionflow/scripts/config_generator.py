import numpy as np
import cv2
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import KMeans
import json
import argparse
import  sys
import  tty, termios
import rospy
from std_msgs.msg import String

av_car = []
def click_and_crop(event, x, y, flags, param):
    global av_car
    if event == cv2.EVENT_LBUTTONDOWN:
        car = get_closest((x,y), human_cars)
        av_car.append(car)
        human_cars.remove(car)
        cv2.circle(calibrate_im, (x,y), 3, (0, 0, 255))
        cv2.imshow("frame", calibrate_im)

def single_config(name, tag, plan, coordinate):
    config = {
        'name': name,
        'tag': tag,
        'length': 0.165,  # [m]
        'width': 0.145,  # [m]
        'structure': 'unicycle',
        'network_config': {
            'name': 'circle-v0'
        },
        'router_config': {
            'route': np.array([
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [1, 0, 0, 0],
            ]).tolist(),
        },
        'planner_config': {
            'name': plan,
            'params': {},
        },
        'controller_config': {
            'name': 'pid',
            'params': {
                'structure': 'unicycle',
            },
        },
        'state': {
            't': 0,  # [s]
            's': 0,  # Arc distance [m]
            'x': int(coordinate[0]),  # Front center x coordinate [m]
            'y': int(coordinate[1]),  # Front center y coordinate [m]
            'theta': 0,  # [rad]
            'vel': 0,  # [m/s]
            'yaw': 0,  # [rad/s]
        },
    }
    # print(type(config))
    return config

def generate_config():
    configs = {}
    for i, human in enumerate(human_cars):
        name = 'unicycle_idm_'+str(i)
        configs[name] = single_config(name, i, 'idm', human)
    for i, av in enumerate(av_car):
        name = 'av_' + str(i)
        configs[name] = single_config(name, i + len(human_cars), 'av', av)

    # print('generating')
    # print(type(configs))
    print(configs)
    with open('configs_gen.json', 'w') as outfile:
        json.dump(configs, outfile)

    # print(configs)
def get_arg():
    parser =  wee.ArgumentParser()
    parser.add_argument("-n","--car_num", type=int, help="number of total cars")
    return parser

def similarity(point_1, point_2):
    a = np.array(point_1)
    b = np.array(point_2)
    dist = np.linalg.norm(a - b)
    return dist

def get_closest(point, list):
    find_min = lambda point_1: lambda point_2: similarity(point_1, point_2)
    closest_point = min(list, key=find_min(point))
    return closest_point

def unwarp(img, src, dst):
    h, w = img.shape[:2]
    # use cv2.getPerspectiveTransform() to get M, the transform matrix, and Minv, the inverse
    M = cv2.getPerspectiveTransform(src, dst)
    # use cv2.warpPerspective() to warp your image to a top-down view
    warped = cv2.warpPerspective(img, M, (800, 800), flags=cv2.INTER_LINEAR)
    return warped, M

def ts(num, scale):
    return round(num * scale)

def rgb_bin(img):
    Grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(Grayimg, 12, 255, cv2.THRESH_BINARY)
    return thresh

def get_pxl_ls(img):
    binary = rgb_bin(img)
    indx = np.where(binary != [0])
    X = list(zip(indx[0], indx[1]))
    X = np.array(X)
    return X

def get_center(X):
    kmeans = KMeans(n_clusters=cluster)
    kmeans.fit(X)
    y_kmeans = kmeans.predict(X)
    centers = kmeans.cluster_centers_
    if show:
        plt.scatter(X[:, 0], X[:, 1], s=10)
        plt.scatter(X[:, 0], X[:, 1], c=y_kmeans, s=10, cmap='viridis')
        plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5)
        plt.show()
    return centers

if __name__ == "__main__":
    # print("hhhhhhhhhhhhhhhhhhh")

    # total car number n, select k cars.
    # select any number.
    # av_car = []
    scale = 1
    # tran_scale
    show = False
    # cap = cv2.VideoCapture('/Users/teee/local_desktop/car_pixl/video_map_mini.mov')
    # cap = cv2.VideoCapture(2)

    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # writer = cv2.VideoWriter('output.mp4', fourcc, 15.0, (800, 800))
    # timestamps = [cap.get(cv2.CAP_PROP_POS_MSEC)]
    time_list = []
    _, im = cap.read()

    timestamps.append(cap.get(cv2.CAP_PROP_POS_MSEC))
    args = get_arg().parse_args()
    cluster = args.car_num

    width = int(im.shape[0] * scale)
    height = int(im.shape[1] * scale)

    im = cv2.resize(im, (height, width))

    w, h = im.shape[0], im.shape[1]

    X13 = (ts(188, scale), ts(249, scale))
    X21 = (ts(635, scale), ts(107, scale))
    X42 = (ts(1106, scale), ts(261, scale))
    X34 = (ts(736, scale), ts(680, scale))
    src = np.float32([X13,
                    X21,
                    X42,
                    X34])

    dst = np.float32([(ts(0, scale), ts(0, scale)),
                    (ts(800, scale), ts(0, scale)),
                    (ts(800, scale), ts(800, scale)),
                    (ts(0, scale), ts(800, scale))])

    calibrate_im, _ = unwarp(im, src, dst)
    calibrate_im = cv2.GaussianBlur(calibrate_im, (3, 3), 0)
    calibrate_im = calibrate_im[:ts(800, scale), :ts(800, scale), :]
    hsv = cv2.cvtColor(calibrate_im, cv2.COLOR_BGR2HSV)
    green_hsv_lower = (34, 25, 25)
    green_hsv_upper = (60, 255, 255)
    green_hsv_lower = (34, 25, 25)
    green_hsv_upper = (55, 255, 255)
    mask = cv2.inRange(hsv, green_hsv_lower, green_hsv_upper)
    imask = mask > 0
    green = np.zeros_like(calibrate_im, np.uint8)
    green[imask] = calibrate_im[imask]
    kernel = np.ones((5, 5), np.uint8)
    green = cv2.erode(green, kernel, iterations=1)
    green = cv2.dilate(green, kernel, iterations=1)
    X = get_pxl_ls(green)
    center_list = get_center(X)
    for center in np.ndarray.tolist(center_list):
        cv2.circle(calibrate_im, (round(center[1]), round(center[0])), 20, (0, 0, 255), 5)

    human_cars = (np.ndarray.tolist(center_list)).copy()
    # print(human_cars)

    cv2.namedWindow("frame")
    cv2.setMouseCallback("frame", click_and_crop)
    while True:
        cv2.imshow('frame', calibrate_im)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("e"):
            generate_config()
            break

        # elif key == ord("b"):
        #     break
    cv2.destroyAllWindows()
    cap.release()
    # writer.release()
