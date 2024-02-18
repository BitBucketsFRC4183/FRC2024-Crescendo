from rknnlite.api import RKNNLite
import cv2
import numpy as np
import platform
import time
from networktables import NetworkTable

# decice tree for RK356x/RK3588
DEVICE_COMPATIBLE_NODE = '/proc/device-tree/compatible'

CLASSES = ("note", "robot")

def get_host():
    # get platform and device type
    system = platform.system()
    machine = platform.machine()
    os_machine = system + '-' + machine
    if os_machine == 'Linux-aarch64':
        try:
            with open(DEVICE_COMPATIBLE_NODE) as f:
                device_compatible_str = f.read()
                if 'rk3588' in device_compatible_str:
                    host = 'RK3588'
                elif 'rk3562' in device_compatible_str:
                    host = 'RK3562'
                else:
                    host = 'RK3566_RK3568'
        except IOError:
            print('Read device node {} failed.'.format(DEVICE_COMPATIBLE_NODE))
            exit(-1)
    else:
        host = os_machine
    return host

RK3588_RKNN_MODEL = 'models/note-model.rknn'

IMG_SIZE = (640, 640)
OBJ_THRESH = 0.25
NMS_THRESH = 0.45

# calibration camera data for Microsoft® LifeCam HD-3000, cx and cy rescaled to (640, 640)
CAM_MATRIX = np.array([
    [1135.3910661264101, 0, 333.331134567], # fx, 0, cx
    [0, 1132.4782732619462, 280.854904434], # 0, fy, cy
    [0, 0, 1]
])
DIST_COEFFS = np.array([
    0.12638832197955957,
    -0.9622506244021928,
    -0.0031641394668092838,
    0.000391140290407239,
    1.5777235084108012])
CAM_OFFSET = (0, 12) # 2d offset of camera and robot centroid in inches

def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with object threshold.
    """
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score* box_confidences >= OBJ_THRESH)
    scores = (class_max_score* box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores

def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.
    # Returns
        keep: ndarray, index of effective boxes.
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep

def softmax(x, axis=None):
    x = x - x.max(axis=axis, keepdims=True)
    y = np.exp(x)
    return y / y.sum(axis=axis, keepdims=True)

def dfl(position):
    # Distribution Focal Loss (DFL)
    n,c,h,w = position.shape
    p_num = 4
    mc = c//p_num
    y = position.reshape(n,p_num,mc,h,w)
    y = softmax(y, 2)
    acc_metrix = np.array(range(mc), dtype=float).reshape(1,1,mc,1,1)
    y = (y*acc_metrix).sum(2)
    return y


def box_process(position):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE[1]//grid_h, IMG_SIZE[0]//grid_w]).reshape(1,2,1,1)

    position = dfl(position)
    box_xy  = grid +0.5 -position[:,0:2,:,:]
    box_xy2 = grid +0.5 +position[:,2:4,:,:]
    xyxy = np.concatenate((box_xy*stride, box_xy2*stride), axis=1)

    return xyxy

def post_process(input_data):
    boxes, scores, classes_conf = [], [], []
    defualt_branch=3
    pair_per_branch = len(input_data)//defualt_branch
    # Python 忽略 score_sum 输出
    for i in range(defualt_branch):
        boxes.append(box_process(input_data[pair_per_branch*i]))
        classes_conf.append(input_data[pair_per_branch*i+1])
        scores.append(np.ones_like(input_data[pair_per_branch*i+1][:,:1,:,:], dtype=np.float32))

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0,2,3,1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    # filter according to threshold
    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    # nms
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores

def draw(image, boxes, scores, classes):

    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = [int(_b) for _b in box]

        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
        cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                    (top, left - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)

def print_outputs(boxes, classes, scores):
    if boxes is not None:
        for box, score, cl in zip(boxes, scores, classes):
            top, left, right, bottom = [int(_b) for _b in box]
            x_mid = (top + right) / 2
            y_mid = (left + bottom) / 2
            print("%s @ (%d %d) %f" % (CLASSES[cl], (x_mid, y_mid, score)))
    else:
        print("No notes detected")


# returns coordinate of closest box
def location_data(boxes):
    left, top, right, bottom = [int(_b) for _b in boxes[0]]

    for box in boxes:
        new_left, new_top, new_right, new_bottom = [int(_b) for _b in box]

        # find closest box to cam
        if new_bottom < bottom:
            left, top, right, bottom = new_left, new_top, new_right, new_bottom

    return (left, top, right, bottom)


if __name__ == '__main__':

    # create networktable to send coordinate data
    nt = NetworkTable()

    cam = cv2.VideoCapture(0)
    # create video output for testing
    #fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    #video = cv2.VideoWriter('video.mp4', fourcc, 24, IMG_SIZE)

    # Get device information
    host_name = get_host()
    if host_name == 'RK3588':
        rknn_model = RK3588_RKNN_MODEL
    else:
        print("This demo cannot run on the current platform: {}".format(host_name))
        exit(-1)

    rknn_lite = RKNNLite()

    # Load RKNN model
    print('--> Load RKNN model')
    ret = rknn_lite.load_rknn(rknn_model)
    if ret != 0:
        print('Load RKNN model failed')
        exit(ret)
    print('done')

    # Init runtime environment
    print('--> Init runtime environment')
    # run on RK356x/RK3588 with Debian OS, do not need specify target.
    if host_name == 'RK3588':
        # For RK3588, specify which NPU core the model runs on through the core_mask parameter.
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
    else:
        ret = rknn_lite.init_runtime()
    if ret != 0:
        print('Init runtime environment failed')
        exit(ret)
    print('done')

    print('--> Running model')

    frames, loopTime, initTime = 0, time.time(), time.time()

    while True:
        frames += 1

        # preprocessing and capturing image with cam
        ret, frame = cam.read()
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, IMG_SIZE)
        img = np.expand_dims(img, 0)

        # Inference
        outputs = rknn_lite.inference(inputs=[img])

        # Show the classification results
        boxes, classes, scores = post_process(outputs)
        detected = boxes is not None

        # 2d pose of the robot, y is axis from robot towards note in front, x is side to side
        x, y = 0, 0

        if detected:
            left, top, right, bottom = location_data(boxes) # middle of 2d

            # clockwise from bottom left, units are inches
            object_points = np.array([
                [0, 0, 0],
                [0, 14, 0],
                [14, 14, 0],
                [14, 0, 0]
            ], dtype=np.float64)

            image_points = np.array([
                [left, bottom],
                [left, top],
                [right, top],
                [right, bottom]
            ], dtype=np.float64)

            # default initial values for rvec and tvec
            rvec_init = np.zeros((3, 1), dtype=np.float64)
            tvec_init = np.zeros((3, 1), dtype=np.float64)

            success, rvec, tvec = cv2.solvePnP(object_points, image_points, CAM_MATRIX, DIST_COEFFS, rvec_init, tvec_init, False)

            if (success):
                rmat = cv2.Rodrigues(rvec)[0] # rotation matrix
                camera_position = -np.dot(rmat.T, tvec)

                x = camera_position[0]
                y = camera_position[1]

                print(f"Camera pose: {x}, {y}")


        # publish data using networktables
        nt.periodic(x, y, detected)

        # print_outputs(boxes, classes, scores)

        # draw the boxes and make video
        # if boxes is not None:
        #     draw(img[0], boxes, scores, classes)
        #     video.write(cv2.cvtColor(img[0], cv2.COLOR_RGB2BGR))
        #     cv2.imwrite("output_image.jpg", cv2.cvtColor(img[0], cv2.COLOR_RGB2BGR))

        # calculate average fps every 30 frames
        if frames % 30 == 0:
            fps = 30 / (time.time() - loopTime)
            # print(f"FPS: {fps: .3f}")
            loopTime = time.time()

        # TODO - remove this in prod
        if frames > 300:
            break

    # Overall average fps
    fps = frames / (time.time() - initTime)
    # print(f"Overall fps: {fps: .3f}")

    nt.close()
    cam.release()
    #video.release()
    rknn_lite.release()
    print('done')