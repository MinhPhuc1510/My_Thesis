"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""


import os
import time
import argparse
import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver
from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO
import tensorflow as tf
import numpy as np
import CNN

WINDOW_NAME = 'TrtYOLODemo'
model_cnn = tf.keras.models.load_model('CNN/CNN_lane_test')
serial_port = CNN.conf_serial()



def parse_args():
    """Parse input arguments."""
    desc = ('Capture and display live camera video, while doing '
            'real-time object detection with TensorRT optimized '
            'YOLO model on Jetson')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument(
        '-c', '--category_num', type=int, default=80,
        help='number of object categories [80]')
    parser.add_argument(
        '-m', '--model', type=str, required=True,
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    args = parser.parse_args()
    return args



is_tuning = False
time_begin_tuning = -1

time_tuning =1
mode = 'T'
status = None
Velocity = "0020"
V_cl = "0020"
#i=1199
cache = {}

def loop_and_detect(cam, trt_yolo, conf_th, vis):
    """Continuously capture images from camera and do object detection.
    
    # Arguments
      cam: the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """
    global is_tuning
    global time_begin_tuning
    global time_tuning
    global mode
    global status
    global V_cl
    global cache
    global i
    
    predict = "150"
    full_scrn = False
    fps = 0.0
    tic = time.time()
    global Velocity
    while True:
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break

        img = cam.read()
        image = img.copy()
        if img is None:
            break
        
        #predict angle
        #cv2.imwrite('/home/quanghuy/Desktop/data28_5/'+str(i)+'.png',img)
        #i+=1      
        
        boxes, confs, clss = trt_yolo.detect(img, conf_th)
        img,mid_point_x,Shcn,cl = vis.draw_bboxes(img, boxes, confs, clss)  
        
        #print(mid_point_x,Shcn)
        if is_tuning :
            if time_begin_tuning + time_tuning < time.time():
                is_tuning = False
                if status != "Go ahead":
                    predict  = cache['predict']
                    Velocity  = cache['Velocity']
                    serial_port.write(('A'+Velocity+"0"+predict+mode+'E').encode())
                start_process = time.time()
                while start_process+2.5>time.time():
                   if status == "Go ahead":
                     predict = "150"
                     serial_port.write(('A'+Velocity+"0"+predict+mode+'E').encode())
                   print("##Turning + {}.....................###".format(status))
                   print(Velocity,predict)

            else : 
                predict = CNN.Main(image,model_cnn)
                if predict == "100" or predict == "190":
                    Velocity = "0030"
                elif predict == "135" or  predict == "170":
                    Velocity = "0020"
                else:
                    Velocity = V_cl
                serial_port.write(('A'+Velocity+"0"+predict+mode+'E').encode())
                print("#Delay.....................##############3")
               
        else:
        #sign dectect
        
            if len(clss) !=0:
                
                if clss[-1] == 4. and Shcn >= 13000:
                    time_begin_tuning = time.time() 
                    is_tuning = True
                    mode = 'S'
                    cache['predict'] = "150"
                    status = 'Stop'
                else:
                    mode = 'T'
                    #Velocity = "0030"
                if clss[-1] == 3 and Shcn >= 13000 :
                    Vcl = "0020"
                    print('Speed 22')
                if clss[-1] == 2.and  Shcn >= 13000:
                    Vcl = "0060"
                    print('Speed 50')

                if clss[-1] == 5. and mid_point_x >= 550:
                    time_begin_tuning = time.time() 
                    is_tuning = True
                    status = 'Go ahead'
                    time_tuning = 1.5
                if clss[-1] == 0 or clss[-1]==1:
                 if cl == 1 and mid_point_x >= 450 and Shcn >= 13000:
                    time_begin_tuning = time.time() 
                    is_tuning = True
                    cache['predict'] = "185"
                    cache['Velocity'] ="0030"
                    status = 'Turn right'
                    time_tuning = 1.5
                 if cl == 0 and mid_point_x >= 520 and Shcn >= 15000 :
                    time_begin_tuning = time.time()
                    is_tuning = True
                    cache['predict'] = "100"
                    cache['Velocity'] ="0030"
                    status = 'Turn left'
                    time_tuning = 2
               
                predict = CNN.Main(image,model_cnn) 
                if predict == "100" or predict == "185":
                    Velocity = "0030"
                elif predict == "130" or  predict == "170":
                    Velocity = "0020"
                else:
                    Velocity = V_cl       
                serial_port.write(('A'+Velocity+"0"+predict+mode+'E').encode())
                
            else: 
                mode = 'T'
                predict = CNN.Main(image,model_cnn)
                if predict == "100" or predict == "190":
                    Velocity = "0030"
                elif predict == "135" or  predict == "170":
                    Velocity = "0020"
                else:
                    Velocity = V_cl
                print("##Predicting Angle.............##")
                
                serial_port.write(('A'+Velocity+"0"+predict+mode+'E').encode())
        #print(Velocity,predict)
        img = show_fps(img, fps,predict)
        cv2.imshow(WINDOW_NAME, img)
        toc = time.time()
        curr_fps = 1.0 / (toc - tic)
        # calculate an exponentially decaying average of fps number
        fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
        tic = toc
        key = cv2.waitKey(1)
        set_display(WINDOW_NAME, full_scrn)
        if key == 27:  # ESC key: quit program
            break
        # elif key == ord('F') or key == ord('f'):  # Toggle fullscreen
        #     full_scrn = not full_scrn
        #     set_display(WINDOW_NAME, full_scrn)


def main():
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)
   
    
    cam = Camera(args)
   
    if not cam.isOpened():
         raise SystemExit('ERROR: failed to open camera!')
    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        cam.img_width, cam.img_height)
    loop_and_detect(cam, trt_yolo, conf_th=0.6, vis=vis)

    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
