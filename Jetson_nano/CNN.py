import time 
import numpy as np
import pickle
import time
import serial
import cv2
import tensorflow as tf
from tensorflow import keras
from jetcam.usb_camera import USBCamera                                                                             




#Set_up serial
def conf_serial(port= "/dev/ttyUSB0",baudrate = 115200,timeout=0.5 ) : 
    serial_port = serial.Serial(port = port, baudrate= baudrate , timeout= timeout )
    serial_port.close()
    serial_port.open() 
    return serial_port


#  Angle
def predict(resize_image=None,classes=['100','130','150','170','185'],model=None):
    image_expand = np.expand_dims(resize_image,0)
    input = tf.constant(image_expand,dtype=tf.float32)
    infer = model.signatures['serving_default']
    predict_angel = infer(input)
    id = np.argmax(predict_angel['predict'].numpy())
    predict_angle = classes[id]
    return predict_angle

def resize_image(image):
    #image = cv2.resize(image,size)
    image = np.array(image)
    
    image = image[250:,:]
    
    resize_image = cv2.resize(image,(270,80))

    return resize_image


#Load model 
# model = tf.saved_model.load('CNN_lane_test')

def Main(image= None,model=None):
    image = resize_image(image)
    predict_angle = predict(resize_image= image, model= model)
    return predict_angle





