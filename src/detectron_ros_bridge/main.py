#!/usr/bin/env python

import socket
import rospy
import sys
import time
import argparse
import json
import base64
from detectron_ros_bridge.srv import *
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image as Im
import io

class DetectronClient():

    def __init__(self,host='192.168.1.118',port=5000):
        self.socket= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(30.0)
        self.host = host #"192.168.1.118" # needs to be in quote
        self.port = port
        self.s_client = rospy.Service('detectron', Detectron, self.handle_detectron_request)
        self.bridge = CvBridge()
        self.ji = 0

    def connect(self):
        try:
           self.socket.connect((self.host, self.port))
        except Exception as ex:
           print str(ex)
           return False

        time.sleep(1)
        #self.socket.send("client_type;{}".format(self.client_type).encode("utf-8"))

        return True

    def listen(self):
        buf = b''
        try:
            buf = self.socket.recv(4096)
            #buf += s.recv(1)
            if len(buf) > 0:
                rospy.loginfo("Received : %s",str(buf))
            return buf
        except:
            return buf

    def close(self):
        #self.send_data("close me".encode("utf-8"))
        #time.sleep(1)
        self.socket.close()

    def send_data(self, data):
        try:
            self.socket.sendall(data)
            time.sleep(1)
            return True
        except Exception as ex:
            print str(ex)
            return False
            #rospy.signal_shutdown("Socket connection failure")


    def handle_detectron_request(self,req):

        data = dict()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        ret, buffer_img = cv2.imencode('.jpg', cv_image)


        #img = base64.b64encode(req.image.data)

        data['img'] = base64.b64encode(buffer_img)
        data['bb_threshold'] = req.conf_threshold
        data['novelty_threshold'] = 0.3#req.conf_threshold
        data['name'] = "rocco"

        if ji == 0:
            if not self.send_data(json.dumps(data)):
                return DetectronResponse("{}")


    def parsedata(self,data):

        data_arr = data[:-1].split(";")

        if data_arr[0] == 'home':
            command = Command()
            command.type = data_arr[0];
            self.pub.publish(command)
            return

        if len(data_arr) < 3:
            rospy.logwarn("Received data %s seems corrupt. Skipping...", data_arr);
            return
        ''''
        pos_arr = data_arr[1].split(";")

        if len(pos_arr) < 2:
            rospy.logwarn("Received pose data %s seems corrupt. Skipping...", pos_arr);
            return
        '''
        command = Command()
        command.type = data_arr[0];
        command.posx = round(float(data_arr[1]),3)
        command.posy = round(float(data_arr[2]),3)

        self.pub.publish(command)

    def image_callback(self,msg):

        data = dict()


        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        ret, buffer_img = cv2.imencode('.jpg', cv_image)

        #encoded = #base64.encodestring(b'data to be encoded')
        #data['bytes'] = encoded.decode('ascii')
        #img =  str(msg.data).encode("base64") #base64.b64encode(data)

        data["img"] = base64.b64encode(buffer_img)#img.encode('ascii')
        data['bb_threshold'] = 0.5
        data['novelty_threshold'] = 0.3
        data["name"] = "rocco"

        jsondata = json.dumps(data)

        #enc = jsondata.encode()  # utf-8 by default
        #print base64.encodestring(enc)
        if self.ji == 0:
            if not self.send_data(jsondata):
                print "Socket error while sending data"
                rospy.signal_shutdown("Socket Error")
            self.ji += 1

        #time.sleep(2)

        #self.rec


# Take in base64 string and return cv image
def stringToRGB(base64_string):
    dataArray = np.frombuffer(base64.decodestring(base64_string), np.float32)
    print dataArray.shape
    #imgdata = base64.b64decode(str(base64_string))
    #im = np.fromstring(base64_string)#.astype(np.float16)
    im = dataArray.reshape(544,960)
    print im.shape
    print np.where(im > 0.5)
    image = Im.fromarray(im)
    filename = "deneme"
    filename +=".tiff"
    print(filename)
        
    image.save(filename)
    #image =Im.open(io.BytesIO(imgdata))
    return image#cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB)



if __name__ == '__main__':


    rospy.init_node('detectron_ros_bridge')

    argparse = argparse.ArgumentParser(prog='main.py');
    argparse.add_argument("--host", type=str, help='Host address',default="localhost")
    argparse.add_argument("--port", type=int, help='Host port', default=5000)
    argparse.add_argument("--camera_topic", type=str, help='camera topic to listen', default="kinect2/qhd/image_color")


    args = argparse.parse_args(rospy.myargv(argv=sys.argv)[1:])


    print args.host
    print args.port
    detectronclient = DetectronClient(host=args.host, port=args.port)

    rospy.Subscriber(args.camera_topic, Image, detectronclient.image_callback)


    if(not detectronclient.connect()):
        rospy.signal_shutdown("Communication Error with the Server")

    rospy.loginfo("Detectron ROS bridge started...");
    data = b''
    while not rospy.is_shutdown():
        
        data += detectronclient.listen()
        
        if len(data) > 0 :
            #print type(data)
            #print data.split()
            try:
                jsonresp = json.loads(data)
                data = b''
                print jsonresp[0]
                img = stringToRGB(jsonresp[1])
                print img
            except Exception as ex:
                print ex
                print "keep trying"
                
            #print len(data.split(","))
            
            #enerothclient.parsedata(data)

    #while not rospy.is_shutdown():
        #data = detectronclient.listen()
        #if len(data) > 0 :
        #    enerothclient.parsedata(data)



    detectronclient.close()
