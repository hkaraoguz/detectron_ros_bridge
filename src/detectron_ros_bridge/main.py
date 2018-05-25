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

class DetectronClient():

    def __init__(self,host='192.168.1.118',port=5000):
        self.socket= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(10.0)
        self.host = host #"192.168.1.118" # needs to be in quote
        self.port = port
        self.s_client = rospy.Service('detectron', Detectron, self.handle_detectron_request)

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
        buf = []
        try:
            buf = self.socket.recv(4096)
            #buf += s.recv(1)
            if len(buf) > 0:
                rospy.loginfo("Received : %s",str(buf))
            return buf
        except:
            return buf

    def close(self):
        self.send_data("close me".encode("utf-8"))
        time.sleep(1)
        self.socket.close()

    def send_data(self, data):
        try:
            self.socket.send(data)
            time.sleep(1)
            return True
        except Exception as ex:
            print str(ex)
            return False
            #rospy.signal_shutdown("Socket connection failure")


    def handle_detectron_request(self,req):

        data = dict()

        img = base64.b64encode(req.image.data)

        data["img"] = img
        data['bb_threshold'] = req.conf_threshold
        data['det_threshold'] = req.conf_threshold
        data["name"] = "rocco"

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




if __name__ == '__main__':


    rospy.init_node('detectron_ros_bridge')

    argparse = argparse.ArgumentParser(prog='main.py');
    argparse.add_argument("--host", type=str, help='Host address')
    argparse.add_argument("--port", type=int, help='Host port', default=5000)


    args = argparse.parse_args(rospy.myargv(argv=sys.argv)[1:])


    detectronclient = DetectronClient(host=args.host, port=args.port)

    if(not detectronclient.connect()):
        rospy.signal_shutdown("Communication Error with the Server")

    rospy.loginfo("Detectron ROS bridge started...");

    rospy.spin()

    #while not rospy.is_shutdown():
        #data = detectronclient.listen()
        #if len(data) > 0 :
        #    enerothclient.parsedata(data)



    detectronclient.close()
