#! /usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import roslib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import tf
import math
from threading import Timer,Thread
import time
import json
from socketIO_client import SocketIO, LoggingNamespace

goal_pose = {'px':0,'py':0,'head':0}
map_info = {}
path_info = {}
GLOBAL_VAR = {}

class IOHelper():

    def __init__(self):
        self.__io = SocketIO("localhost", 80, wait_for_connection=True)
        self.__connected = False

        self.__threads = []
        t1 = Thread(target=self.thread_spin, args='')
        self.__threads.append(t1)
        self.thread_start();

    def event(self, event, func):
        self.__io.on(event,func)

    def thread_spin(self):
        while not rospy.is_shutdown():
            self.__io.wait();

    def thread_start(self):
        for t in self.__threads:
            t.setDaemon(True)
            t.start()

    def upload_pose(self,x,y,head):
        self.__io.emit('pose_upload', {'type': 'robot1', 'id': '1', 'pose':
            {'px': x, 'py': y, 'head': head}})

    def upload_info(self,info):
        if len(info) :
            self.__io.emit('notification', {'type': 'robot_map_info', 'id': '1', 'info':
            {'resolution': info['resolution'],
             'width': info['width'],
             'height': info['height'],
             'origin_x': info['origin_x'],
             'origin_y': info['origin_y'],
             'goal_x':goal_pose['px'],
             'goal_y':goal_pose['py']
             }})

    def notify_heartbeat(self):
        self.__io.emit('heart_beat')

    def upload_path(self,path,size):
        self.__io.emit('path',{'type': 'robot_path_info',
                               'id': '1',
                               'size':size,
                               'path': path})

def on_notification_respose(*args):
    if args[0]["type"] == "goal_set":
        if not map_info.has_key('resolution'):
            map_info['resolution']=0.05
            print('[ERROR]resolution not get');

        goal_pose['px'] = float(args[0]['pose']['px']* map_info['resolution'])
        goal_pose['py'] = float(args[0]['pose']['py']* map_info['resolution'])
        goal_pose['head'] = float(args[0]['pose']['head'])

        data = MoveBaseActionGoal()
        data.goal.target_pose.pose.position.x = goal_pose['px']
        data.goal.target_pose.pose.position.y = goal_pose['py']
        data.goal.target_pose.pose.orientation.w = 0.1
        data.goal.target_pose.header.frame_id = "map"
        GLOBAL_VAR['goal_publisher'].publish(data)

    if args[0]["type"] == "robot_map_info_req":
        GLOBAL_VAR['io'].upload_info(map_info)

def map_callback(data):
    map_info['resolution'] = float(data.info.resolution)
    map_info['width'] =  data.info.width
    map_info['height'] = data.info.height
    map_info['origin_x'] = data.info.origin.position.x
    map_info['origin_y'] = data.info.origin.position.y
    GLOBAL_VAR['io'].upload_info(map_info)

def global_path_callback(data):
    len_path = len(data.poses)
    gap = 1
    if len_path >= 10:
        gap = int(len_path / 10.0 + 0.5)
    path = data.poses[::gap]
    #construct path json data
    json_data = []
    for i in range(len(path)):
        json_data.append({'x':path[i].pose.position.x/map_info['resolution'],
                   'y':path[i].pose.position.y/map_info['resolution']})
    GLOBAL_VAR['io'].upload_path(json_data,len(path))

if __name__ == "__main__":
    #socketio instance register
    io = IOHelper()
    GLOBAL_VAR['io']=io
    io.event('notification', on_notification_respose)
    rospy.init_node('xmap_robot_sender',anonymous=True)
    #register publisher to GLOBAL,for reason need time for first publish
    publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
    GLOBAL_VAR['goal_publisher'] = publisher
    rospy.Subscriber("map", OccupancyGrid, map_callback)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, global_path_callback)
    #tf transform
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    rospy.logout("Xmap Server Started")
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        io.upload_pose(trans[0]/map_info['resolution'],trans[1]/map_info['resolution'],rot[2])
        io.notify_heartbeat();
        rate.sleep()
