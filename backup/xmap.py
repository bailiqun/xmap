#! /usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import tf
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from socketIO_client import SocketIO
from threading import Thread

Global_Variable={
    "goal":{'px':0,'py':0,'head':0},
    "map_info":{'resolution': 0.05, 'width': 0,'height': 0,'origin_x': 0,'origin_y':0,'goal_x':0,'goal_y':0,'goal_head':0},
    "vel":{'vx':0,'vy':0,'vth':0},
    "ros":{'goal_publisher':0},
    "move_base":{'status':0}
}

class IOHelper():

    def __init__(self,ip,port):
        self.__io = SocketIO(ip, port, wait_for_connection=True)
        self.__connected = False
        self.__threads = []
        t1 = Thread(target=self.thread_spin, args='')
        self.__threads.append(t1)
        self.thread_start();
    def on(self, event, func):
        self.__io.on(event,func)
    def thread_spin(self):
        while not rospy.is_shutdown():
            self.__io.wait();
    def thread_start(self):
        for t in self.__threads:
            t.setDaemon(True)
            t.start()
    def pose_vel_info_send(self, x, y, yaw, vx, vy, vth, status):
        self.__io.emit('/notification/robot',{
            'topic': 'pose_vel_info',
            'from' : 'robot1',
            'id': 1,
            'status': status,
            'pose': {'px': x, 'py': y, 'head': yaw},
            'vel' : {'vx':vx, 'vy':vy, 'vth' :vth}})
    def map_info_send(self,info):
        if len(info) :
            self.__io.emit('/notification/robot', {
                'topic': 'map_info',
                'from': 'robot1',
                'id': '1',
                'info':{
                    'resolution': info['resolution'],
                    'width': info['width'],
                    'height': info['height'],
                    'origin_x': info['origin_x'],
                    'origin_y': info['origin_y'],
                    'goal_x': Global_Variable['goal']['px'],
                    'goal_y': Global_Variable['goal']['py']
             }})
    def path_send(self, path, size):
        self.__io.emit('/notification/robot', {
            'topic': 'path_info',
            'from': 'robot1',
            'id': 1,
            'size': size,
            'path': path})
    def heartbeat_ask(self):
        self.__io.emit('heart_beat')

def on_socketio(*args):
    if args[0]["topic"] == "goal_info":
        Global_Variable['goal']['px'] = float(args[0]['pose']['px'] * Global_Variable['map_info']['resolution'])
        Global_Variable['goal']['py'] = float(args[0]['pose']['py'] * Global_Variable['map_info']['resolution'])
        Global_Variable['goal']['head'] = float(args[0]['pose']['head'])
        data = MoveBaseActionGoal()
        data.goal.target_pose.pose.position.x = Global_Variable['goal']['px']
        data.goal.target_pose.pose.position.y = Global_Variable['goal']['py']
        data.goal.target_pose.pose.orientation.w =0.1
        data.goal.target_pose.header.frame_id = "map"
        Global_Variable['ros']['goal_publisher'].publish(data)
    if args[0]["topic"] == "mapinfo_request":
        Global_Variable['io_helper'].map_info_send(Global_Variable['map_info'])

def ros_map_callback(data):
    Global_Variable['map_info']['resolution'] = float(data.info.resolution)
    Global_Variable['map_info']['width']  =  data.info.width
    Global_Variable['map_info']['height'] =  data.info.height
    Global_Variable['map_info']['origin_x'] = data.info.origin.position.x
    Global_Variable['map_info']['origin_y'] = data.info.origin.position.y
    Global_Variable['map_info']['goal_x'] = Global_Variable['goal']['px']
    Global_Variable['map_info']['goal_y'] = Global_Variable['goal']['py']
    Global_Variable['map_info']['goal_head'] = Global_Variable['goal']['head']
    Global_Variable['io_helper'].map_info_send(Global_Variable['map_info'])

def ros_global_path_callback(data):
    len_path = len(data.poses)
    gap = 1
    if len_path >= 10:
        gap = int(len_path / 10.0 + 0.5)
    path = data.poses[::gap]
    json_data = []
    for i in range(len(path)):
        json_data.append(
            {'x':path[i].pose.position.x / Global_Variable['map_info']['resolution'],
             'y':path[i].pose.position.y / Global_Variable['map_info']['resolution']})
    json_data.append({'x': Global_Variable['goal']['px'] / Global_Variable['map_info']['resolution'],
                      'y': Global_Variable['goal']['py'] / Global_Variable['map_info']['resolution']})
    Global_Variable['io_helper'].path_send(json_data,len(path))

def ros_vel_callback(data):
    Global_Variable['vel']['vx']= data.linear.x
    Global_Variable['vel']['vy'] = data.linear.y
    Global_Variable['vel']['vth'] = data.angular.z

def ros_move_base_status_callback(data):
    state_num = len(data.status_list) - 1
    if state_num < 0 :
        state_num = 0;
    status_id = data.status_list[state_num].status
    state_tabl = ['PENDING','ACTIVE','PREEMPTED','SUCCEEDED','ABORTED','REJECTED','PREEMPTING','RECALLING','RECALLED','LOST']
    Global_Variable['move_base']['status'] = state_tabl[status_id]

def ros_current_goal_callback(data):
    Global_Variable['goal']['px'] = data.goal.target_pose.pose.position.x
    Global_Variable['goal']['py'] = data.goal.target_pose.pose.position.y

if __name__ == "__main__":
    rospy.init_node('xmap_robot',anonymous=True)
    io_helper = IOHelper('192.168.3.4',80)
    Global_Variable['io_helper']= io_helper
    io_helper.on('/notification/robot', on_socketio)

    ros_goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
    Global_Variable['ros']['goal_publisher'] = ros_goal_publisher

    rospy.Subscriber("/map", OccupancyGrid, ros_map_callback)
    rospy.Subscriber("/mobile_base/commands/velocity", Twist, ros_vel_callback)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, ros_global_path_callback)
    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, ros_current_goal_callback)
    rospy.Subscriber("/move_base/status", GoalStatusArray, ros_move_base_status_callback)

    #tf transform
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    rospy.logout("Xmap Server Started")
    while not rospy.is_shutdown():
        try:
            (trans, quaternion) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            rpy = tf.transformations.euler_from_quaternion(quaternion)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        io_helper.pose_vel_info_send(trans[0] / Global_Variable['map_info']['resolution'],# position x
                                     trans[1] / Global_Variable['map_info']['resolution'],# position y
                                     rpy[2],# Yaw
                                     Global_Variable['vel']['vx'],
                                     Global_Variable['vel']['vy'],
                                     Global_Variable['vel']['vth'],
                                     Global_Variable['move_base']['status'])
        io_helper.heartbeat_ask()
        rate.sleep()
