from html import unescape
from multiprocessing.sharedctypes import Value
import time
from turtle import forward
import rclpy
from rclpy.node import Node
from pipecrawler.action import Crawlaction
from fm_interfaces.srv import Checkimu
from fm_interfaces.srv import Checklidar
from reel.action import Reelaction
from rclpy.action import ActionClient
from std_msgs.msg import Int16
from transitions import Machine
from transitions.extensions import GraphMachine

states = ['initializing','top_elbow','vertical_descent','bottom_elbow',
          'lateral_crawl','reached_end','retrieval','finalizing','failiure']

state_mach_params = dict(initial='initializing', title='WRPS Lateral Gamma Scanner State Machine',
                  show_conditions=True, show_state_attributes=False)

crawler_rate= 0.6 #Feet per extension 
rc_synch = 6 # THIS SLEEP TIME MAINLY SYNCHRONIZES REEL AND CRAWLER

crawler_commands = {
    "forward": [1,3,5,2,4,6],
    "backward": [5,3,1,6,4,2],
    "loosen": [4,2,6,0,0,0],
    "fix": [4,1,3,0,0,0],
    "none": [0,0,0,0,0,0],
    "engage_back": [1,0,0,0,0,0],
    "engage_front": [3,0,0,0,0,0],
}

reel_commands = {
    # "name":[velocity(int),interval(float), continuous(bool) ]
    "forward":[1, 2.15, True], # THESE VALUES FINELY SYNCHRONIZE REEL AND CRAWLER
    "backward":[-1, 2.15, True], 
    "stop": [0, 0.0, False],
    "unwind": [1, 2.15, False],
    "retrieve": [-1,2.15,True],
}

class StateMachineNode(Node):
    def __init__(self):
        super().__init__("state_machine")
        self.declare_parameter('pipe_length', 160)
        self.get_logger().info("State machine node is up")
        self._crawler_client = ActionClient(self, Crawlaction, 'crawl_through_pipe')
        self._reel_client = ActionClient(self, Reelaction, 'activate_reel' )
        self._pitch_client = self.create_client(Checkimu, 'pitch_service')
        self._distance_client = self.create_client(Checklidar, 'distance_service')
        # while not self._pitch_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('Front Module IMU not available, Trying again...')
        # while not self._distance_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('Front Module LiDAR not available, Trying again...')
        self.imu_req = Checkimu.Request()
        self.imu_req.request = 1
        self.lidar_req = Checklidar.Request()
        self.lidar_req.requestlidar = 1
        time.sleep(2)

    def is_vertical(self):
        # self.get_logger().info("Checking if crawler has cleared top elbow")
        # self.pitch_is_vertical = False
        # while self.pitch_is_vertical == False:   
            # self.future = self._pitch_client.call_async(self.imu_req)
            # rclpy.spin_until_future_complete(self, self.future)
            # self.pitch = abs(self.future.result().pitch)
            # self.get_logger().info("Current inclination angle: {0}".format(self.pitch))
            # if self.pitch > 75 and self.pitch < 100:
                # self.pitch_is_vertical = True
                # self.get_logger().info("Top elbow cleared")
                # self.get_logger().info("Transitioning to state: {0}".format(self.state))
                # break
            # else:
                # self.get_logger().info("Not cleared top elbow yet, crawling forward")
                # self.crawl('forward')
                
        self.crawl('forward')
        return True        
        
    def is_horizntl(self):
        self.get_logger().info("Checking if crawler has cleared bottom elbow")
        # self.pitch_is_horizntl = False
        # while self.pitch_is_horizntl == False:   
            # self.future = self._pitch_client.call_async(self.imu_req)
            # rclpy.spin_until_future_complete(self, self.future)
            # self.pitch = abs(self.future.result().pitch)
            # self.get_logger().info("Current inclination angle: {0}".format(self.pitch))
            # if self.pitch < 15 and self.pitch < 100:
                # self.pitch_is_horizntl = True
                # self.get_logger().info("Bottom elbow cleared")
                # self.get_logger().info("Transitioning to state: {0}".format(self.state))
                # time.sleep(2)
                # break
            # else:
                # self.get_logger().info("Not cleared bottom elbow yet, crawling forward")
                # self.crawl('forward')
        return True        
        
    def end_reached(self):
        self.get_logger().info("Checking if crawler has reached end")
        # self.has_reached_end = False
        # while self.has_reached_end == False:   
            # self.future = self._distance_client.call_async(self.lidar_req)
            # rclpy.spin_until_future_complete(self, self.future)
            # self.future = self._distance_client.call_async(self.lidar_req)
            # rclpy.spin_until_future_complete(self, self.future)
            # self.d_to_end = abs(self.future.result().distance)
            # self.get_logger().info("Distance until end: {0}".format(self.d_to_end))
            # if self.d_to_end < 1.0:
                # self.has_reached_end = True
                # break
            # else:
                # self.get_logger().info("Not reached end yet, crawling forward")
                # time.sleep(3)
        return True     


    def reel_activate(self, cmd="forward"):
        reel_goal = Reelaction.Goal()
        reel_goal.reelcommand.reel_vel = reel_commands[cmd][0]
        reel_goal.reelcommand.interval = reel_commands[cmd][1]
        reel_goal.reelcommand.continuous = reel_commands[cmd][2]
        self._reel_client.send_goal_async(reel_goal)

    def crawl(self, dir='forward'):
        self._crawler_client.wait_for_server(timeout_sec=2)
        self._reel_client.wait_for_server(timeout_sec=2)
        crawl_goal = Crawlaction.Goal()
        crawl_goal.crawlercommand.crawlpattern = crawler_commands[dir]
        crawl_goal.crawlercommand.continuous = False
        self.get_logger().info("Sending {0} crawl request ".format(dir))
        self._crawler_client.send_goal_async(crawl_goal)
        self.reel_activate(dir)
        time.sleep(rc_synch) 
        self.reel_activate("stop")

def initialize_machine():
    rclpy.init()
    _sm = StateMachineNode()
    machine = GraphMachine(model=_sm, states=states ,**state_mach_params)
    machine.add_transition('vert_transition', source='initializing', dest='top_elbow')
    machine.add_transition('vert_pitch', ['top_elbow'], dest= '=', after= 'crawl', unless='is_vertical')
    machine.add_transition('descend', source='top_elbow', dest='vertical_descent', conditions='is_vertical')
    machine.add_transition('lat_transition', source='vertical_descent', dest='bottom_elbow')
    machine.add_transition('hori_pitch', ['bottom_elbow'],  dest= '=', unless='is_horizntl')
    machine.add_transition('begin_crawl', source='bottom_elbow', dest='lateral_crawl', conditions='is_horizntl')
    machine.add_transition('continue', ['lateral_crawl'],  dest= '=', unless='end_reached')
    machine.add_transition('confirm_end', source='lateral_crawl', dest='reached_end', conditions='end_reached')
    machine.add_transition('retrieve', source=['reached_end', 'failiure'], dest='retrieval')
    machine.add_transition('final_position', source='retrieval', dest='finalizing')
    machine.add_transition('fail', source=['top_elbow','vertical_descent','bottom_elbow','lateral_crawl' ], dest='failiure')
    machine.machine_attributes['ratio'] = '0.5'
    _sm.get_graph().draw('lgs_state_diagram.png', prog='dot')
    
    ## Add  transition callbacks as parameters like: prepare/before/after='<callback_name>'
    ## Or add state callbacks like 'on_enter/exit_<callback_name>'

    machine.on_enter_top_elbow('crawl')
    return _sm

def main(args=None):
    sm = initialize_machine()
    sm.get_logger().info("Current State: {0}".format(sm.state))
    sm.vert_transition("forward")
    time.sleep(1)
    sm.get_logger().info("Current State: {0}".format(sm.state))
    sm.descend()
    time.sleep(1) 
    sm.get_logger().info("Current State: {0}".format(sm.state))
    sm.lat_transition()
    time.sleep(1)
    sm.get_logger().info("Current State: {0}".format(sm.state))
    sm.begin_crawl()
    time.sleep(1)
    sm.get_logger().info("Current State: {0}".format(sm.state))
    sm.confirm_end()
    time.sleep(1)
    sm.get_logger().info("Current State: {0}".format(sm.state))
    sm.retrieve()
    time.sleep(1)
    sm.get_logger().info("Current State: {0}".format(sm.state))

if __name__ == "__main__":
    main()
