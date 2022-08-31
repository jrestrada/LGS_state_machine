from multiprocessing.connection import wait
from multiprocessing.sharedctypes import Value
import time
import rclpy
from rclpy.node import Node
from pipecrawler.action import Crawlaction
from fm_interfaces.srv import Checkimu
from fm_interfaces.srv import Checklidar
from reel.action import Reelaction
from rclpy.action import ActionClient
from std_msgs.msg import Int16
from transitions.extensions import GraphMachine

states = ['initializing',{'name':'top_elbow', 'on_enter':'engage_safety','on_exit':'check_vertical'}, 
         {'name':'vertical_pipe', 'on_enter':'advance'},
         {'name':'bottom_elbow', 'on_enter':'engage_safety', 'on_exit':'check_horizontal'},
         {'name':'lateral_crawl', 'on_enter':'advance'},'reached_end',
         {'name':'retrieval', 'on_enter':'wind_up'},
         {'name':'vertical_retrieval', 'on_enter':'wind_up'},
          'finalized','horizontal_failure','horizontal_failure',]

state_mach_params = dict(initial='vertical_pipe', title='WRPS Lateral Gamma Scanner State Machine', send_event= True,
                  prepare_event = 'log_state' , show_conditions=True, show_state_attributes=True)

vertical_height = 12  # Feet
lateral_length = 10   # Feet
auto_retrieve = 20
end_margin = 2        # Feet
crawler_rate= 8       # Inches per extension 
rc_synch = 17       # THIS SLEEP TIME MAINLY SYNCHRONIZES REEL AND CRAWLER
# rc_synch = 6       # THIS SLEEP TIME MAINLY SYNCHRONIZES REEL AND CRAWLER
wind_rate = 1.3       # reel vel to inches 
unwind_rate = 1.2     # reel vel to inches 
top_elbow_buffer = 2  # Extensions to clear top elbow after IMU vertical pitch 
horizontal_angle = 15 # degrees
vertical_angle = 80   # degrees

time_from_white = 42 # seconds

crawler_commands = {
    "forward":       [1,3,5,2,4,6],
    "backward":      [5,3,1,6,4,2],
    "release":        [4,2,6,0,0,0],
    "fix":           [4,1,3,0,0,0],
    "none":          [0,0,0,0,0,0],
    "engage_back":   [1,0,0,0,0,0],
    "release_back":   [2,0,0,0,0,0],
    "forward_safe":  [3,5,2,4,1,6],
}

reel_commands = {
    "forward":      [1,   2.2, True], 
    "backward":     [-1,  2.1,  True], 
    "release":       [0,  0.0,  False],
    "engage_back":  [0,  0.0,  False],
    "release_back":  [0,  0.0,  False],
    "forward_safe": [1,   2.12, True], 
    "stop":         [0,   0.0, False],
    "unwind":       [2,   0.8, False],
    "retrieve":     [-2,  0.8,  True],
}

def feet_to_in(feet):
    return 12*feet

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
   
    def full_unwind(self, event):
        self.unwind_complete = False
        self.unwound_length = 0.0
        while self.unwind_complete == False:
            if self.unwound_length >= feet_to_in(vertical_height):
                self.unwind_complete = True
                self.get_logger().info("Bottom elbow reached")
                break
            else:
                self.get_logger().info("Bottom elbow not reached")
                self.activate_reel('unwind')
                self.get_logger().info("Unwound : {0} feet".format(self.unwound_length/12))
                self.unwound_length += reel_commands['unwind'][0]*wind_rate
                time.sleep(reel_commands['unwind'][1]+0.1)
        return True  

    def wind_up(self, event):
        self.unwind_complete = False
        self.retrieved_length = 0.0
        self.target_length = event.kwargs.get('len')
        while self.unwind_complete == False:
            if self.retrieved_length >= self.target_length:
                self.unwind_complete = True
                self.get_logger().info("retrieval section complete")
                break
            else:
                self.activate_reel('retrieve')
                self.retrieved_length +=  abs(reel_commands['retrieve'][0])/wind_rate
                self.get_logger().info("Retrieved : {0}".format(self.retrieved_length/12))
                time.sleep(reel_commands['retrieve'][1]+0.1)
        self.activate_reel("stop")
    
    def advance(self, event):
        for i in range(event.kwargs.get('times')):
            self.activate_crawler(event.kwargs.get('cmd'), 1 )

    def get_current_pitch(self):
        # self.future = self._pitch_client.call_async(self.imu_req)
        # rclpy.spin_until_future_complete(self, self.future)
        # return abs(self.future.result().pitch)    
        print('type angle')
        pitch_fake = input()
        return int(pitch_fake)
    
    def get_current_distance(self):
        # self.future = self._distance_client.call_async(self.lidar_req)
        # rclpy.spin_until_future_complete(self, self.future)
        # self.future = self._distance_client.call_async(self.lidar_req)
        # rclpy.spin_until_future_complete(self, self.future)
        # distance_fake = abs(self.future.result().distance) 
        print('type distance')
        distance_fake = input() 
        return distance_fake

    def is_hori(self):
        self.pitch = self.get_current_pitch()
        self.get_logger().info("Angle : {0}".format(self.pitch))
        return (self.pitch < horizontal_angle and self.pitch < 100)

    def is_vert(self):
        self.pitch = self.get_current_pitch()
        self.get_logger().info("Angle : {0}".format(self.pitch))
        return (self.pitch > vertical_angle and self.pitch < 100)

    def reached(self):
        self.pitch = self.get_current_distance()
        return (self.pitch < 2)

    def check_vertical(self, event):
        self.get_logger().info("Checking Verticality")
        while not self.is_vert():
            self.get_logger().info("Not vertical yet, crawling forward")
            self.activate_crawler(event.kwargs.get('cmd'), event.kwargs.get('times'))
        self.get_logger().info("Top elbow cleared")

    def check_horizontal(self, event):
        self.get_logger().info("Checking Horizontality")
        while not self.is_hori():
            self.get_logger().info("Not horizontal yet, crawling forward")
            self.activate_crawler(cmd='forward_safe', times=1)
        self.get_logger().info("Bottom elbow cleared")
 
    def end_reached(self, event):
        self.get_logger().info("Checking distance to end")
        while not self.reached():
            self.get_logger().info("Not reached end yet, crawling forward")
            self.activate_crawler(cmd='forward', times=1)
        self.get_logger().info("End reached")
        return True

    def activate_reel(self, cmd="forward"):
        self.get_logger().info("Sending {0} reel request ".format(cmd))
        reel_goal = Reelaction.Goal()
        reel_goal.reelcommand.reel_vel = reel_commands[cmd][0]
        reel_goal.reelcommand.interval = reel_commands[cmd][1]
        reel_goal.reelcommand.continuous = reel_commands[cmd][2]
        self._reel_client.send_goal_async(reel_goal)

    def activate_crawler(self, cmd = 'forward', times =1):
        self._crawler_client.wait_for_server(timeout_sec=2)
        self._reel_client.wait_for_server(timeout_sec=2)
        crawl_goal = Crawlaction.Goal()
        crawl_goal.crawlercommand.crawlpattern = crawler_commands[cmd]
        crawl_goal.crawlercommand.continuous = False
        for i in range(times):
            self.get_logger().info("Sending {0} crawl request ".format(cmd))
            self.activate_reel(cmd)
            self._crawler_client.send_goal_async(crawl_goal)
            time.sleep(rc_synch) 
            self.activate_reel("stop")
            self.hold()
    
    def engage_safety(self, event):
        self.activate_crawler("engage_back", 1)
    
    def hold(self, cmd = "stop", times = 1):
        time.sleep(2)

    def log_state(self, event):
        self.get_logger().info("Current state: {0}".format(self.state))

def initialize_machine():
    rclpy.init()
    _sm = StateMachineNode()
    machine = GraphMachine(model=_sm, states=states ,**state_mach_params)
    machine.add_transition('crawl_in', source='initializing', dest='top_elbow')
    machine.add_transition('descend', source='top_elbow', dest='vertical_pipe')
    machine.add_transition('unwind', source='vertical_pipe', dest='bottom_elbow', conditions='full_unwind')
    machine.add_transition('enter_lateral', source='bottom_elbow', dest='lateral_crawl')
    machine.add_transition('check_end', source='lateral_crawl', dest='reached_end', conditions='end_reached')
    machine.add_transition('retrieve', source=['reached_end', 'horizontal_failure'], dest='retrieval')
    machine.add_transition('transition', source='retrieval', dest='vertical_retrieval')
    machine.add_transition('retrieve', source='vertical_failure', dest='vertical_retrieval')
    machine.add_transition('final_position', source='vertical_retrieval', dest='finalized')
    machine.add_transition('fail', source=['top_elbow','vertical_pipe'], dest='vertical_failure')
    machine.add_transition('fail', source=['lateral_crawl','bottom_elbow' ], dest='horizontal_failure')

    machine.machine_attributes['ratio'] = '0.5'
       
    # _sm.get_graph().draw('lgs_state_diagram.png', prog='dot')
    return _sm

def main(args=None):
    sm = initialize_machine()
    # sm.crawl_in(cmd = "forward_safe", times=1)
    # sm.descend(cmd = "forward_safe", times=top_elbow_buffer)
    sm.activate_crawler(cmd = "release_back", times=1)
    sm.unwind(cmd = "forward_safe", times=1)
    sm.enter_lateral(cmd="forward", times=int(0.8*feet_to_in(lateral_length)/crawler_rate))
    sm.check_end(cmd = "forward", times = 1)
    sm.retrieve(len = feet_to_in(lateral_length))
    while True:
        if sm.is_vert():
            time.sleep(1)
            break
        else:
            sm.activate_reel('retrieve')
            time.sleep(2)
            sm.activate_reel('stop')
    sm.transition(len = feet_to_in(vertical_height))
    while True:
        if sm.is_hori():
            time.sleep(1)
            break
        else:
            sm.activate_reel('retrieve')
            time.sleep(2)
            sm.activate_reel('stop')
    sm.final_position()
    sm.get_logger().info("Current State: {0}".format(sm.state))

if __name__ == "__main__":
    main()
