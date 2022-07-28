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

states = ['initializing',{'name':'top_elbow', 'on_enter':'crawl','on_exit':'hold'}, 'descent',
         {'name':'bottom_elbow', 'on_enter':'crawl'},
          'lateral_crawl','reached_end','retrieval','vertical_retrieval',
          'finalizing','horizontal_failure','horizontal_failure',]

state_mach_params = dict(initial='initializing', title='WRPS Lateral Gamma Scanner State Machine',
                  show_conditions=True, show_state_attributes=True)

def wait_for_input():
    gotime = False
    validselections =[0,1]
    while gotime == False:
            try:
                print('select')
                selection = int(input())
                print('selection made')
            except ValueError:
                print("Invalid option! Select again")
                continue
            if selection in validselections:    
                gotime == True
                break
            else:
                print("Invalid option! Select again")
    if selection == 0:
        return False
    if selection ==1:            
        return True

vertical_height = 60 # Inches
lateral_length = 54  # Inches
end_margin = 2       # Feet
crawler_rate= 8      # inches per extension 
# rc_synch = 19      # THIS SLEEP TIME MAINLY SYNCHRONIZES REEL AND CRAWLER
rc_synch = 6         # THIS SLEEP TIME MAINLY SYNCHRONIZES REEL AND CRAWLER
wind_rate = 1.3      # reel vel to inches 
unwind_rate = 1.2    # reel vel to inches 
top_elbow_buffer = 2 # Extensions to clear top elbow after IMU vertical pitch 


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
    "forward":      [1,   2.12, True], 
    "backward":     [-1,  2.1,  True], 
    "release":       [0,  0.0,  False],
    "engage_back":  [0,  0.0,  False],
    "release_back":  [0,  0.0,  False],
    "forward_safe": [1,   2.12, True], 
    "stop":         [0,   0.0, False],
    "unwind":       [2,   0.8, False],
    "retrieve":     [-2,  0.8,  True],
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

    def is_vertical(self, cmd ='stop', times = 1):
        self.crawl(cmd)
        self.get_logger().info("Checking if crawler has cleared top elbow")
        # self.pitch_vertical = False
        # while self.pitch_vertical == False:   
        #     self.future = self._pitch_client.call_async(self.imu_req)
        #     rclpy.spin_until_future_complete(self, self.future)
        #     self.pitch = abs(self.future.result().pitch)
        #     self.get_logger().info("Current inclination angle: {0}".format(self.pitch))
        #     if self.pitch > 75 and self.pitch < 100:
        #         self.pitch_vertical = True
        #         self.get_logger().info("Top elbow cleared")
        #         self.get_logger().info("Transitioning to state: {0}".format(self.state))
        #         break
        #     else:
        #         self.get_logger().info("Not cleared top elbow yet, crawling forward")
        #         self.crawl('forward_safe', 1)
        leggo = False
        while leggo == False:
            wego = wait_for_input()
            if wego == True:
                self.get_logger().info("Top elbow cleared")
                break
            else:
                self.crawl(cmd)
        return True        
 
    def unwind_complete(self, dir = "unwind", times = 1):
        self.elbow_is_reached = False
        self.unwound_length = 0.0
        while self.elbow_is_reached == False:
            if self.unwound_length >= vertical_height:
                self.elbow_is_reached = True
                self.get_logger().info("Bottom elbow reached")
                break
            else:
                self.get_logger().info("Bottom elbow not reached")
                self.activate_reel('unwind')
                self.unwound_length += reel_commands['unwind'][0]*wind_rate
                time.sleep(reel_commands['unwind'][1]+0.1)
        return True  

    def lateral_wind(self):
        self.elbow_is_reached = False
        self.retrieved_length = 0.0
        while self.elbow_is_reached == False:
            if self.retrieved_length >= lateral_length:
                self.elbow_is_reached = True
                self.get_logger().info("Bottom elbow passed")
                break
            else:
                self.activate_reel('retrieve')
                self.retrieved_length +=  abs(reel_commands['retrieve'][0])/wind_rate
                self.get_logger().info("Retrieved : {0}".format(self.retrieved_length))
                time.sleep(reel_commands['retrieve'][1]+0.1)
        self.activate_reel("stop")
        return True  
    
    def is_horizontal(self, cmd ='stop', times = 1):
        self.get_logger().info("Checking if crawler has cleared bottom elbow")
        self.pitch_horizontal = False
        # while self.pitch_horizontal == False:   
            # self.future = self._pitch_client.call_async(self.imu_req)
            # rclpy.spin_until_future_complete(self, self.future)
            # self.pitch = abs(self.future.result().pitch)
            # self.get_logger().info("Current inclination angle: {0}".format(self.pitch))
            # if self.pitch < 15 and self.pitch < 100:
                # self.pitch_horizontal = True
                # self.get_logger().info("Bottom elbow cleared")
                # self.get_logger().info("Transitioning to state: {0}".format(self.state))
                # time.sleep(2)
                # break
            # else:
                # self.get_logger().info("Not cleared bottom elbow yet, crawling forward")
                # self.crawl('forward_safe', 1)
        leggo = False
        while leggo == False:
            wego = wait_for_input()
            if wego == True:
                self.get_logger().info("Bottom elbow cleared")
                break
            else:
                self.crawl(cmd)
        return True        
        
    def end_reached(self):
        self.get_logger().info("Checking if crawler has reached end")
        self.has_reached_end = False
        # while self.has_reached_end == False:   
        #     self.future = self._distance_client.call_async(self.lidar_req)
        #     rclpy.spin_until_future_complete(self, self.future)
        #     self.future = self._distance_client.call_async(self.lidar_req)
        #     rclpy.spin_until_future_complete(self, self.future)
        #     self.d_to_end = abs(self.future.result().distance)
        #     self.get_logger().info("Distance until end: {0}".format(self.d_to_end))
        #     if self.d_to_end < end_margin:
        #         self.has_reached_end = True
        #         break
        #     else:
        #         self.get_logger().info("Not reached end yet, crawling forward")
        #         self.crawl('forward', 1)
        leggo = False
        while leggo == False:
            wego = wait_for_input()
            if wego == True:
                self.get_logger().info("Bottom elbow cleared")
                break
            else:
                self.crawl('forward', 1)
        return True     

    def activate_reel(self, cmd="forward"):
        self.get_logger().info("Sending {0} reel request ".format(cmd))
        reel_goal = Reelaction.Goal()
        reel_goal.reelcommand.reel_vel = reel_commands[cmd][0]
        reel_goal.reelcommand.interval = reel_commands[cmd][1]
        reel_goal.reelcommand.continuous = reel_commands[cmd][2]
        self._reel_client.send_goal_async(reel_goal)

    def crawl(self, dir='forward', times=1):
        self._crawler_client.wait_for_server(timeout_sec=2)
        self._reel_client.wait_for_server(timeout_sec=2)
        crawl_goal = Crawlaction.Goal()
        crawl_goal.crawlercommand.crawlpattern = crawler_commands[dir]
        crawl_goal.crawlercommand.continuous = False
        for i in range(times):
            self.get_logger().info("Sending {0} crawl request ".format(dir))
            self.activate_reel(dir)
            self._crawler_client.send_goal_async(crawl_goal)
            time.sleep(rc_synch) 
            self.activate_reel("stop")
            # if times > 1:
            self.hold()
    
    def hold(self, cmd = "stop", times = 1):
        self.get_logger().info("Pausing Crawl")
        time.sleep(2)

def initialize_machine():
    rclpy.init()
    _sm = StateMachineNode()
    machine = GraphMachine(model=_sm, states=states ,**state_mach_params)
    machine.add_transition('crawl_in', source='initializing', dest='top_elbow')
    machine.add_transition('check_angle', source='top_elbow', dest='descent', conditions='is_vertical', after ='crawl')
    machine.add_transition('unwind', source='descent', dest='bottom_elbow', conditions='unwind_complete')
    machine.add_transition('check_angle', source='bottom_elbow', dest='lateral_crawl', conditions='is_horizontal')
    machine.add_transition('advance', source='lateral_crawl', dest='reached_end', conditions='end_reached')
    machine.add_transition('retrieve', source=['reached_end', 'horizontal_failure'], dest='retrieval')
    machine.add_transition('transition', source='retrieval', dest='vertical_retrieval')
    machine.add_transition('retrieve', source='vertical_failure', dest='vertical_retrieval')
    machine.add_transition('final_position', source='vertical_retrieval', dest='finalizing')
    machine.add_transition('fail', source=['top_elbow','descent','bottom_elbow'], dest='vertical_failure')
    machine.add_transition('fail', source=['lateral_crawl','reached_end' ], dest='horizontal_failure')

    machine.machine_attributes['ratio'] = '0.5'
       
    _sm.get_graph().draw('lgs_state_diagram.png', prog='dot')
    return _sm

def main(args=None):
    sm = initialize_machine()
    sm.crawl("engage_back", 1)
    sm.get_logger().info("Current State: {0}".format(sm.state))
    sm.crawl_in("forward_safe", 1)
    time.sleep(1)
    sm.get_logger().info("Current State: {0}".format(sm.state))
    sm.check_angle("forward_safe", top_elbow_buffer)
    time.sleep(1)
    sm.crawl("release_back", 1)
    sm.unwind("engage_back", 1)
    sm.get_logger().info("Current State: {0}".format(sm.state))
    time.sleep(1) 
    sm.check_angle("forward_safe", 1)
    sm.get_logger().info("Current State: {0}".format(sm.state))
    time.sleep(1)
    sm.advance()
    sm.get_logger().info("Current State: {0}".format(sm.state))
    time.sleep(1)
    sm.retrieve()
    sm.get_logger().info("Current State: {0}".format(sm.state))
    time.sleep(1)
    sm.transition()
    sm.get_logger().info("Current State: {0}".format(sm.state))
    time.sleep(1)
    sm.final_position()
    sm.get_logger().info("Current State: {0}".format(sm.state))


if __name__ == "__main__":
    main()
