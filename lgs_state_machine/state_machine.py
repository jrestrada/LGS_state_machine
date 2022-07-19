from html import unescape
from multiprocessing.sharedctypes import Value
import time
import rclpy
from rclpy.node import Node
from pipecrawler.action import Crawlaction
from fm_interfaces.srv import Checkimu
from reel.action import Reelaction
from rclpy.action import ActionClient
from std_msgs.msg import Int16
from transitions import Machine
from transitions.extensions import GraphMachine

states = ['initializing','top_elbow','vertical_descent','bottom_elbow',
          'lateral_crawl','reached_end','retrieval','finalizing','failiure']

extra_args = dict(initial='initializing', title='WRPS Lateral Gamma Scanner State Machine',
                  show_conditions=True, show_state_attributes=False)

class StateMachineNode(Node):
    def __init__(self):
        super().__init__("state_machine")
        self.declare_parameter('pipe_length', 160)
        self.get_logger().info("State machine node is up")
        self._crawler_client = ActionClient(self, Crawlaction, 'crawl_through_pipe')
        self._reel_client = ActionClient(self, Reelaction, 'activate_reel' )
        self.pitch_cli = self.create_client(Checkimu, 'pitch_service')
        while not self.pitch_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Checkimu.Request()
        self.req.request = 1

    def is_vertical(self):
        self.future = self.pitch_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.pitch = abs(self.future.result().pitch)
        self.get_logger().info("Current inclination angle: {0}".format(self.pitch))
        if self.pitch > 75:
            return True
        else:
            return False
        
    def is_horizntl(self):
        self.future = self.pitch_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.pitch = abs(self.future.result().pitch)
        self.get_logger().info("Inclination angle: {0}".format(self.pitch))
        if self.pitch < 15:
            return True
        else:
            return False
        
    def end_reached(self):
        length = 1
        return True

    def crawl_count_feedback(self, feedback):
        self.get_logger().info("received feednacl")
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.count))

    def crawl(self, dir='forward'):
        crawl_goal = Crawlaction.Goal()
        reel_goal = Reelaction.Goal()
        reel_goal.reelcommand.reel_vel = 1
        reel_goal.reelcommand.interval = 2.15
        reel_goal.reelcommand.continuous = True
        crawl_goal.crawlercommand.crawlpattern = [1,3,5,2,4,6]
        crawl_goal.crawlercommand.continuous = True
        self.get_logger().info("Sending {0} crawl request ".format(dir))
        self._crawler_client.wait_for_server(timeout_sec=2)
        self._crawler_client.send_goal_async(crawl_goal, feedback_callback = self.crawl_count_feedback)
        self._reel_client.wait_for_server(timeout_sec=2)
        self._reel_client.send_goal_async(reel_goal)

    def crawl2(self, dir='stop'):
        crawl_goal = Crawlaction.Goal()
        reel_goal = Reelaction.Goal()
        reel_goal.reelcommand.reel_vel = 0
        reel_goal.reelcommand.interval = 1.0
        reel_goal.reelcommand.continuous = False
        crawl_goal.crawlercommand.crawlpattern = [0,0,0,0,0,0]
        crawl_goal.crawlercommand.continuous = False
        self.get_logger().info("Sending {0} crawl request".format(dir))
        self._crawler_client.wait_for_server(timeout_sec=2)
        self._crawler_client.send_goal_async(crawl_goal, feedback_callback= self.crawl_count_feedback)
        self._crawler_client.send_goal_async(crawl_goal)
        self._reel_client.wait_for_server(timeout_sec=2)
        self._reel_client.send_goal_async(reel_goal)

def initialize_machine():
    rclpy.init()
    _sm = StateMachineNode()
    machine = GraphMachine(model=_sm, states=states ,**extra_args)
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

    # machine.on_exit_initializing('crawl')
    # machine.on_exit_top_elbow('crawl2')
    return _sm

def main(args=None):
    sm = initialize_machine()
    sm.get_logger().info("Current State: {0}".format(sm.state))
    sm.vert_transition()
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
