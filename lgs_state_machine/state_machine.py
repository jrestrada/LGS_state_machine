import time
import rclpy
from rclpy.node import Node
from pipecrawler.action import Crawlaction
from reel.action import Reelaction
from rclpy.action import ActionClient
from transitions import Machine
from transitions.extensions import GraphMachine

states = ['initializing','top_elbow','vertical_descent','bottom_elbow',
          'lateral_crawl','reached_end','retrieval','finalizing','failiure']

class StateMachineNode(Node):
    def __init__(self):
        super().__init__("state_machine")
        self.get_logger().info("State machine node is up")
        self._crawler_client = ActionClient(self, Crawlaction, 'crawl_through_pipe')
        self._reel_client = ActionClient(self, Reelaction, 'activate_reel' )

    def crawl_count_feedback(self, feedback):
        self.get_logger().info("received feednacl")
        self.get_logger().info('Received feedback: {0}'.format(feedback.jfeedback.count))

    def crawl(self, dir='forward'):
        crawl_goal = Crawlaction.Goal()
        reel_goal = Reelaction.Goal()
        reel_goal.reelcommand.reel_vel = 1
        reel_goal.reelcommand.interval = 0.2
        reel_goal.reelcommand.continuous = True
        crawl_goal.crawlercommand.crawlpattern = [2,5,3,6,1,4]
        crawl_goal.crawlercommand.continuous = True
        self.get_logger().info("Sending Crawl Request")
        self._crawler_client.wait_for_server(timeout_sec=5)
        self._crawler_client.send_goal_async(crawl_goal, feedback_callback = self.crawl_count_feedback)
        self._reel_client.wait_for_server(timeout_sec=5)
        self._reel_client.send_goal_async(reel_goal)

    def crawl2(self, dir='stop'):
        crawl_goal = Crawlaction.Goal()
        reel_goal = Reelaction.Goal()
        reel_goal.reelcommand.reel_vel = 0
        reel_goal.reelcommand.interval = 1.0
        reel_goal.reelcommand.continuous = False
        crawl_goal.crawlercommand.crawlpattern = [0,0,0,0,0,0]
        crawl_goal.crawlercommand.continuous = False
        self.get_logger().info("Sending Crawl 2 Request")
        self._crawler_client.wait_for_server(timeout_sec=5)
        self._crawler_client.send_goal_async(crawl_goal, feedback_callback= self.crawl_count_feedback)
        self._crawler_client.send_goal_async(crawl_goal)
        self._reel_client.wait_for_server(timeout_sec=5)
        self._reel_client.send_goal_async(reel_goal)


def main(args=None):
    rclpy.init(args=args)
    sm = StateMachineNode()
    machine = GraphMachine(model=sm, states=states, initial='initializing')
    machine.add_transition('init_position', source='initializing', dest='top_elbow')
    machine.add_transition('confirm_vertical', ['top_elbow'], None, after= 'crawl')
    machine.add_transition('descend', source='top_elbow', dest='vertical_descent')
    machine.add_transition('lat_transition', source='vertical_descent', dest='bottom_elbow')
    machine.add_transition('begin_crawl', source='bottom_elbow', dest='lateral_crawl')
    machine.add_transition('confirm_end', source='lateral_crawl', dest='reached_end')
    machine.add_transition('retrieve', source=['reached_end', 'failiure'], dest='retrieval')
    machine.add_transition('final_position', source='retrieval', dest='finalizing')
    machine.add_transition('fail', source=['top_elbow','vertical_descent','bottom_elbow','lateral_crawl' ], dest='failiure')
    sm.get_graph().draw('lgs_state_diagram.png', prog='dot')
    machine.on_exit_initializing('crawl')
    machine.on_exit_top_elbow('crawl2')
    # sm.init_position()
    # time.sleep(5)
    # sm.descend()

if __name__ == "__main__":
    main()
