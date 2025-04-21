import rclpy 
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from time import sleep

class ControlLifecycle(Node):

    def __init__(self) -> None:

        super().__init__("control_crazyflie_lifecycle_node")
        self.declare_parameter("node_name", "crazyflie_aruco_control_node")
        self.__node_name = self.get_parameter("node_name").value
        self.control_client = self.create_client(ChangeState, 
                                        f"/{self.__node_name}/change_state")
        self.get_logger().info("Control aruco movement node has been initialized")

    def change_state(self, transition: Transition) -> None:
        
        self.control_client.wait_for_service()

        request = ChangeState.Request()
        request.transition = transition
        future = self.control_client.call_async(request=request)
        rclpy.spin_until_future_complete(self, future)


    def initialize_sequence(self) -> None:

        self.get_logger().info("Trying to change the state")

        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE

        self.get_logger().info("Transition from unconfigured to inactive state")
        self.change_state(transition=transition)

        sleep(0.5)

        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE

        self.get_logger().info("Transition from inactive to active state")
        self.change_state(transition)


def main(args = None) -> None:

    rclpy.init(args = args)

    try:
        control_lc = ControlLifecycle()
        control_lc.initialize_sequence()
    except KeyboardInterrupt:...
    except Exception as ex: print("Exception: ", ex)
    finally: control_lc.destroy_node()

if __name__ == "__main__":
    main()