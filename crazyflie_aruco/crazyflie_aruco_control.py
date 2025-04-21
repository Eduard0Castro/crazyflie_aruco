#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from pathlib import Path
from viscap_utils.crazyflie.viscap_crazyflie import ViscapCrazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from time import time
from crazyflie_aruco.setup_configs import SetupConfig


class CrazyflieArucoControl(LifecycleNode):


    def __init__(self, single_crazyflie: bool = True) -> None:

        """
        Crazyflie Aruco Control Constructor

        Parameters
        ----------

        :param single_crazyflie (bool): True for a single crazyflie application. False for a
         swarm application
        """

        super().__init__("crazyflie_aruco_control_node")

        self.crazyflie = ViscapCrazyflie(self)
        self.single = single_crazyflie

        self.get_logger().info("Aruco movement node has been started")
             

    def __set_tools(self) -> None:

        self.get_logger().info("Setting up the configurations")
        self.recordVideo = SetupConfig().set_video_record("filmagem.mp4",
                                                          (324, 244),
                                                          30.0)
        self.detector, self.aruco = SetupConfig().aruco_config(5, 1000)
        self.previous_id = self.current_id = None
        self.velocities, self.t_start, self.actions = SetupConfig().set_actions()
        self.ai_deck = self.crazyflie.create_ai_deck()
        self.img = None
        

    def init_crazyflies(self) -> None:

        """
        Initialize the radio addresses and crazyflie swarm 
        """

        self.uris = {'radio://0/80/2M/E7E7E7E7E7',}
                    #  'radio://0/80/2M/E7E7E7E7E8',
                    #  'radio://0/80/2M/E7E7E7E7E9'}
        
        self.crazyflie.init_crazyflie_swarm(self.uris, self.swarm)


    def swarm(self, sync: SyncCrazyflie) -> None:

        """
        Function to pass to all crazyflies in the swarm
        """

        self.get_logger().info("Swarm application is started")
        sync.cf.param.set_value('stabilizer.controller', 1)
        motion = MotionCommander(sync)
        self.create_timer(lambda : self.__setattr__("img", self.ai_deck.get_image()))
        # motion.takeoff(1.0)

        while True:
            __stop = self.stream_cam(self.img, motion)
            if __stop: break


    def single_crazyflie(self) -> None:

        """
        Initialize the motion commander and take off the single drone
        """

        self.motion = self.crazyflie.init_crazyflie(True, 
                                "radio://0/80/2M/E7E7E7E7E7")
        # self.motion.take_off(1.0)


    def stream_cam(self, 
                   img: cv2.typing.MatLike, 
                   motion: MotionCommander = None) -> None | bool:

        """
        Get the image and control the drone with aruco actions dictionary

        :param img (cv2.typing.MatLike): cv2 image to do the aruco detection
        :param motion (MotionCommander): the motion commander to the specific crazyflie.
         It has been used for the swarm application.
        """
        
        frame = img
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        bbox, ids, _ = self.detector.detectMarkers(frame)
        motion = self.motion if self.single else motion

        if ids is not None:
            self.aruco.drawDetectedMarkers(img, bbox, ids)
            id_ = ids[0][0]
            self.previous_id = self.current_id
            self.current_id = id_
            
            if self.previous_id == self.current_id and self.previous_id is not None:
                if self.t_start is None:
                    self.t_start = time()
                
            else: self.t_start = None

            if self.t_start and time() - self.t_start >= 0.2:

                if id_ == 150 and self.single: 

                    self.get_logger().info("Landing the drone and cleaning application")
                    motion.land()
                    self.__clean_image()
                    return

                label, self.velocities = self.actions.get(id_, ("Unkown", (0.0, 0.0, 0.0)))
                corner  = bbox[0][0][0]
                x1 = 10
                y1 = 25
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(img, (x1, y1-5-th), (x1 + tw, y1+2), (0, 0, 0), -1)
                cv2.putText(img, label, (x1-1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (255, 255, 255), 1)
                
        else: self.velocities = (0.0, 0.0, 0.0)    


        vel_x, vel_y, vel_z = self.velocities
        # motion.start_linear_motion(vel_x, vel_y, vel_z)

        cv2.imshow("Detection", img)
        self.recordVideo.write(img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.recordVideo.release()
            cv2.destroyWindow("Detection")
            self.crazyflie.cleanup()
            return True


    def __clean_image(self)-> None:

        self.destroy_timer(self.crazyflie.ai_deck_timer)
        self.recordVideo.release()
        cv2.destroyAllWindows()


    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        
        self.__set_tools()
        self.init_crazyflies() if not self.single else self.single_crazyflie()
        self.crazyflie.cleaned = False

        return TransitionCallbackReturn.SUCCESS
    

    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:

        self.get_logger().info("Cleanup the settings")
        self.crazyflie.swarm.parallel_safe(self.__land_swarm) if not self.single \
                                                              else self.motion.land()
        self.crazyflie.cleanup()
        self.recordVideo.release()

        return TransitionCallbackReturn.SUCCESS
    

    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:

        super().on_activate(previous_state)

        self.get_logger().info("Initializing AI Deck image")
        if self.single: self.crazyflie.processing_ai_deck_image(self.stream_cam)

        return TransitionCallbackReturn.SUCCESS


    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:

        self.get_logger().info("In deactivate callback")
        super().on_deactivate(previous_state)

        if self.single: self.__clean_image()

        return TransitionCallbackReturn.SUCCESS


    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:

        self.get_logger().info("In on shutdown callback")
        self.motion.land()
        self.crazyflie.cleanup()

        return TransitionCallbackReturn.SUCCESS
    

def main(args = None) -> None:

    rclpy.init(args = args)
    try:
        aruco_movement = CrazyflieArucoControl(True)
        rclpy.spin(aruco_movement)

    except KeyboardInterrupt:...
    except Exception as ex: print(f"Exception: {ex}")
    finally:
        print("\nShutting down Crazyflie Aruco Movement")
        aruco_movement.crazyflie.cleanup()
        aruco_movement.destroy_node()

if __name__ == "__main__":
    main()

