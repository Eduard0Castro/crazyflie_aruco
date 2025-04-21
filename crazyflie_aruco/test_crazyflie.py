import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
from pathlib import Path
from viscap_utils.crazyflie.viscap_crazyflie import ViscapCrazyflie
from time import time


class AIDeckExample(LifecycleNode):

    PATH = Path(__file__).resolve().parent

    def __init__(self) -> None:

        super().__init__("ai_deck_example_node")
        self.crazyflie = ViscapCrazyflie(self)

        self.get_logger().info("Aruco crazyflie lifecycle node has been started")

    def __aruco_config(self) -> None:
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        param = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dictionary, param)
        self.previous_id = self.current_id = None

    def __set_video_record(self) -> None:

        fcode = cv2.VideoWriter.fourcc(*"mp4v")
        video_file_name = f"{AIDeckExample.PATH}/filmagem.mp4"
        videoDimension = (648, 488)
        frame_rate = 12.0
        self.recordVideo = cv2.VideoWriter(video_file_name, fcode, frame_rate, videoDimension)

    def __actions_config(self) -> None:
        self.velocity = 0.05
        self.velocities = (0.0, 0.0, 0.0)
        self.t_start = None
        self.actions = {0:  ("FORWARD",   (self.velocity, 0.0, 0.0)),
                        600:("BACKWARD", (-self.velocity, 0.0, 0.0)),
                        900:("LEFT",      (0.0, self.velocity, 0.0)),
                        200:("RIGHT",    (0.0, -self.velocity, 0.0)),
                        800:("UP",        (0.0, 0.0, self.velocity)),
                        100:("DOWN",     (0.0, 0.0, -self.velocity))}

    def stream_cam(self, img: cv2.typing.MatLike) -> None:
        
        img = cv2.resize(img, (648, 488))
        frame = img
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        bbox, ids, _ = self.detector.detectMarkers(frame)

        if ids:
            aruco.drawDetectedMarkers(img, bbox, ids)
            id_ = ids[0][0]
            self.previous_id = self.current_id
            self.current_id = id_
            
            if self.previous_id == self.current_id and self.previous_id is not None:
                if self.t_start is None:
                    self.t_start = time()
                
            else: self.t_start = None

            if self.t_start and time() - self.t_start >= 0.5:
                # if id_ == 205 and self.single: self.motion.land()

                label, self.velocities = self.actions.get(id_, ("Unkown", (0.0, 0.0, 0.0)))
                corner  = bbox[0][0][0]
                x1 = int(corner[0])-1
                y1 = int(corner[1])-5
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(img, (x1, y1-5-th), (x1 + tw, y1+2), (0, 255, 0), -1)
                cv2.putText(img, label, (x1-1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (255, 255, 255), 1)
        else: self.velocities = (0.0, 0.0, 0.0)    
 
        cv2.imshow("Teste", img)
        self.recordVideo.write(img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.recordVideo.release()
            cv2.destroyWindow("Teste")
            self.crazyflie.cleanup()
            self.recordVideo.release()

    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        
        self.get_logger().info("Setting up the configurations")
        self.__set_video_record()
        self.__aruco_config()
        self.__actions_config()

        self.crazyflie.create_ai_deck()

        # self.init_crazyflies() if not self.single else self.single_crazyflie()

        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:

        self.get_logger().info("Cleanup the settings")
        self.crazyflie.swarm.parallel_safe(self.__land_swarm) if not self.single \
                                                              else self.motion.land()
        self.crazyflie.cleanup()

        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:

        self.get_logger().info("Initialize aruco movement")
        self.crazyflie.processing_ai_deck_image(self.stream_cam)
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:


        self.destroy_timer(self.crazyflie.ai_deck_timer)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:

        self.get_logger().info("In on shutdown callbak")
        self.crazyflie.swarm.parallel_safe(self.__land_swarm) if not self.single \
            else self.motion.land()
        self.crazyflie.cleanup()

        return TransitionCallbackReturn.SUCCESS

def main(args = None) -> None:

    rclpy.init(args = args)
    try:
        ai_deck_example = AIDeckExample()
        rclpy.spin(ai_deck_example)

    except KeyboardInterrupt:...
    except Exception as ex: 
        print(ex)
        ai_deck_example.crazyflie.cleanup()
    finally: ai_deck_example.destroy_node()

if __name__ == "__main__":
    main()

