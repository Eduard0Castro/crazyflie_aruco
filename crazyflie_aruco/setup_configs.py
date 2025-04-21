import cv2
import cv2.aruco as aruco
from pathlib import Path

class SetupConfig:

    PATH = Path(__file__).resolve().parent

    @staticmethod
    def aruco_config(marker_dict: int, 
                     total_markers: int) -> tuple[aruco.ArucoDetector]:
        
        """
        Function to set aruco configs

        :param marker_dict (int): aruco dictionary if 4x4 = 4, 5x5 = 5, ...
        :param total_markers (int): number of different markers

        """
        
        key = f"{getattr(aruco, f"DICT_{marker_dict}X{marker_dict}")}_{total_markers}"
        dictionary = aruco.getPredefinedDictionary(key)
        param = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, param)

        return detector, aruco

    @staticmethod
    def set_video_record(video_name: str,
                         video_dimension: tuple[int, int], 
                         frame_rate: float) -> None:
        """
        Function to set video recorder configs

        :param video_name (str): name for the video file to be saved. (Without extension.)
         Example: 
            name passed: detection
            video recorded: detection.mp4

        :param video_dimension (tuple[int, int]): video image dimension for the recorder
        :param frame_rate (float): the frame rate for record the video

        """
        fcode = cv2.VideoWriter.fourcc(*"mp4v")
        video_file_name = f"{SetupConfig.PATH}/{video_name}.mp4"
        recordVideo = cv2.VideoWriter(video_file_name, 
                                      fcode, 
                                      frame_rate, 
                                      video_dimension)
        
        return recordVideo

    @staticmethod
    def set_actions() -> None:

        """
        Function to set the aruco dictionary 
        """

        velocity = 0.5
        velocities = (0.0, 0.0, 0.0)
        t_start = None
        actions =           {100:("Forward",   (velocity, 0.0, 0.0)),
                             200:("Backward", (-velocity, 0.0, 0.0)),
                             300:("Left",      (0.0, velocity, 0.0)),
                             600:("Right",    (0.0, -velocity, 0.0)),
                             400:("Up",        (0.0, 0.0, velocity)),
                             800:("Down",     (0.0, 0.0, -velocity))}
        
        return velocities, t_start, actions



if __name__ == "__main__":
    SetupConfig.set_actions()
    SetupConfig.aruco_config()
    SetupConfig.set_video_record()