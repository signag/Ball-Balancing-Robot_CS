import paho.mqtt.client as mqtt
from picamera2 import Picamera2
import signal
import threading
import time
import datetime
import cv2
import json
import os
from pathlib import Path
import math
import numpy as np
from collections import deque
from flask import Flask, Response
import csv
import gc
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import math

import logging
#
# Robot configuration
#
HOMEDIR = os.getcwd()
if not os.path.exists(os.path.join(HOMEDIR, "config")):
    HOMEDIR = os.path.join(os.environ.get("HOME", os.getcwd()), "bb_robot_home")
CONFIG_DIR = os.path.join(HOMEDIR, "config")
os.makedirs(CONFIG_DIR, exist_ok=True)
DATA_DIR = os.path.join(HOMEDIR, "data")
os.makedirs(DATA_DIR, exist_ok=True)
ROBOT_CONFIG_FILE = "bb_robot_config.json"
#
#Initial Ball Position
x, y = 100, 75
#
# Logging configuration
#
logsPath = os.path.join(HOMEDIR, "logs")
os.makedirs(logsPath, exist_ok=True)
logFile = os.path.join(logsPath, "bb_robot.log")
Path(logFile).touch(exist_ok=True)
filehandler = logging.FileHandler(logFile)
filehandler.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s in %(name)s: %(message)s'))
streamhandler = logging.StreamHandler()
streamhandler.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s in %(name)s: %(message)s'))

logger = logging.getLogger("bb_robot_server")
logger.addHandler(streamhandler)
# logger.addHandler(filehandler)
logger.setLevel(logging.ERROR)

#
# Flask application
#
app = Flask(__name__)

#
# Global variables for camera and robot state
#
latest_frame = None
latest_image = None
frame_lock = threading.Lock()
image_lock = threading.Lock()
thread_lock = threading.Lock()
shutdown = False
stop_capture = False

class PID_Recorder:
    def __init__(self, save_path="pid_data.csv"):
        self.save_path = save_path
        self.samples = []

    def record(self, data):
        self.samples.append([
            data["dt"],
            data["err"][0], data["err"][1],
            data["err_i"][0], data["err_i"][1],
            data["err_d"][0], data["err_d"][1],
            data["theta"], data["phi"]
        ])

    def save(self):
        with open(self.save_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "dt",
                "err_x", "err_y",
                "sum_err_x", "sum_err_y",
                "d_err_x", "d_err_y",
                "theta", "phi"
            ])
            writer.writerows(self.samples)
        logger.debug("PID_Recorder.save - Saved %s samples → %s", len(self.samples), self.save_path)


class BallBalancingRobot:
    def __init__(self):
        """Initialize the ball balancing robot with specified parameters and set it to a default state.
        """
        logger.debug("Initializing BallBalancingRobot")

        # Load robot configuration from file if it exists
        self.mode = "manual"
        self.show_contours = False
        self.configuration = {
            "LP": 7.14,
            "L1": 7.50,
            "L2": 4.50,
            "LB": 5.56,
            "INVERT": False,
            "h_work": 9.53
        }
        self.calibration = {
            "theta1_offset": 0.0,
            "theta2_offset": 0.0,
            "theta3_offset": 0.0
        }
        self.pid_parameters = {
            "kp": 0.0063,
            "ki": 0.0006,
            "kd": 0.0060,
            "alpha": 0.65,
            "beta": 0.3
        }
        self.cam_parameters = {
            "resolution": (1640, 1232),
            "resolution_work": (200, 150),
            "center": (100, 75),
            "target": (100, 75),
            "detection_radius": 55, 
            "format": "RGB888"
        }
        
        self.read_config()

        self.robot = RobotKinematics(lp=self.configuration["LP"], l1=self.configuration["L1"], l2=self.configuration["L2"], lb=self.configuration["LB"], invert=self.configuration["INVERT"])
        self.controller = RobotController(self.robot, calibration=self.calibration)
        self.cam = Camera(self.cam_parameters)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fn = Path(DATA_DIR) / f"pid_data_{ts}.csv"
        self.pid_recorder = PID_Recorder(fn)
        self.record = False

        self._h = (self.robot.maxh + self.robot.minh) / 2
        self._theta = 0.0
        self._theta_max = self.robot.max_theta(self._h)
        self._phi = 0.0
        self._theta_rad = math.radians(self._theta)
        self._phi_rad   = math.radians(self._phi)

        self.pid = PIDcontroller(
            self.pid_parameters["kp"],
            self.pid_parameters["ki"],
            self.pid_parameters["kd"],
            self.pid_parameters["alpha"],
            self.pid_parameters["beta"],
            max_theta=self._theta_max,
            conversion="tanh",
            recorder=self.pid_recorder,
            record=self.record
        )

        self.set_orientation(0.0, 0.0, self.robot.minh)
        time.sleep(0.5)
        self.set_orientation(0.0, 0.0, self.robot.maxh)
        time.sleep(0.5)
        self.set_orientation(0.0, 0.0, self.configuration["h_work"])

    def read_config(self):
        fp = Path(CONFIG_DIR) / ROBOT_CONFIG_FILE
        if fp.is_file():
            logger.debug("Loading robot configuration from file: %s", fp)
            with open(fp, "r") as f:
                robot_config = json.load(f)
                if "mode" in robot_config:
                    self.mode = robot_config["mode"]
                if "configuration" in robot_config:
                    configuration = robot_config["configuration"]
                    self.configuration.update({
                        "LP": configuration.get("LP", self.configuration["LP"]),
                        "L1": configuration.get("L1", self.configuration["L1"]),
                        "L2": configuration.get("L2", self.configuration["L2"]),
                        "LB": configuration.get("LB", self.configuration["LB"]),
                        "INVERT": configuration.get("INVERT", self.configuration["INVERT"]),
                        "h_work": configuration.get("h_work", self.configuration["h_work"])
                    })
                logger.debug("Loaded robot configuration: %s", self.configuration)
                if "calibration" in robot_config:
                    calibration = robot_config["calibration"]
                    self.calibration.update({
                        "theta1_offset": calibration.get("theta1_offset", 0.0),
                        "theta2_offset": calibration.get("theta2_offset", 0.0),
                        "theta3_offset": calibration.get("theta3_offset", 0.0)
                    })
                    logger.debug("Loaded calibration parameters: %s", self.calibration)
                if "pid_parameters" in robot_config:
                    pid_parameters = robot_config["pid_parameters"]
                    self.pid_parameters.update({
                        "kp": pid_parameters.get("kp", self.pid_parameters["kp"]),
                        "ki": pid_parameters.get("ki", self.pid_parameters["ki"]),
                        "kd": pid_parameters.get("kd", self.pid_parameters["kd"]),
                        "alpha": pid_parameters.get("alpha", self.pid_parameters["alpha"]),
                        "beta": pid_parameters.get("beta", self.pid_parameters["beta"])
                    })
                    logger.debug("Loaded PID parameters: %s", self.pid_parameters)

                if "cam_parameters" in robot_config:
                    cam_parameters = robot_config["cam_parameters"]
                    self.cam_parameters.update({
                        "resolution": tuple(
                            cam_parameters.get("resolution", self.cam_parameters["resolution"])
                        ),
                        "resolution_work": tuple(
                            cam_parameters.get("resolution_work", self.cam_parameters["resolution_work"])
                        ),
                        "center": tuple(
                            cam_parameters.get("center", self.cam_parameters["center"])
                        ),
                        "target": tuple(
                            cam_parameters.get("target", self.cam_parameters["target"])
                        ),
                        "detection_radius": cam_parameters.get("detection_radius", self.cam_parameters["detection_radius"]),
                        "format": cam_parameters.get("format", self.cam_parameters["format"])
                    })
                    logger.debug("Loaded camera parameters: %s", self.cam_parameters)
        else:
            logger.debug("No robot configuration file found at %s, using default configuration", fp)
            robot_config = {}
            robot_config["mode"] = self.mode
            robot_config["configuration"] = self.configuration
            robot_config["calibration"] = self.calibration
            robot_config["pid_parameters"] = self.pid_parameters
            robot_config["cam_parameters"] = self.cam_parameters
            with open(fp, "w") as f:
                json.dump(robot_config, f, indent=4)
            logger.debug("Saved default robot configuration to file: %s", fp)

    @property
    def theta_max(self):
        return self._theta_max

    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, value):
        self._theta = min(value, self.theta_max)
        self._theta_rad = math.radians(self._theta)

    @property
    def phi(self):
        return self._phi

    @phi.setter
    def phi(self, value):
        self._phi = value % 360
        self._phi_rad   = math.radians(self._phi)

    @property
    def h(self):
        return self._h

    @h.setter
    def h(self, value):
        self._h = min(max(value, self.robot.minh), self.robot.maxh)
        self._theta_max = self.robot.max_theta(self._h)
        self.pid.max_theta = self._theta_max
        if self._theta > self._theta_max:
            self._theta = self._theta_max

    @property
    def x(self):
        self._x = math.sin(self._theta_rad) * math.cos(self._phi_rad)
        return self._x

    @property
    def y(self):
        self._y = math.sin(self._theta_rad) * math.sin(self._phi_rad)
        return self._y

    @property
    def z(self):
        self._z = math.cos(self._theta_rad)
        return self._z

    def set_orientation(self, theta:float, phi:float, h:float) -> bool:
        """Set the orientation of the robot

        Args:
            theta (float): angle between z-axis of surface normal
            phi (float): azimuth angle between x-axis of surface normal
            h (float): height of center of upper plane

        Returns:
            bool: True if orientation was set successfully, False otherwise
        """
        self.theta = theta
        self.phi = phi
        self.h = h
        success = True
        try:
            self.robot.solve_inverse_kinematics_vector(self.x, self.y, self.z, self.h)
            self.controller.set_motor_angles(
                math.degrees(math.pi*0.5 - self.robot.theta1), 
                math.degrees(math.pi*0.5 - self.robot.theta2), 
                math.degrees(math.pi*0.5 - self.robot.theta3)
            )       
        except Exception as e:
            logger.error("Error during robot initialization: %s", e)
            success = False
        return success

    def set_mode(self, params, response):
        """Set the robot's mode.
        """
        logger.debug("Setting BallBalancingRobot mode to %s", params.get("mode"))
        self.mode = params.get("mode")
        response["status"] = "success"
        response["message"] = ""
        response["state"] = self.state
        return response

    def reset(self, response):
        """Reset the robot to its default state.
        """
        global x, y
        
        logger.debug("Resetting BallBalancingRobot to default state")
        self.read_config()
        self.robot = RobotKinematics(lp=self.configuration["LP"], l1=self.configuration["L1"], l2=self.configuration["L2"], lb=self.configuration["LB"], invert=self.configuration["INVERT"])
        self.controller = RobotController(self.robot, calibration=self.calibration)
        self._h = self.configuration["h_work"]
        self._theta = 0.0
        self._theta_max = self.robot.max_theta(self._h)
        self._phi = 0.0
        self._theta_rad = math.radians(self._theta)
        self._phi_rad   = math.radians(self._phi)

        self.pid = PIDcontroller(
            self.pid_parameters["kp"],
            self.pid_parameters["ki"],
            self.pid_parameters["kd"],
            self.pid_parameters["alpha"],
            self.pid_parameters["beta"],
            max_theta=self._theta_max,
            conversion="tanh"
        )

        self.set_orientation(0.0, 0.0, self.robot.minh)
        time.sleep(0.5)
        self.set_orientation(0.0, 0.0, self.robot.maxh)
        time.sleep(0.5)
        self.set_orientation(0.0, 0.0, self.configuration["h_work"])

        self.cam.calibrate(self.cam_parameters)

        #Initialize Ball Position
        x, y = bb_robot.cam_parameters["target"]

        response["status"] = "success"
        response["message"] = ""
        response["state"] = self.state
        return response
    
    def update(self, params, response) -> dict:
        """Uptate the robot's state to given height and orientation of upper plane


        params:
            theta: angle between surface normal and z-axis
            phi  : azimuth angle between x-axis of surface normal
            h    : height of center of upper plane

        """
        logger.debug("BallBalancingRobot.update - params= %s", params)

        theta = float(params.get("theta"))
        phi = float(params.get("phi"))
        h = float(params.get("h"))
        logger.debug("Processing update with theta: %s, phi: %s, h: %s", theta, phi, h)

        if self.set_orientation(theta, phi, h) == True:
            response["status"] = "success"
            response["message"] = ""
            response["state"] = self.state
        else:
            response["status"] = "error"
            response["message"] = ""
        return response

    def calibrate(self, params, response) -> dict:
        """Calibrate the robot by setting the specified servo offset.

        Args:
            params (dict): A dictionary containing the servo to calibrate and the offset value.
            response (dict): A dictionary to be updated with the calibration result.

        Returns:
            dict: The updated response dictionary with calibration status and message.
        """
        logger.debug("BallBalancingRobot.calibrate - params= %s", params)

        servo = int(params.get("servo"))
        offset = float(params.get("offset"))

        response["status"] = "success"
        response["message"] = f"Calibrated {servo} with offset {offset}"

        if servo == 1:
            self.calibration["theta1_offset"] = offset
        elif servo == 2:
            self.calibration["theta2_offset"] = offset
        elif servo == 3:
            self.calibration["theta3_offset"] = offset
        else:
            response["status"] = "error"
            response["message"] = f"Invalid servo: {servo}"

        self.controller.calibrate(self.calibration)
        self.set_orientation(0.0, 0.0, self.h)

        return response

    def calibrate_cam(self, params, response) -> dict:
        """Calibrate the robot's camera by setting the specified parameters.

        Args:
            params (dict): A dictionary containing the camera calibration parameters.
            response (dict): A dictionary to be updated with the calibration result.

        Returns:
            dict: The updated response dictionary with calibration status and message.
        """
        logger.debug("BallBalancingRobot.calibrate_cam - params= %s", params)

        center = params.get("center")
        target = params.get("target")
        detection_radius = params.get("detection_radius")

        response["status"] = "success"
        response["message"] = f"Calibrated camera with center {center}, target {target}, and detection radius {detection_radius}"

        self.cam_parameters["center"] = center
        self.cam_parameters["target"] = target
        self.cam_parameters["detection_radius"] = detection_radius

        self.cam.calibrate(self.cam_parameters)

        return response

    def save_calibration(self, response) -> dict:
        """Save the current calibration parameters to the configuration file.

        Args:
            response (dict): A dictionary to be updated with the save result.

        Returns:
            dict: The updated response dictionary with save status and message.
        """
        config_path = Path(CONFIG_DIR) / ROBOT_CONFIG_FILE
        logger.debug("Saving calibration parameters to file %s", config_path)
        try:
            if config_path.is_file():
                with open(config_path, "r") as f:
                    robot_config = json.load(f)
            else:
                robot_config = {}

            robot_config["calibration"] = self.calibration
            robot_config["cam_parameters"] = self.cam_parameters

            with open(config_path, "w") as f:
                json.dump(robot_config, f, indent=4)

            logger.debug("Calibration parameters saved successfully: %s", robot_config)
            response["status"] = "success"
            response["message"] = "Calibration parameters saved successfully"
        except Exception as e:
            logger.error("Error saving calibration parameters: %s", e)
            response["status"] = "error"
            response["message"] = f"Error saving calibration parameters: {e}"
        return response

    def save_reset_pid(self, data: dict, response: dict) -> dict:
        """Save the given PID parameters and reset the PID controller with the new parameters.

        Args:
            data (dict): PID parameters
            response (dict): A dictionary to be updated with the save result.

        Returns:
            dict: The updated response dictionary with save status and message.
        """
        self.pid_parameters["kp"] = float(data.get("kp", self.pid_parameters["kp"]))
        self.pid_parameters["ki"] = float(data.get("ki", self.pid_parameters["ki"]))
        self.pid_parameters["kd"] = float(data.get("kd", self.pid_parameters["kd"]))
        self.pid_parameters["alpha"] = float(data.get("alpha", self.pid_parameters["alpha"]))
        self.pid_parameters["beta"] = float(data.get("beta", self.pid_parameters["beta"]))


        self.pid = PIDcontroller(
            self.pid_parameters["kp"],
            self.pid_parameters["ki"],
            self.pid_parameters["kd"],
            self.pid_parameters["alpha"],
            self.pid_parameters["beta"],
            max_theta=self._theta_max,
            conversion="tanh"
        )

        logger.debug("Saving PID parameters to file")
        config_path = Path(CONFIG_DIR) / ROBOT_CONFIG_FILE
        try:
            if config_path.is_file():
                with open(config_path, "r") as f:
                    robot_config = json.load(f)
            else:
                robot_config = {}

            robot_config["pid_parameters"] = self.pid_parameters

            with open(config_path, "w") as f:
                json.dump(robot_config, f, indent=4)

            logger.debug("PID parameters saved successfully")
            response["status"] = "success"
            response["message"] = "PID parameters saved successfully"
        except Exception as e:
            logger.error("Error saving PID parameters: %s", e)
            response["status"] = "error"
            response["message"] = f"Error saving PID parameters: {e}"
        return response
    
    def set_pid_recording(self, data: dict, response: dict) -> dict:
        """Enable or disable PID recording based on the given parameter.

        Args:
            data (dict): A dictionary containing the "record" key with a boolean value.
            response (dict): A dictionary to be updated with the recording status.

        Returns:
            dict: The updated response dictionary with recording status and message.
        """
        record = bool(data.get("record", False))
        self.record = record
        self.pid.record = record
        if record == True:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            fn = Path(DATA_DIR) / f"pid_data_{ts}.csv"
            self.pid_recorder = PID_Recorder(fn)
            self.pid.recorder = self.pid_recorder
            logger.debug("PID recording enabled")
            response["status"] = "success"
            response["message"] = "PID recording enabled"
            response["pid_recording"] = self.pid_recording_state
        else:
            self.pid_recorder.save()
            self.pid_recorder=None
            self.pid.recorder = None
            logger.debug("PID recording disabled")
            response["status"] = "success"
            response["message"] = "PID recording disabled"
            response["pid_recording"] = self.pid_recording_state
        return response
    
    def set_ctrl_params(self, data: dict, response: dict) -> dict:
        """Set control parameters based on the given data.

        Args:
            data (dict): A dictionary containing control parameters to be updated.
            response (dict): A dictionary to be updated with the new control parameters.

        Returns:
            dict: The updated response dictionary with new control parameters and message.
        """
        show_contours = bool(data.get("show_contours", self.show_contours))
        self.show_contours = show_contours
        logger.debug("Updated control parameters: show_contours=%s", self.show_contours)
        response["status"] = "success"
        response["message"] = "Control parameters updated successfully"
        response["state"] = self.state
        return response

    @property
    def pid_recording_state(self) -> dict:
        """Return the current state of PID recording, including whether it is enabled and how many samples have been recorded.
        """
        result = {
            "recording": self.record,
            "recorded": len(self.pid_recorder.samples) if self.pid_recorder else 0
        }
        return result

    @property
    def state(self) -> dict:
        """Return the current state of the robot, including angles, height, and motor positions.
        """
        result = {
            "mode": self.mode,
            "show_contours": self.show_contours,
            "theta": self.theta,
            "phi": self.phi,
            "h": self.h,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "theta1": math.degrees(self.robot.theta1),
            "theta2": math.degrees(self.robot.theta2),
            "theta3": math.degrees(self.robot.theta3),
            "theta_max": self.theta_max,
            "theta1_offset": self.calibration["theta1_offset"],
            "theta2_offset": self.calibration["theta2_offset"],
            "theta3_offset": self.calibration["theta3_offset"],
            "center": self.cam_parameters["center"],
            "target": self.cam_parameters["target"],
            "detection_radius": self.cam_parameters["detection_radius"]
        }
        return result

    @property
    def params(self) -> dict:
        """Return the robot's static (construction) parameters.
        """
        result = {
            "lp": self.robot.lp,
            "l1": self.robot.l1,
            "l2": self.robot.l2,
            "lb": self.robot.lb,
            "minh": self.robot.minh,
            "maxh": self.robot.maxh,
            "invert": self.robot.invert,
            "theta_ubound": self.robot.theta_ubound,
            "kp": self.pid_parameters["kp"],
            "ki": self.pid_parameters["ki"],
            "kd": self.pid_parameters["kd"],
            "alpha": self.pid_parameters["alpha"],
            "beta": self.pid_parameters["beta"]
            }
        return result

class Camera:
    #1640, 1232
    def __init__(self, camera_parameters: dict):
        logger.debug("Camera.__init__ - Initializing camera with camera_parameters: %s", camera_parameters)
        self.camera_parameters = camera_parameters
        self.center = camera_parameters["center"]
        self.target = camera_parameters["target"]
        self.detection_radius = camera_parameters["detection_radius"]
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={
                "size": camera_parameters["resolution"],
                "format": camera_parameters["format"]
            }, 
            controls={
                "FrameDurationLimits": (8333, 8333)
            }
        )
        self.picam2.configure(config)

        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 50])
        # self.upper_black = np.array([99, 99, 99])
        self.gray_threshold = 60

        self.queue = deque(maxlen=16)
        self.queue.append(self.camera_parameters["target"])  

        self.picam2.start()

    def calibrate(self, params):
        logger.debug("Camera.calibrate - params= %s", params)
        center = params.get("center")
        target = params.get("target")
        detection_radius = params.get("detection_radius")

        self.camera_parameters["center"] = center
        self.camera_parameters["target"] = target
        self.camera_parameters["detection_radius"] = detection_radius
        self.center = center
        self.target = target
        self.detection_radius = detection_radius

    def take_picture(self):
        image = self.picam2.capture_array()
        frame_resized = cv2.resize(image, self.camera_parameters["resolution_work"])
        return frame_resized

    def draw_position(self, image, pos):
        x0, y0 = self.center
        cv2.line(image, (x0 - 10, y0), (x0 + 10, y0), (0, 255, 0), 2)
        cv2.line(image, (x0, y0 - 10), (x0, y0 + 10), (0, 255, 0), 2)
        cv2.circle(
            image,
            center=self.center,
            radius=self.detection_radius,
            color=(0, 255, 0),
            thickness=2
        )        
        x0, y0 = self.target
        cv2.line(image, (x0 - 10, y0), (x0 + 10, y0), (0, 255, 255), 2)
        cv2.line(image, (x0, y0 - 10), (x0, y0 + 10), (0, 255, 255), 2)

        x, y = pos
        cv2.line(image, (x - 10, y), (x + 10, y), (0, 0, 255), 2)
        cv2.line(image, (x, y - 10), (x, y + 10), (0, 0, 255), 2)
        return image

    def terminate(self):
        self.picam2.stop()
        cnt = 0
        while self.picam2.started == True:
            time.sleep(0.01)
            cnt += 1
            if cnt > 100:
                logger.warning("Camera did not stop after 1 second, forcing shutdown")
                break
        # Close camera
        if self.picam2.is_open == True:
            self.picam2.close()
        # garbage collection
        gc.collect()
        cv2.destroyAllWindows()

    def coordinate(self, image, dispplay_image=None):
        
        prev_time = time.time()

        # Apply Gaussian blur.
        frame_blurred = cv2.GaussianBlur(image, (3, 3), 0)
        
        # Convert from BGR to HSV.
        frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)
        frame_gray = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2GRAY)

        #Filter based on Darkness + HSV
        mask_hsv = cv2.inRange(frame_hsv, self.lower_black, self.upper_black)
        mask_gray = cv2.threshold(frame_gray, self.gray_threshold, 255, cv2.THRESH_BINARY_INV)[1]
        mask_combined = cv2.bitwise_or(mask_hsv, mask_gray)

        #Process Edges
        mask_eroded = cv2.erode(mask_combined, None, iterations=1)
        mask_dilated = cv2.dilate(mask_eroded, None, iterations=1)

        # --- Find Contours (circles)
        valid_detections = []
        # contours, _ = cv2.findContours(mask_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours, _ = cv2.findContours(mask_dilated.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Minimum Enclosing Circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            radius = int(radius)

            # Ignore small objects
            # if radius < 5 or radius > 100:  # Adjust min/max radius based on expected size
            if radius < 5 or radius > self.detection_radius:  # Adjust min/max radius based on expected size
                if dispplay_image is not None:
                    cv2.drawContours(dispplay_image, [contour], -1, (255, 255, 0), 1)
                continue

            #Compute Circularity 4π(Area / Perimeter²)
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                if dispplay_image is not None:
                    cv2.drawContours(dispplay_image, [contour], -1, (255, 255, 0), 1)
                continue
            circularity = (4 * np.pi * area) / (perimeter ** 2)
            if circularity < 0.6:  # Threshold to eliminate non-circular objects
                if dispplay_image is not None:
                    cv2.drawContours(dispplay_image, [contour], -1, (0, 255, 255), 1)
                continue

            if dispplay_image is not None:
                cv2.drawContours(dispplay_image, [contour], -1, (0, 0, 255), 2)
            # Compute Aspect Ratio of Bounding Box
            x, y, w, h = cv2.boundingRect(contour)
            xd = int(x + w / 2)
            yd = int(y + h / 2)

            # Check that detection is within the detection radius around the center
            dx = xd - self.center[0]
            dy = yd - self.center[1]
            distance = np.sqrt(dx**2 + dy**2)

            # If the contour passes all filters
            if distance <= self.detection_radius:
                valid_detections.append((area, (xd, yd)))


        # logger.debug("coordinate - Found %d valid detections from %d contours", len(valid_detections), len(contours))
        if valid_detections:
            best_center = max(valid_detections, key=lambda item: item[0])[1]  
            self.queue.append(best_center) 
        else:
            if False and len(self.queue) >= 5 and self.queue[-1] == self.queue[-2] == self.queue[-3] == self.queue[-4] == self.queue[-5]:
                self.queue.append((100, 75))
            else:
                self.queue.append(self.queue[-1])

        x, y = self.queue[-1]

        return self.queue[-1]

class RobotController:
    def __init__(self, model, calibration=None):
        logger.debug("RobotController.__init__ - Initializing with calibration=%s", calibration)
        self.robot = model
        self.calibration = calibration or {"theta1_offset": 0.0, "theta2_offset": 0.0, "theta3_offset": 0.0}
        
        # Initialize the ServoKit and assign servos
        self.Controller = ServoKit(channels=16)
        self.s1 = self.Controller.servo[14]
        self.s2 = self.Controller.servo[15]
        self.s3 = self.Controller.servo[13]

        # Configure servos
        for s in (self.s1, self.s2, self.s3):
            s.actuation_range = 270
            s.set_pulse_width_range(500, 2500)

        self.initialize()

    def initialize(self):
      
        logger.debug("RobotController.initialize - Initializing ...")
        #self.set_motor_angles(54, 54, 54)
        #self.interpolate_time([19, 19, 19], duration=0.25)
        #time.sleep(1)
        #self.interpolate_time([90, 90, 90], duration=0.25)
        #time.sleep(1)
        #self.Goto_time_spherical(0, 0, 8.26, t=0.25)
        #time.sleep(1)
        logger.debug("RobotController.initialize - Initialized")

    def clamp(self, value, lower=19, upper=90):
        return max(lower, min(value, upper))
    
    def set_motor_angles(self, theta1, theta2, theta3):
        # Calibrate offsets 
        self.s1.angle = self.clamp(theta1 + self.calibration["theta1_offset"])
        self.s2.angle = self.clamp(theta2 + self.calibration["theta2_offset"])
        self.s3.angle = self.clamp(theta3 + self.calibration["theta3_offset"])

    def interpolate_time(self, target_angles, steps=100, duration=0.3, individual_durations=None):
 
        current_angles = [self.s1.angle, self.s2.angle, self.s3.angle]
        if individual_durations is None:
            individual_durations = [duration] * 3
        max_duration = max(individual_durations)
        steps = max(1, int(max_duration / 0.01))
        for i in range(steps + 1):
            t = i * max_duration / steps
            angles = [
                c + (t_angle - c) * min(t / d, 1) if d > 0 else t_angle 
                for c, t_angle, d in zip(current_angles, target_angles, individual_durations)
            ]
            self.set_motor_angles(*angles)
            time.sleep(max_duration / steps)

    def interpolate_speed(self, target_angles, speed=30, individual_speeds=None):
   
        current_angles = [self.s1.angle, self.s2.angle, self.s3.angle]
        if individual_speeds is None:
            individual_speeds = [speed] * 3
        durations = [
            abs(t - c) / s if s > 0 else 0 
            for c, t, s in zip(current_angles, target_angles, individual_speeds)
        ]
        max_duration = max(durations)
        steps = max(1, int(max_duration / 0.01))
        for i in range(steps + 1):
            t = i * max_duration / steps
            angles = [
                c + (t_angle - c) * min(t / d, 1) if d > 0 else t_angle 
                for c, t_angle, d in zip(current_angles, target_angles, durations)
            ]
            self.set_motor_angles(*angles)
            time.sleep(max_duration / steps)

    def Goto_time_spherical(self, theta, phi, h, t=0.5):
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]
        self.interpolate_time(target_angles, duration=t)

    def Goto_time_vector(self, a, b, c, h, t=0.5):
        self.robot.solve_inverse_kinematics_vector(a, b, c, h)
        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]
        self.interpolate_time(target_angles, duration=t)

    def Goto_N_time_vector(self, a, b, c, h):
        self.robot.solve_inverse_kinematics_vector(a, b, c, h)
        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]
        self.set_motor_angles(*target_angles)
    
    def Goto_N_time_spherical(self, theta, phi, h):
        
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]
        #print(theta, phi, target_angles)
        self.set_motor_angles(*target_angles)

    '''
    def Goto_Speed(self, alpha, beta, gamma, h, speed=240):

        gamma_ = max(math.sin(math.pi * 5 / 12), gamma)
        self.robot.solve_inverse_kinematics(alpha, beta, gamma_, h)
        target_angles = [
            radians_to_degrees(math.pi * 0.5 - self.robot.theta1),
            radians_to_degrees(math.pi * 0.5 - self.robot.theta2),
            radians_to_degrees(math.pi * 0.5 - self.robot.theta3)
        ]
        self.interpolate_speed(target_angles, speed=speed)

    def Goto_NOPOLATE(self, alpha, beta, gamma, h):
  
        gamma_ = max(math.sin(math.pi * 5 / 12), gamma)
        self.robot.solve_inverse_kinematics(alpha, beta, gamma_, h)
        self.set_motor_angles(
            radians_to_degrees(math.pi * 0.5 - self.robot.theta1),
            radians_to_degrees(math.pi * 0.5 - self.robot.theta2),
            radians_to_degrees(math.pi * 0.5 - self.robot.theta3)
        )
    '''

    def calibrate(self, calibration:dict) ->None:
        self.calibration = calibration or {"theta1_offset": 0.0, "theta2_offset": 0.0, "theta3_offset": 0.0}
        

    def Dance1(self):
 
        self.Goto_time_vector(0.258819045103, 0, 0.965925826289, 8)
        for _ in range(3):
            for i in range(100):
                t = (2 * math.pi / 100) * i
                x = math.cos(math.pi * 5 / 12) * math.cos(t)
                y = math.cos(math.pi * 5 / 12) * math.sin(t)
                z = math.sin(math.pi * 5 / 12)
                print(x, y, z, math.sqrt(x**2 + y**2 + z**2))
                self.Goto_N_time_vector(x, y, z, 8)
                time.sleep(1/100)
        self.Goto_time_vector(0, 0, 1, 8)

class PIDcontroller:
    def __init__(self, kp, ki, kd, alpha, beta, max_theta, conversion="linear", recorder=None, record:bool=False): 

        self.kp, self.ki, self.kd = kp, ki, kd
        self.alpha = alpha  #Exponential Filter: α⋅x + (1-α)⋅x_last
        self.beta = beta  #Coefficient for converting magnitude, either βx or tanh(βx)
        self.max_theta = max_theta
        self.recorder = recorder
        self.record = record
        
        if conversion == "linear":
            self.magnitude_convert = 1 #Linear
        elif conversion == "tanh":
            self.magnitude_convert = 0 #Tanh
        else:
            self.magnitude_convert = -1
 
        self.prev_out_x = 0.0
        self.prev_err_x = 0.0  
        self.prev_out_y = 0.0
        self.prev_err_y = 0.0

        self.sum_err_x = 0.0  #Integral
        self.sum_err_y = 0.0  #Integral
        
        self.last_time = None

    def pid(self, target, current):

        #dt
        new_time = time.perf_counter()
        dt = new_time - self.last_time if self.last_time is not None else 0.001

        #errors
        err_x0 = current[0] - target[0]
        err_y0 = current[1] - target[1]

        # Rottate errors by 90 degrees to align with robot's coordinate system
        err_x = - err_y0
        err_y = err_x0

        self.sum_err_x += err_x * dt
        self.sum_err_y += err_y * dt
        d_err_x = (err_x - self.prev_err_x) / dt if dt > 0 else 0
        d_err_y = (err_y - self.prev_err_y) / dt if dt > 0 else 0

        #output
        pid_x = self.kp * err_x + self.ki * self.sum_err_x + self.kd * d_err_x
        pid_y = self.kp * err_y + self.ki * self.sum_err_y + self.kd * d_err_y
        filtered_x = self.alpha * pid_x + (1 - self.alpha) * self.prev_out_x
        filtered_y = self.alpha * pid_y + (1 - self.alpha) * self.prev_out_y
        
        #Convert to spherical coordinates
        phi = math.degrees(math.atan2(filtered_y, filtered_x))
        phi = phi + 180
        if phi < 0:
            phi += 360
        r = math.sqrt(filtered_x**2 + filtered_y**2)
        if self.magnitude_convert == 1:
            theta = min(max(0, self.beta*r), self.max_theta)
        else:
            theta = max(0, 15*math.tanh(self.beta*r))


        self.prev_err_x = err_x
        self.prev_err_y = err_y
        self.prev_out_x = filtered_x
        self.prev_out_y = filtered_y
        self.last_time = new_time

        data = {
            "dt": dt,
            "target": target,
            "current": current,
            "err_abs": math.sqrt(err_x**2 + err_y**2),
            "err": (err_x, err_y),
            "err_i": (self.sum_err_x, self.sum_err_y),
            "err_d": (d_err_x, d_err_y),
            "pid": (pid_x, pid_y),
            "pid_f": (filtered_x, filtered_y),
            "theta": theta,
            "phi": phi
        }
        if self.record and self.recorder is not None:
            self.recorder.record(data)

        return theta, phi, data

class RobotKinematics:

    def __init__(self, lp=7.125, l1=6.20, l2=4.50, lb=4.00, invert=False):

        logger.debug("RobotKinematics.__init__ - Initializing with parameters: lp=%s, l1=%s, l2=%s, lb=%s, invert=%s", lp, l1, l2, lb, invert)

        self.lp = lp    #Radius of Top
        self.l1 = l1    #Top Arm
        self.l2 = l2    #Bottom Arm
        self.lb = lb    #Radius of Bottom
        self.invert = invert    #Whether the arm stays inward or outward

        self.maxh = self.compute_maxh() - 0.2    #maximum height that the Top plane should be 
        self.minh = self.compute_minh() + 0.45
        self.p = [0.0,0.0,self.maxh]    #Center of the Top plane
        self.maxtheta = 10
        self.max_theta((self.maxh + self.minh)/2)
        self.theta_ubound = self.maxtheta

        #Top Nodes
        self.A1 = [0,0,0] 
        self.A2 = [0,0,0]
        self.A3 = [0,0,0]

        #Bottom Nodes
        self.B1 = [0,0,0]
        self.B2 = [0,0,0]
        self.B3 = [0,0,0]

        #Middle Nodes
        self.C1 = [0.0, 0.0, 0.0]
        self.C2 = [0.0, 0.0, 0.0]
        self.C3 = [0.0, 0.0, 0.0]


        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0

    def compute_maxh(self):
        return math.sqrt(((self.l1 + self.l2) ** 2) - ((self.lp - self.lb) ** 2))

    def compute_minh(self):
        if self.l1 > self.l2:
            return math.sqrt((self.l1 ** 2) - ((self.lb + self.l2 - self.lp) ** 2))
        elif self.l2 > self.l1:
            return math.sqrt(((self.l2 - self.l1) ** 2) - ((self.lp - self.lb) ** 2))
        else:
            return 0
    
    def solve_top(self, a, b, c, h): #Orientation vector n: [a:x, b:y, c:z], h: height
        
        if not self.invert:
            #A1, A2, A3 are ball-joint coordinates, A2 is the vertex on plane y=0
            self.A1 = [ -(self.lp*c) / (math.sqrt(4*c**2 + (a - math.sqrt(3)*b)**2)),
                (math.sqrt(3)*self.lp*c) / (math.sqrt(4*c**2 + (a - math.sqrt(3)*b)**2)),
                h + ((a - math.sqrt(3)*b)*self.lp) / (math.sqrt(4*c**2 + (a - math.sqrt(3)*b)**2))]
            
            self.A2 = [ (self.lp*c) / (math.sqrt(c**2 + a**2)),
                    0,
                        h - ((self.lp*a) / (math.sqrt(c**2 + a**2)))]
            
            self.A3 = [ -(self.lp*c) / (math.sqrt(4*c**2 + (a + math.sqrt(3)*b)**2)),
                -(math.sqrt(3)*self.lp*c) / (math.sqrt(4*c**2 + (a + math.sqrt(3)*b)**2)),
                h + ((a + math.sqrt(3)*b)*self.lp) / (math.sqrt(4*c**2 + (a + math.sqrt(3)*b)**2))]
        else:
            self.A1 = [ -(self.lp*c) / (math.sqrt(4*c**2 + (a - math.sqrt(3)*b)**2)),
                (math.sqrt(3)*self.lp*c) / (math.sqrt(4*c**2 + (a - math.sqrt(3)*b)**2)),
                h + ((a - math.sqrt(3)*b)*self.lp) / (math.sqrt(4*c**2 + (a - math.sqrt(3)*b)**2))]
            
            self.A2 = [ (self.lp*c) / (math.sqrt(c**2 + a**2)),
                    0,
                        h - ((self.lp*a) / (math.sqrt(c**2 + a**2)))]
            
            self.A3 = [ -(self.lp*c) / (math.sqrt(4*c**2 + (a + math.sqrt(3)*b)**2)),
                -(math.sqrt(3)*self.lp*c) / (math.sqrt(4*c**2 + (a + math.sqrt(3)*b)**2)),
                h + ((a + math.sqrt(3)*b)*self.lp) / (math.sqrt(4*c**2 + (a + math.sqrt(3)*b)**2))]


    def solve_middle(self):


        a11, a12, a13 = map(float, self.A1)
        a21, a22, a23 = map(float, self.A2)
        a31, a32, a33 = map(float, self.A3)


        p1 = (-a11 + math.sqrt(3)*a12 - 2*self.lb) / a13
        q1 = (a11**2 + a12**2 + a13**2 + self.l2**2 - self.l1**2 - self.lb**2) / (2*a13)
        r1 = p1**2 + 4
        s1 = 2*p1*q1 + 4*self.lb
        t1 = q1**2 + self.lb**2 - self.l2**2

        p2 = (self.lb - a21) / a23
        q2 = (a21**2 + a23**2 - self.lb**2 + self.l2**2 - self.l1**2) / (2*a23)
        r2 = p2**2 + 1
        s2 = 2*(p2*q2 - self.lb)
        t2 = q2**2 - self.l2**2 + self.lb**2

        p3 = (-a31 - math.sqrt(3)*a32 - 2*self.lb) / a33
        q3 = (a31**2 + a32**2 + a33**2 + self.l2**2 - self.l1**2 - self.lb**2) / (2*a33)
        r3 = p3**2 + 4
        s3 = 2*p3*q3 + 4*self.lb
        t3 = q3**2 + self.lb**2 - self.l2**2

        if not self.invert:

            c11 = (-s1 - math.sqrt(s1**2 - 4*r1*t1)) / (2*r1)
            c12 = -math.sqrt(3) * c11
            c13 = math.sqrt(self.l2**2 - 4*(c11**2) - 4*self.lb*c11 - self.lb**2)

            self.C1 = [c11, c12, c13]

            c21 = (-s2 + math.sqrt(s2**2 - 4*r2*t2)) / (2*r2)
            c22 = 0
            c23 = math.sqrt(self.l2**2 - (c21 - self.lb)**2)

            self.C2 = [c21, c22, c23]

            c31 = (-s3 - math.sqrt(s3**2 - 4*r3*t3)) / (2*r3)
            c32 =  math.sqrt(3) * c31
            c33 = math.sqrt(self.l2**2 - 4*(c31**2) - 4*self.lb*c31 - self.lb**2)

            self.C3 = [c31, c32, c33]

        else:

            c11 = (-s1 - math.sqrt(s1**2 - 4*r1*t1)) / (2*r1)
            c12 = -math.sqrt(3) * c11
            c13 = math.sqrt(self.l2**2 - 4*(c11**2) - 4*self.lb*c11 - self.lb**2)

            self.C1 = [c11, c12, -c13]

            c21 = (-s2 + math.sqrt(s2**2 - 4*r2*t2)) / (2*r2)
            c22 = 0
            c23 = math.sqrt(self.l2**2 - (c21 - self.lb)**2)

            self.C2 = [c21, c22, -c23]

            c31 = (-s3 - math.sqrt(s3**2 - 4*r3*t3)) / (2*r3)
            c32 = math.sqrt(3) * c31
            c33 = math.sqrt(self.l2**2 - 4*(c31**2) - 4*self.lb*c31 - self.lb**2)

            self.C3 = [c31, c32, -c33]

    def solve_inverse_kinematics_vector(self, a, b, c, h):

        self.B1 = [-0.5*self.lb, math.sqrt(3)*0.5*self.lb,0]
        self.B2 = [self.lb,0,0]
        self.B3 = [-0.5*self.lb, -1*math.sqrt(3)*0.5*self.lb,0]

        self.solve_top(a, b, c, h)
        self.solve_middle()

        self.theta1 = math.pi/2 - math.atan2(math.sqrt(self.C1[0]**2 + self.C1[1]**2) - self.lb, self.C1[2])
        self.theta2 = math.atan2(self.C2[2], self.C2[0] - self.lb)
        self.theta3 = math.pi/2 - math.atan2(math.sqrt(self.C3[0]**2 + self.C3[1]**2) - self.lb, self.C3[2])

    def solve_inverse_kinematics_spherical(self, theta, phi, h): #psi = azimuthal angle, theta = polar angle

        #conversion
        self.max_theta(h)

        theta = min(theta, self.maxtheta)

        a = math.sin(math.radians(theta)) * math.cos(math.radians(phi))
        b  =  math.sin(math.radians(theta)) * math.sin(math.radians(phi))
        c = math.cos(math.radians(theta))
        
        try:
            self.solve_inverse_kinematics_vector(a,b,c,h)
        except Exception as e:
            print(a,b,c,h, theta, phi)
            pass


    def max_theta(self, h, tol=1e-3):
        theta_low, theta_high = 0.0, math.radians(20)
        def valid(theta):
            c = math.cos(theta)
            for s in (1, -1):
                a21 = self.lp * c
                a23 = h - self.lp * (s * math.sin(theta))
                try:
                    p2 = (self.lb - a21) / a23
                    q2 = (a21**2 + a23**2 - self.lb**2 + self.l2**2 - self.l1**2) / (2 * a23)
                    r2 = p2**2 + 1
                    s2 = 2 * (p2 * q2 - self.lb)
                    t2 = q2**2 - self.l2**2 + self.lb**2
                    disc = s2**2 - 4 * r2 * t2
                    if disc < 0: return False
                    c21 = (-s2 + math.sqrt(disc)) / (2 * r2)
                    delta = self.l2**2 - (c21 - self.lb)**2
                    if delta < 0: return False
                    c23 = math.sqrt(delta)
                    if abs(math.sqrt((a21-c21)**2 + (a23-c23)**2) - self.l1) > 1e-3: return False
                    if abs(math.sqrt((self.lb-c21)**2 + c23**2) - self.l2) > 1e-3: return False
                except:
                    return False
            return True
        while theta_high - theta_low > tol:
            theta_mid = (theta_low + theta_high) / 2
            if valid(theta_mid): theta_low = theta_mid
            else: theta_high = theta_mid

        self.maxtheta = max(0, math.degrees(round(theta_low, 4)) - 0.5)
        return self.maxtheta

def on_connect(client, userdata, flags, reason_code, properties):
    """Handle MQTT connect

    Args:
        client (mqtt.Client): The MQTT client instance.
        userdata (any): The private user data as set in Client() or userdata_set().
        flags (dict): Response flags sent by the broker.
        reason_code (int): The reason code for the connection.
        properties (mqtt.Properties): The properties of the MQTT message.
    """
    logger.debug("Connected with reason code: %s", reason_code)
    client.subscribe("robot/request")
    logger.debug("Subscribed to topic: robot/request")

def on_message(client, userdata, msg):
    """Message event handler

    Args:
        client (mqtt.Client): The MQTT client instance.
        userdata (any): The private user data as set in Client() or userdata_set().
        msg (mqtt.MQTTMessage): An instance of MQTTMessage, which contains topic, payload, qos, retain.
    """
    global bb_robot
    data = json.loads(msg.payload.decode())
    logger.debug("Received: %s", data)

    method = data.get("method")
    params = data.get("params", {})

    response = {
        "status": "error",
        "received": data,
        "message": "Invalid method"
    }
    if method == "get_state":
        response["status"] = "success"
        response["state"] = bb_robot.state
        response["pid_recording"] = bb_robot.pid_recording_state
        response["message"] = ""
    if method == "get_data":
        response["status"] = "success"
        response["params"] = bb_robot.params
        response["state"] = bb_robot.state
        response["pid_recording"] = bb_robot.pid_recording_state
        response["message"] = ""
    if method == "update":
        response = bb_robot.update(params, response)
    if method == "reset":
        response = bb_robot.reset(response)
    if method == "set_mode":
        response = bb_robot.set_mode(params, response)
        response["state"] = bb_robot.state
    if method == "calibrate":
        response = bb_robot.calibrate(params, response)
        response["state"] = bb_robot.state
    if method == "calibrate_cam":
        response = bb_robot.calibrate_cam(params, response)
        response["state"] = bb_robot.state
    if method == "save_calibration":
        response = bb_robot.save_calibration(response)
        bb_robot.mode = "manual"
        response["state"] = bb_robot.state
    if method == "simulate_pid":
        response = simulate_pid(bb_robot, response)
        response["state"] = bb_robot.state
    if method == "save_reset_pid":
        response = bb_robot.save_reset_pid(params, response)
        response = simulate_pid(bb_robot, response)
        response["params"] = bb_robot.params
        response["state"] = bb_robot.state
    if method == "set_pid_recording":
        response = bb_robot.set_pid_recording(params, response)
    if method == "set_ctrl_params":
        response = bb_robot.set_ctrl_params(params, response)
    
    client.publish("robot/response", json.dumps(response))
    logger.debug("Sent response: %s", response)

def shutdown(sig, frame):
    logger.debug("Shutting down...")
    global shutdown
    with thread_lock:
        shutdown = True
    client.disconnect()   # 👈 stops loop_forever
    sys.exit(0)

# --- MJPEG streaming ---
def generate_frames():
    while True:
        with image_lock:
            frame = latest_image.copy() if latest_image is not None else None

        if frame is not None:
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   jpg_bytes + b'\r\n')

        time.sleep(0.03)


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame')


# --- Start Flask in separate thread ---
def start_flask():
    app.run(host="0.0.0.0", port=5100, threaded=True)

def capture(cam):
    global shutdown
    global stop_capture
    global latest_frame
    running = True
    while running == True:
        frame = cam.take_picture()
        #with frame_lock:
        latest_frame = frame 
        # logger.debug("capture - Captured new frame")
        #with thread_lock:
        if shutdown == True:
            running = False
        if stop_capture == True:
            running = False
    # Stop the camera
    cam.terminate()

def process(rob:BallBalancingRobot):
    global shutdown
    cam = rob.cam
    hz = 50
    global latest_frame, latest_image, x, y
    x_t, y_t = rob.cam_parameters["target"]
    running = True
    while running == True:
        loop_start = time.perf_counter()
        #with frame_lock:
        if latest_frame is None:
            continue 
        frame_copy = latest_frame.copy()
        # logger.debug("process - Copied latest frame")

        with image_lock:
            latest_image = frame_copy.copy()
            if rob.show_contours:
                x, y = cam.coordinate(frame_copy, latest_image)
            else:
                x, y = cam.coordinate(frame_copy)
            # logger.debug("process - Calculated coordinates: x=%s, y=%s", x, y)
            latest_image = cam.draw_position(latest_image, (x, y))
            # logger.debug("process - position drawn on image")
        if rob.mode == "auto":
            update_robot_pos(rob, x_t, y_t, x, y)
            # logger.debug("process - Updated robot position")
        #with thread_lock:
        if shutdown == True:
            running = False
        elapsed = time.perf_counter() - loop_start
        sleep_time = (1 / hz) - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

def update_robot_pos(rob:BallBalancingRobot, x_t, y_t, x, y): #x_t, y_t: target position, x, y: current position, t: duration 

    robotcontroller = rob.controller
    robotkinematics = rob.robot
    pidcontroller = rob.pid

    rob.theta, rob.phi, _ = pidcontroller.pid((x_t, y_t), (x, y))
    rob.h = rob.configuration["h_work"]
    #print(rob.theta, rob.phi)
    #robotcontroller.Goto_time_spherical(rob.theta, rob.phi, rob.h, 0.02)
    robotcontroller.Goto_N_time_spherical(rob.theta, rob.phi, rob.h)

def simulate_pid(rob:BallBalancingRobot, response:dict) -> dict:
    """Return simulation result for current ball position

    Returns:
        dict: simulation result containing current ball position and resulting robot orientation
    """
    global latest_frame
    with frame_lock:
        if latest_frame is None:
            response["status"] = "error"
            response["message"] = "No frame available"
            return response
        frame_copy = latest_frame.copy()
    x_t, y_t = rob.cam_parameters["target"]
    x, y = rob.cam.coordinate(frame_copy)
    theta, phi, data = rob.pid.pid((x_t, y_t), (x, y))
    response["status"] = "success"
    response["message"] = ""
    response["pid_simulation"] = data
    return response

if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)    

    logger.debug("Starting ball balancing robot server")
    #
    # Instantiate the robot
    #
    bb_robot = BallBalancingRobot()

    #Initialize Ball Position
    x, y = bb_robot.cam_parameters["target"]

    # Start threads
    threading.Thread(target=capture, args=(bb_robot.cam,), daemon=True).start()
    threading.Thread(target=process, args=(bb_robot,), daemon=True).start()
    threading.Thread(target=start_flask, daemon=True).start()
    time.sleep(2)

    #
    # Start MQTT client loop
    #
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("localhost", 1883)
    client.loop_forever()