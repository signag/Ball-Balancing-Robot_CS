import paho.mqtt.client as mqtt
import signal
import threading
import time
import datetime
import cv2
import json
import os
from pathlib import Path
import math
from robotKinematics import RobotKinematics
from controller import RobotController
from camera import Camera
from PID import PIDcontroller
from flask import Flask, Response
import csv
import logging
#
# Robot configuration
#
ROBOT_CONFIG_FILE = "bb_robot_config.json"
#
# Configure logging
#
logsPath = os.getcwd() + "/logs"
os.makedirs(logsPath, exist_ok=True)
logFile = logsPath + "/bb_robot.log"
Path(logFile).touch(exist_ok=True)
filehandler = logging.FileHandler(logFile)
filehandler.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s in %(name)s: %(message)s'))
streamhandler = logging.StreamHandler()
streamhandler.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s in %(name)s: %(message)s'))

#Initialize Ball Position
x, y = 100, 75

for logger in (
    logging.getLogger("bb_robot_server"),
    logging.getLogger("robotKinematics"),
    logging.getLogger("controller"),
    logging.getLogger("camera"),
    logging.getLogger("PIDcontroller"),
):
    logger.setLevel(logging.ERROR)
    logger.addHandler(filehandler)
    # logger.addHandler(streamhandler)

# >>>>> Explicitely set specific log levels.
# logging.getLogger("bb_robot_server").setLevel(logging.DEBUG)
# logging.getLogger("robotKinematics").setLevel(logging.DEBUG)
# logging.getLogger("controller").setLevel(logging.DEBUG)
# logging.getLogger("camera").setLevel(logging.DEBUG)

logger = logging.getLogger("bb_robot_server")

app = Flask(__name__)

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
        self.configuration = {
            "LP": 7.125,
            "L1": 6.20,
            "L2": 4.50,
            "LB": 4.00,
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
            "ki": 0.00005,
            "kd": 0.006025,
            "alpha": 0.65,
            "beta": 0.3
        }
        self.cam_parameters = {
            "resolution": (1640, 1232),
            "resolution_work": (200, 150),
            "center": (100, 75),
            "detection_radius": 70, 
            "format": "RGB888"
        }
        
        self.read_config()

        self.robot = RobotKinematics(lp=self.configuration["LP"], l1=self.configuration["L1"], l2=self.configuration["L2"], lb=self.configuration["LB"], invert=self.configuration["INVERT"])
        self.controller = RobotController(self.robot, calibration=self.calibration)
        self.cam = Camera(self.cam_parameters)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fn = Path(__file__).parent / f"pid_data_{ts}.csv"
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
        fp = Path(__file__).parent / ROBOT_CONFIG_FILE
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
                        "detection_radius": cam_parameters.get("detection_radius", self.cam_parameters["detection_radius"]),
                        "format": cam_parameters.get("format", self.cam_parameters["format"])
                    })
                    logger.debug("Loaded camera parameters: %s", self.cam_parameters)

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

        #Initialize Ball Position
        x, y = bb_robot.cam_parameters["center"]

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

    def save_calibration(self, response) -> dict:
        """Save the current calibration parameters to the configuration file.

        Args:
            response (dict): A dictionary to be updated with the save result.

        Returns:
            dict: The updated response dictionary with save status and message.
        """
        logger.debug("Saving calibration parameters to file")
        config_path = Path(__file__).parent / ROBOT_CONFIG_FILE
        try:
            if config_path.is_file():
                with open(config_path, "r") as f:
                    robot_config = json.load(f)
            else:
                robot_config = {}

            robot_config["calibration"] = self.calibration

            with open(config_path, "w") as f:
                json.dump(robot_config, f, indent=4)

            logger.debug("Calibration parameters saved successfully")
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
        config_path = Path(__file__).parent / ROBOT_CONFIG_FILE
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
            fn = Path(__file__).parent / f"pid_data_{ts}.csv"
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
            "theta3_offset": self.calibration["theta3_offset"]
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
        "status": "invalid",
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
        with frame_lock:
            latest_frame = frame 
        with thread_lock:
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
    running = True
    while running == True:
        with frame_lock:
            if latest_frame is None:
                continue 
            frame_copy = latest_frame.copy()
        
        loop_start = time.perf_counter()
        x, y = cam.coordinate(frame_copy)  
        x_t, y_t = rob.cam_parameters["center"]
        with image_lock:
            latest_image = cam.draw_position(frame_copy, (x_t, y_t), (x, y))
        if rob.mode == "auto":
            update_robot_pos(rob, x_t, y_t, x, y)
        with thread_lock:
            if shutdown == True:
                running = False
        elapsed = time.perf_counter() - loop_start
        sleep_time = (1 / hz) - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

def simulate_pid(rob:BallBalancingRobot, response:dict) -> dict:
    """Return simulation result for current ball position

    Returns:
        dict: simulation result containing current ball position and resulting robot orientation
    """
    global latest_frame
    with frame_lock:
        if latest_frame is None:
            response["status"] = "error: no frame available"
            return response
        frame_copy = latest_frame.copy()
    x_t, y_t = rob.cam_parameters["center"]
    x, y = rob.cam.coordinate(frame_copy)
    theta, phi, data = rob.pid.pid((x_t, y_t), (x, y))
    response["status"] = "success"
    response["message"] = ""
    response["pid_simulation"] = data
    return response


def update_robot_pos(rob:BallBalancingRobot, x_t, y_t, x, y): #x_t, y_t: target position, x, y: current position, t: duration 

    robotcontroller = rob.controller
    robotkinematics = rob.robot
    pidcontroller = rob.pid

    rob.theta, rob.phi, _ = pidcontroller.pid((x_t, y_t), (x, y))
    rob.h = rob.configuration["h_work"]
    #print(rob.theta, rob.phi)
    #robotcontroller.Goto_time_spherical(rob.theta, rob.phi, rob.h, 0.02)
    robotcontroller.Goto_N_time_spherical(rob.theta, rob.phi, rob.h)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)    

    logger.debug("Starting ball balancing robot server")
    #
    # Instantiate the robot
    #
    bb_robot = BallBalancingRobot()

    #Initialize Ball Position
    x, y = bb_robot.cam_parameters["center"]

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