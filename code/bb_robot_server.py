import paho.mqtt.client as mqtt
import signal
import json
import os
from pathlib import Path
import math
from robotKinematics import RobotKinematics
from controller import RobotController
import logging
#
# Robot construction parameters
#
LP = 7.14    #Radius of Top
L1 = 7.50    #Top Arm
L2 = 4.50    #Bottom Arm
LB = 5.56    #Radius of Bottom
INVERT = False    #Whether the arm stays inward or outward
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

for logger in (
    logging.getLogger("bb_robot_server"),
    logging.getLogger("robotKinematics"),
    logging.getLogger("controller")
):
    logger.setLevel(logging.ERROR)
    logger.addHandler(filehandler)
    logger.addHandler(streamhandler)

# >>>>> Explicitely set specific log levels.
logging.getLogger("bb_robot_server").setLevel(logging.DEBUG)
logging.getLogger("robotKinematics").setLevel(logging.DEBUG)
logging.getLogger("controller").setLevel(logging.DEBUG)

logger = logging.getLogger("bb_robot_server")

class BallBalancingRobot:
    def __init__(self):
        """Initialize the ball balancing robot with specified parameters and set it to a default state.
        """
        logger.debug("Initializing BallBalancingRobot")
        self.robot = RobotKinematics(lp=LP, l1=L1, l2=L2, lb=LB, invert=INVERT)
        self.controller = RobotController(self.robot)
        try:
            self.robot.solve_inverse_kinematics_vector(self.robot.alpha, self.robot.beta, self.robot.gamma, self.robot.h)
            self.controller.set_motor_angles(
                math.degrees(math.pi*0.5 - self.robot.theta1), 
                math.degrees(math.pi*0.5 - self.robot.theta2), 
                math.degrees(math.pi*0.5 - self.robot.theta3)
            )       
        except Exception as e:
            logger.error("Error during robot initialization: %s", e)

    def reset(self, response):
        """Reset the robot to its default state.
        """
        logger.debug("Resetting BallBalancingRobot to default state")
        self.robot = RobotKinematics(lp=LP, l1=L1, l2=L2, lb=LB, invert=INVERT)
        self.controller = RobotController(self.robot)
        try:
            self.robot.solve_inverse_kinematics_vector(self.robot.alpha, self.robot.beta, self.robot.gamma, self.robot.h)
            self.controller.set_motor_angles(
                math.degrees(math.pi*0.5 - self.robot.theta1), 
                math.degrees(math.pi*0.5 - self.robot.theta2), 
                math.degrees(math.pi*0.5 - self.robot.theta3)
            )       
        except Exception as e:
            logger.error("Error during robot initialization: %s", e)
            response["status"] = "error"
            response["message"] = str(e)
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

        theta = params.get("theta")
        phi = params.get("phi")
        h = params.get("h")
        logger.debug("Processing update with theta: %s, phi: %s, h: %s", theta, phi, h)
        self.robot.theta = float(theta)
        self.robot.phi = float(phi)
        self.robot.h = float(h)

        theta_rad = math.radians(self.robot.theta)
        phi_rad   = math.radians(self.robot.phi)
        self.robot.alpha = math.sin(theta_rad) * math.cos(phi_rad)
        self.robot.beta  = math.sin(theta_rad) * math.sin(phi_rad)
        self.robot.gamma = math.cos(theta_rad)
        try:
            self.robot.solve_inverse_kinematics_vector(self.robot.alpha, self.robot.beta, self.robot.gamma, self.robot.h)
            self.controller.set_motor_angles(
                math.degrees(math.pi*0.5 - self.robot.theta1), 
                math.degrees(math.pi*0.5 - self.robot.theta2), 
                math.degrees(math.pi*0.5 - self.robot.theta3)
            )       
            logger.debug("Update processed successfully")
        except Exception as e:
            logger.error("Error during update processing: %s", e)
            response["status"] = "error"
            response["message"] = str(e)
        response["status"] = "success"
        response["message"] = ""
        response["state"] = self.state
        return response

    @property
    def state(self) -> dict:
        """Return the current state of the robot, including angles, height, and motor positions.
        """
        result = {
            "theta": self.robot.theta,
            "phi": self.robot.phi,
            "h": self.robot.h,
            "alpha": math.degrees(self.robot.alpha),
            "beta": math.degrees(self.robot.beta),
            "gamma": math.degrees(self.robot.gamma),
            "theta1": math.degrees(self.robot.theta1),
            "theta2": math.degrees(self.robot.theta2),
            "theta3": math.degrees(self.robot.theta3),
            "theta_max": self.robot.theta_max
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
            "theta_ubound": self.robot.theta_ubound
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
        response["message"] = ""
    if method == "get_data":
        response["status"] = "success"
        response["params"] = bb_robot.params
        response["state"] = bb_robot.state
        response["message"] = ""
    if method == "update":
        response = bb_robot.update(params, response)
    if method == "reset":
        response = bb_robot.reset(response)
    
    client.publish("robot/response", json.dumps(response))
    logger.debug("Sent response: %s", response)

def shutdown(sig, frame):
    logger.debug("Shutting down...")
    client.disconnect()   # 👈 stops loop_forever
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)    

logger.debug("Starting ball balancing robot server")
#
# Instantiate the robot
#
bb_robot = BallBalancingRobot()
#
# Start MQTT client loop
#
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1883)
client.loop_forever()