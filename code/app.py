import os
from pathlib import Path
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import math
from robotKinematics import RobotKinematics
from controller import RobotController
import logging
from flask.logging import default_handler

app = Flask(__name__)
socketio = SocketIO(app)
CORS(app)


# Configure loggers
logsPath = os.path.dirname(app.instance_path) + "/logs"
os.makedirs(logsPath, exist_ok=True)
logFile = logsPath + "/app.log"
Path(logFile).touch(exist_ok=True)
filehandler = logging.FileHandler(logFile)
filehandler.setFormatter(app.logger.handlers[0].formatter)
for logger in (
    app.logger,
    logging.getLogger("code.robotKinematics"),
    logging.getLogger("code.controller")
):
    logger.setLevel(logging.ERROR)

# >>>>> Uncomment the following line in order to log to the log file
app.logger.addHandler(filehandler)

# >>>>> Explicitely set specific log levels.
app.logger.setLevel(logging.DEBUG)
logging.getLogger("code.robotKinematics").setLevel(logging.DEBUG)
logging.getLogger("code.controller").setLevel(logging.DEBUG)

#
# Initialize robot
#
app.logger.debug("Application started")
robot = RobotKinematics(lp=7.14, l1=7.50, l2=4.50, lb=5.56, invert=False)
rc = RobotController(robot)
try:
    robot.solve_inverse_kinematics_vector(robot.alpha, robot.beta, robot.gamma, robot.h)
    rc.set_motor_angles(math.degrees(math.pi*0.5 - robot.theta1), math.degrees(math.pi*0.5 - robot.theta2), math.degrees(math.pi*0.5 - robot.theta3))       
except Exception as e:
    app.logger.error("Error during robot initialization: %s", e)

@app.route("/")
def index():
    global robot
    return render_template("index.html", rob=robot)

@socketio.on("update_robot")
def update_robot(data):
    app.logger.debug("Start update_robot with data: %s", data)
    robot.phi   = float(data["phi"])
    robot.h     = float(data["h"])
    robot.theta = float(data["theta"])

    theta_rad = math.radians(robot.theta)
    phi_rad   = math.radians(robot.phi)
    robot.alpha = math.sin(theta_rad) * math.cos(phi_rad)
    robot.beta  = math.sin(theta_rad) * math.sin(phi_rad)
    robot.gamma = math.cos(theta_rad)

    try:
        robot.solve_inverse_kinematics_vector(robot.alpha, robot.beta, robot.gamma, robot.h)
        rc.set_motor_angles(math.degrees(math.pi*0.5 - robot.theta1), math.degrees(math.pi*0.5 - robot.theta2), math.degrees(math.pi*0.5 - robot.theta3))       
    except Exception as e:
        app.logger.error("Error during robot update: %s", e)

    result = {
        "theta": int(100 * robot.theta) / 100,
        "phi": int(100 * robot.phi) / 100,
        "h": int(100 * robot.h) / 100,
        "alpha": int(100 * math.degrees(robot.alpha)) / 100,
        "beta": int(100 * math.degrees(robot.beta)) / 100,
        "gamma": int(100 * math.degrees(robot.gamma)) / 100,
        "theta1": int(100 * math.degrees(robot.theta1)) / 100,
        "theta2": int(100 * math.degrees(robot.theta2)) / 100,
        "theta3": int(100 * math.degrees(robot.theta3)) / 100,
        "theta_max": int(100 * robot.theta_max) / 100
    }
    app.logger.debug("End update_robot with computed angles: %s", result)
    socketio.emit("client_update", result)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, debug=True)
