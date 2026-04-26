import os
from pathlib import Path
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
import math
from robotKinematics import RobotKinematics
from controller import RobotController
import logging
from flask.logging import default_handler

app = Flask(__name__)
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

app.logger.debug("Application started")
robot = RobotKinematics()
rc = RobotController(robot)
# Hard-coded parameters
robot.lp = 7.14
robot.l1 = 7.50
robot.l2 = 4.50
robot.lb = 5.56

def compute_maxh(l1, l2, lp, lb):
    return math.sqrt(((l1 + l2) ** 2) - ((lp - lb) ** 2))

def compute_minh(l1, l2, lp, lb):
    if l1 > l2:
        return math.sqrt((l1 ** 2) - ((lb + l2 - lp) ** 2))
    elif l2 > l1:
        return math.sqrt(((l2 - l1) ** 2) - ((lp - lb) ** 2))
    else:
        return 0

robot.maxh = compute_maxh(robot.l1, robot.l2, robot.lp, robot.lb) - 0.2
robot.minh = compute_minh(robot.l1, robot.l2, robot.lp, robot.lb) + 0.45

@app.route("/")
def index():
    global robot
    return render_template("index.html", rob=robot)

@app.route("/update", methods=["POST"])
def update_robot():
    app.logger.debug("Received update request with data: %s", request.get_json())
    data = request.get_json()
    slider_theta = float(data.get("slider_theta", 0))   # 0..2000 => 0..20
    slider_phi   = float(data.get("slider_phi", 0))     # 0..36000 => 0..360
    slider_h     = float(data.get("slider_h", 814))     # scaled by 100 => real h

    theta_deg = float(data["theta"])
    phi_deg   = float(data["phi"])
    h         = float(data["h"])

    robot.theta_o = theta_deg
    robot.phi_o   = phi_deg
    robot.h_o     = h

    alpha = 0.0
    beta  = 0.0
    gamma = 1.0
    max_theta_for_h = 10.0  # fallback

    try:
        theta_rad = math.radians(theta_deg)
        phi_rad   = math.radians(phi_deg)
        alpha = math.sin(theta_rad) * math.cos(phi_rad)
        beta  = math.sin(theta_rad) * math.sin(phi_rad)
        gamma = math.cos(theta_rad)

        robot.alpha = alpha
        robot.beta = beta
        robot.gamma = gamma

        max_theta_for_h = robot.max_theta(h)
        robot.solve_inverse_kinematics_vector(alpha, beta, gamma, h)
          # in degrees
        
        rc.set_motor_angles(math.degrees(math.pi*0.5 - robot.theta1), math.degrees(math.pi*0.5 - robot.theta2), math.degrees(math.pi*0.5 - robot.theta3))       
    except:
        pass
    return render_template("index.html", rob=robot)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, debug=True)
