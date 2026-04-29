import os
import json
from pathlib import Path
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import paho.mqtt.client as mqtt
from flask_cors import CORS
import logging
from flask.logging import default_handler

app = Flask(__name__)
socketio = SocketIO(app)
CORS(app)

# Configure loggers
logsPath = os.path.dirname(app.instance_path) + "/logs"
os.makedirs(logsPath, exist_ok=True)
logFile = logsPath + "/bb_robot.log"
Path(logFile).touch(exist_ok=True)
filehandler = logging.FileHandler(logFile)
filehandler.setFormatter(app.logger.handlers[0].formatter)
logger = app.logger
logger.setLevel(logging.DEBUG)
logger.addHandler(filehandler)

#
# Set up messaging client
#
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

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
    client.subscribe("robot/response")
    logger.debug("Subscribed to topic: robot/response")

def on_message(client, userdata, msg):
    """Process MQTT message from robot

    Args:
        client (mqtt.Client): The MQTT client instance.
        userdata (any): The private user data as set in Client() or userdata_set().
        msg (mqtt.MQTTMessage): An instance of MQTTMessage, which contains topic, payload, qos, retain.
    """
    data = json.loads(msg.payload.decode())
    logger.debug("Received MQTT message: %s", data)
    status = data.get("status")
    received = data.get("received")
    message = data.get("message")
    if "params" in data:
        logger.debug("Received robot params: %s", data["params"])
        params = data.get("params", {})
        update_robot_params(params)
    if "state" in data:
        logger.debug("Received robot state: %s", data["state"])
        state = data.get("state", {})
        update_robot_state(state)

client.on_connect = on_connect
client.on_message = on_message
client.connect("localhost", 1883)
client.loop_start()

def update_robot_state(state):
    """Update the robot state based on incoming data and return the new state.

    Args:
        state (dict): A dictionary containing the new state parameters for the robot.
    """
    logger.debug("update_robot_state - state: %s", state)
    socketio.emit("state_update", state)

def update_robot_params(params):
    """Update the robot parameters based on incoming data and return the new parameters.

    Args:
        params (dict): A dictionary containing the static parameters for the robot.
    """
    logger.debug("update_robot_params - params: %s", params)
    socketio.emit("params_update", params)

@app.route("/")
def index():
    return render_template("index.html")

@socketio.on("request_initial_data")
def handle_initial_request():
    request = {
        "method": "get_data"
    }
    client.publish("robot/request", json.dumps(request))
    logger.debug("Published initial data request to MQTT: %s", request)

@socketio.on("update_robot")
def update_robot(data):
    logger.debug("Start update_robot with data: %s", data)
    request = {
        "method" : "update",
        "params" : {
            "theta"   : data.get("theta"),
            "phi"     : data.get("phi"),
            "h"       : data.get("h")
        }
    }
    client.publish("robot/request", json.dumps(request))
    logger.debug("Published update request to MQTT: %s", request)

@socketio.on("reset_robot")
def handle_reset_request():
    request = {
        "method": "reset"
    }
    client.publish("robot/request", json.dumps(request))
    logger.debug("Published reset request to MQTT: %s", request)

if __name__ == "__main__":
    logger.debug("Starting ball balancing robot client")
    app.run(host="0.0.0.0", port=5000, debug=False)
