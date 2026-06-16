# Installation of Ball-Balancing-Robot_CS

This document assumes a headless setup of the Raspberry Pi with WiFi activated.

1. Connect to the Raspberry Pi using SSH:<br>```ssh <user>@<host>```<br>with <user> and <host> as specified during setup with Imager.
2. Update the system<br>```sudo apt update```<br>```sudo apt full-upgrade```
3. Create a root directory under which you will install programs (e.g. 'prg')<br>```mkdir prg```<br>```cd prg```
4. Clone the Ball-Balancing-Robot_CS repository:<br>```git clone --branch main --single-branch --depth 1 https://github.com/signag/Ball-Balancing-Robot_CS```
5. Create a virtual environment ('.venv') on the 'Ball-Balancing-Robot_CS' folder:<br>```cd Ball-Balancing-Robot_CS```<br>```python -m venv --system-site-packages .venv```<br>For the reasoning to include system site packages, see the [picamera2-manual.pdf](./picamera2-manual.pdf), chapter 9.5.
6. Activate the virtual environment<br>```cd ~/prg/Ball-Balancing-Robot_CS```<br>```source .venv/bin/activate```<br>The active virtual environment is indicated by ```(.venv)``` preceeding the system prompt.<br>(If you need to leave the virtual environment at some time, use ```deactivate```)
7. Install required packages<br>```python -m pip install --upgrade pip```<br>```python -m pip install paho-mqtt```<br>```python -m pip install adafruit-blinka```<br>```python -m pip install adafruit-circuitpython-pca9685```<br>```python -m pip install adafruit-circuitpython-servokit```<br>```python -m pip install flask-socketio```<br>```python -m pip install flask-cors```
8. Adjust service configuration<br>```cp ~/prg/Ball-Balancing-Robot_CS/config/*.service ~```<br>```nano ~/bb_robot_server.service```<br>Replace all 3 occurrences of ```<your_username>``` with your username, save and exit<br>```nano ~/bb_robot_client.service```<br>Replace all 3 occurrences of ```<your_username>``` with your username, save and exit
9. Stage service files<br>```sudo mv ~/bb_robot_*.service /etc/systemd/system```
10. Start Services<br>```sudo systemctl start bb_robot_server.service```<br>```sudo systemctl start bb_robot_client.service```
11. Verify successful service start<br>```sudo journalctl -ef```
12. Enable services for automatic start at system startup<br>```sudo systemctl enable bb_robot_server.service```<br>```sudo systemctl enable bb_robot_client.service```
13. Reboot system<br>```sudo reboot```

Successful server start is indicated by the initialization move of the robot.
