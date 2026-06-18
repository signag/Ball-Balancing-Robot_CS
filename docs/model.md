# BB Robot Modified Parts

## Printed Parts

STL files used for the robot print can be found in folder ```model```.

Some of the parts had to be slightly modified, compared to the [original ones](https://makerworld.com/en/models/1197770-ball-balancing-robot?from=recommend#profileId-1210633): 

- *Bottom.stl* has been modified to allow the Raspberry Pi and the PCA9685 to be screwed down.<br>Furthermore, an attachment has been integrated which can hold a standard Female 5.5 mm x 2.1 mm Hollow Connector for servo power supply.<br>*Barrel_Jack_Cover.stl* is required to fix the connector.
- *Camera_Mount.stl* has bee modified for a deeper camera position which allows a field of view covering the entire top ring when a [Raspberry Pi Camera Module 3 Wide](https://www.raspberrypi.com/documentation/accessories/camera.html#camera-module-3) camera is used.
- *Link1.stl* has been modified to allow for M5x10x7 Threaded Inserts holding the SA5 Ball Joint.

For convenience, some additional parts have been printed:

- *Ball.stl* is for a printed ball of suitable size.<br>It is not ideal, as slight deviations from perfect geometry and homogeneity may let it behave like a chaotic pendulum. But this can be additional challenge for the algorithm.
- *Border.stl* is for a small border which can be put on top of the top panel to prevent the ball from escaping during training.
- *Link1_Stand.stl* is a stand which can be used to hold the upper arm exactly vertical when pressing in the threaded insert.
- *top_panel_pattern.stl* is for a thin pattern which can be used to cut out and drill the top panel from a transparent plastic plate purchased in a hardware store.

## Non-printed Parts

Folder ```model``` contains an Excel book listing required parts ([bb_robot_parts_list.pdf](./bb_robot_parts_list.pdf)) as well as a shopping list ([bb_robot_shopping_list.pdf](./bb_robot_shopping_list.pdf)).

Note that there are some discrepancies to the [original parts list](https://github.com/I-M-Robotics-Lab/Ball-Balancing-Robot#build-instruction).