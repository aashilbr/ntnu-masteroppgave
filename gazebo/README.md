# Gazebo

We use Gazebo as our simulation tool. In Gazebo we spawn all our 3D models (in this project: a robot, the Huldra rig, and maybe some valve-like objects).

This module simply installs and runs Gazebo.

Note: To be able to spawn models in Gazebo correctly, the models might have to be stored inside the Gazebo container. Links inside model files will be relative to the container path.