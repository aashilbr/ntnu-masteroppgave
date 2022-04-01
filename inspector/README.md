# Inspector

This project is mainly about finding good positions for inspection points of interest. We don't really care about carrying out inspections, and we don't really need a robot for calculating positions.
However, for debugging purposes, we need the Inspector module with the inspector robot.

The inspector module is responsible for spawning an inspector robot, and making the robot carry out inspections. Put simply, the module takes a list of POIs and a list of corresponding inspection poses. Then, the inspector robot takes a photo of each POI, from the corresponding inspection pose.