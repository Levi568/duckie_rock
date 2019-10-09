# duckie_rock hw2
Create a custom ROS message inside your package that contains:
 * A string labeled id
 * A point labeled position 
Modify the package.xml and CMakeLists.txt files to build the message
Create three ROS nodes:
 * Node 1: publishes points (as geometry_msg/Point type) with random x,y,z values on topic /obstacles_detected
 * Node 2: subscribes to /obstacles_detected, ingests those points, adds a unique identifier to them (your choice of format) and publishes this new dat
 * a structure as YOUR MESSAGE TYPE from question 4 on topic /registered_obstacles
 * Node 3: Subscribes to /registered_obstacles and logs an info-level message to rosout for each one in a human readable format such as:
 * “Detected obstacle <id> at position <x>,<y>,<z>”
