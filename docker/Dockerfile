//Dockerfile for my repo

FROM duckietown/rpi-duckiebot-base:master19

RUN ["cross-build-start"]

#### START CUSTOM CATKIN_WS ####
RUN /bin/bash -c "mkdir -p robot_ws/src/"

# Copy or init your packages in here
COPY open_loop_control robot/src/open_loop_control
RUN chmod +x catkin_ws/src/open_loop_control/src/open_loop.py

RUN /bin/bash -c "cd robot_ws/src/"

# Do not change the below line! This ensures that your workspace is
# overlayed on top of the Duckietown stack!
# MAKE sure this line is present in the build:
# This workspace overlays: /home/software/catkin_ws/devel;/opt/ros/kinetic
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && catkin_init_workspace && cd ../.."
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && catkin_make -j -C robot_ws/"
RUN /bin/bash -c "source /home/software/robot_ws/devel/setup.bash"

#### END CUSTOM CATKIN_WS ####

RUN ["cross-build-end"]
