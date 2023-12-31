#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec echo "This runs the localization and planning launch script."
dt-exec rosrun localization vrpn_to_abstract_state.py
sleep 2
dt-exec rosrun localization vrpn_to_angle.py 
sleep 2
dt-exec rosrun planning_simple hlp.py
sleep 2
dt-exec rosrun controls pure_pursuit_heading.py
sleep 2
dt-exec rosrun controls heading_tracker.py
# sleep 2
# dt-exec rosrun controls lane_pose_publish.py
# dt-exec rosrun planning_simple hlp.py
# sleep 2
# dt-exec rosrun low_level_simple wheel_controller.py
# dt-exec rosrun controls heading_tracker_wheel_speeds.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
