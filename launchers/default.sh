#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec echo "This runs the localization launch script."
dt-exec rosrun localization vrpn_to_abstract_state.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
