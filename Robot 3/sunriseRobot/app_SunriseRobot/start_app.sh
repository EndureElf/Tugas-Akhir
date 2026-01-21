#! /bin/bash

###############################################################################
# ubuntu 22.04 
# add Additional startup programs
# start_app
# bash /home/sunrise/sunriseRobot/app_SunriseRobot/start_app.sh
# start app program
###############################################################################


sleep 15

xfce4-terminal -e "bash -c 'sudo python3 /home/sunrise/sunriseRobot/app_SunriseRobot/app_SunriseRobot.py; exec bash'"

wait
exit 0
