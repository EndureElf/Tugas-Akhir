#! /bin/bash
# close service
systemctl stop yahboom_oled.service


len1=` ps -ef|grep oled.py |grep -v grep| wc -l`
echo "Number of processes="$len1

if [ $len1 -eq 0 ] 
then
    echo "oled.py is not running "
else
    # ps -ef| grep oled.py| grep -v grep| awk '{print $2}'| xargs kill -9  
    pid_number=` ps -ef| grep oled.py| grep -v grep| awk '{print $2}'`
    kill -9 $pid_number
    echo "oled.py killed, PID:"
    echo $pid_number
    
fi
# Clear OLED
python3 /home/sunrise/sunriseRobot/app_SunriseRobot/oled.py clear
sleep .1
