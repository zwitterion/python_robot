~/arduino-cli/bin/arduino-cli compile --fqbn arduino:avr:mega motor_controller


OUT=$?
if [ $OUT -eq 0 ];then
    ~/arduino-cli/bin/arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:avr:mega motor_controller -v
else
   echo "****ERROR*****"
fi


