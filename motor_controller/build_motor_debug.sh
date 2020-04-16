arduino-cli compile --fqbn arduino:avr:mega motor_debug


OUT=$?
if [ $OUT -eq 0 ];then
    arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega motor_debug -v
else
   echo "****ERROR*****"
fi


