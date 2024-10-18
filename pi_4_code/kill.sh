# first run chmod +x kill.sh in command line
# run ./kill.sh in command line to stop motor running
sudo bash -c "echo PWM 0 0 0 > /dev/ttyACM0" && sudo bash -c "echo PWM 1 1 0 > /dev/ttyACM0"
