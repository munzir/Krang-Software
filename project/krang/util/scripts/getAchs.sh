#!/bin/bash

KRANG_ADDRESS=krang.local

# check args
echo $#
if [ $# -eq 1 ]
then
    KRANG_ADDRESS=$1
fi

# Create the ach channels
ach -C llwa_ft -o 666 -1
ach -C rlwa_ft -o 666 -1
ach -C imu-data -o 666 -1
ach -C llwa-state -o 666 -1
ach -C rlwa-state -o 666 -1
ach -C waist-state -o 666 -1
ach -C joystick-data -o 666 -1
ach -C torso-state -o 666 -1

# Create tmux tabs and pull the data from the krang
tmux split-window -t 0 -h
tmux send-keys -t 1 "achd -r -v pull $KRANG_ADDRESS waist-state" C-m
tmux split-window -t 0
tmux send-keys -t 2 "achd -r -v pull $KRANG_ADDRESS imu-data" C-m
tmux split-window -t 1
tmux send-keys -t 3 "achd -r -v pull $KRANG_ADDRESS rlwa-state" C-m
tmux split-window -t 0
tmux send-keys -t 4 "achd -r -v pull $KRANG_ADDRESS llwa-state" C-m
tmux split-window -t 1
tmux send-keys -t 5 "achd -r -v pull $KRANG_ADDRESS rlwa_ft" C-m
tmux split-window -t 2
tmux send-keys -t 6 "achd -r -v pull $KRANG_ADDRESS llwa_ft" C-m
tmux split-window -t 3
tmux send-keys -t 7 "achd -r -v pull $KRANG_ADDRESS joystick-data" C-m
tmux split-window -t 4
tmux send-keys -t 8 "achd -r -v pull $KRANG_ADDRESS torso-state" C-m

# switch back to the original pane and wait for the user to hit enter
tmux select-pane -t 0
sleep 1
echo "Press enter to clean up."
read foo

# kill all the tmux windows we created
tmux kill-pane -a -t 0

# clean up the ach channels we left behind
ach rm llwa_ft
ach rm rlwa_ft
ach rm imu-data
ach rm llwa-state
ach rm rlwa-state
ach rm waist-state
ach rm torso-state
