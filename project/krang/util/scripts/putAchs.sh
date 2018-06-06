#!/bin/bash

KRANG_ADDRESS=krang.local

# check args
echo $#
if [ $# -eq 1 ]
then
    KRANG_ADDRESS=$1
fi

# create ach channels
ach -C llwa-cmd -o 666 -1
ach -C rlwa-cmd -o 666 -1
ach -C waist-cmd -o 666 -1
ach -C lgripper-cmd -o 666 -1
ach -C rgripper-cmd -o 666 -1
ach -C torso-cmd -o 666 -1
ach -C waistd-cmd -o 666 -1

# create a bunch of tmux tabs and use them to push to krang
tmux split-window -t 0 -h
tmux send-keys -t 1 "achd -r -v push $KRANG_ADDRESS waist-cmd" C-m
tmux split-window -t 0
tmux send-keys -t 2 "achd -r -v push $KRANG_ADDRESS rlwa-cmd" C-m
tmux split-window -t 1
tmux send-keys -t 3 "achd -r -v push $KRANG_ADDRESS llwa-cmd" C-m
tmux split-window -t 0
tmux send-keys -t 4 "achd -r -v push $KRANG_ADDRESS rgripper-cmd" C-m
tmux split-window -t 1
tmux send-keys -t 5 "achd -r -v push $KRANG_ADDRESS lgripper-cmd" C-m
tmux split-window -t 2
tmux send-keys -t 6 "achd -r -v push $KRANG_ADDRESS torso-cmd" C-m
tmux split-window -t 3
tmux send-keys -t 7 "achd -r -v push $KRANG_ADDRESS waistd-cmd" C-m


# switch back to the original pane and wait for the user to hit enter
tmux select-pane -t 0
sleep 1
echo "Press enter to clean up."
read foo

# kill all the tmux windows we created
tmux kill-pane -a -t 0

# clean up the ach channels we left behind
ach rm llwa-cmd
ach rm rlwa-cmd
ach rm waist-cmd
ach rm lgripper-cmd
ach rm rgripper-cmd
ach rm torso-cmd
ach rm waistd-cmd
