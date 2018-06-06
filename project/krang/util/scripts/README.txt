This folder contains useful scripts we wrote. The subfolder initd has the scripts that needs
to be "symbolicly linked" to /etc/init.d folder. The point of symbolic link is so that we can
follow the changes and commit them to this git repo. 

Here is an explanation of the files in this folder:

1- getAchs.sh: Allows you to create multiple panels in tmux which pull different channel information
               from krang.
2- putAchs.sh: The same as getachs, but pushes command channels to krang so that you can run
               controllers on other computers
3- achs:       A much cleaner combination of the above two. See the file itself for usages.

Here is for the initd folder:

1- krang: The init script that creates the ach channels and starts the daemons.
2- getNetworkFromMain: The init script for the vision computer that configures its ports.
3- shareNetworkWithVision: That changes the iptables of Krang to route vision pc network.
