#!/bin/bash

IMAGE_NAME=${1:-mrs_2d_sim:humble}
CONTAINER_NAME=${2:-mrs_project_1}


# Hook to the current SSH_AUTH_LOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

docker run \
  -it \
  --network host \
  --privileged \
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  --volume /tmp/.x11-unix:/tmp/.x11-unix \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name $CONTAINER_NAME \
  $IMAGE_NAME