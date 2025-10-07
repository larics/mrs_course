#!/bin/bash

CONTAINER_NAME=${1:-mrs_project_1}

ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock
docker start $CONTAINER_NAME
docker attach $CONTAINER_NAME
