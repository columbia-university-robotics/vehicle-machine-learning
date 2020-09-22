#!/bin/sh

#if [ $# -ne 1 ]; then
#
#    echo usage: launch_sim \<PATH_TO_SIM\>
#    exit 1
#
#fi

sim_path=~/Workspace/srcp2-competitors/docker/scripts/launch/roslaunch_docker

echo running simulation $sim_path
$sim_path --run-round 1
