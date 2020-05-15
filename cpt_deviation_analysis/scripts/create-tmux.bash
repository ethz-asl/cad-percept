#!/bin/bash
pid=$PPID
echo $pid
tmux new-session -d -s selectiveicp #"reptyr -L $pid"

