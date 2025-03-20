#!/bin/bash

# Run a new session
tmux new-session -d -s eurobot-kill-script

# Remove main
tmux send-keys -t 0 "echo 'ditrobotics' | sudo -S docker compose -p main-run -f /home/main/Eurobot-2025-Main-ws/docker/docker-compose-lin.yml down" C-m

# Attach to the session
tmux attach-session -d

# Delete the tmux session
tmux kill-session -t eurobot-kill-script
# tmux kill-server
