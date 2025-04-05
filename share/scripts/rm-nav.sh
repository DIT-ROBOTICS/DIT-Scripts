#!/bin/bash

# Run a new session
tmux new-session -d -s eurobot-kill-script

# Remove navigation
tmux send-keys -t 0 "echo 'ditrobotics' | sudo -S docker compose -p navigation-run -f /home/navigation/Eurobot-2025-machine-ws/src/Eurobot-2025-Navigation2-envs/Navigation2-humble-deploy/docker-compose.yaml down" C-m

# Attach to the session
tmux attach-session -d

# Delete the tmux session
tmux kill-session -t eurobot-kill-script
# tmux kill-server
