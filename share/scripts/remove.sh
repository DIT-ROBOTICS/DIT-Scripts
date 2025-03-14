#!/bin/bash

# Run a new session
tmux new-session -d -s eurobot-kill-script

# Remove localization
tmux send-keys -t 0 "echo 'ditrobotics' | sudo -S docker compose -p communication -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/yellow-compose.yml down" C-m
tmux send-keys -t 0 "echo 'ditrobotics' | sudo -S docker compose -p communication -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/blue-compose.yml down" C-m
# Remove navigation
tmux send-keys -t 0 "echo 'ditrobotics' | sudo -S docker compose -p navigation-run -f /home/navigation/Eurobot-2025-machine-ws/src/Eurobot-2025-Navigation2-envs/Navigation2-humble-deploy/docker-compose.yaml down" C-m

# Attach to the session
tmux attach-session -d

# Delete the tmux session
#tmux kill-session -t eurobot-script
tmux kill-server
