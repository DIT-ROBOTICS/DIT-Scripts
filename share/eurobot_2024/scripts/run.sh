#!/bin/bash

# Run a new session
tmux new-session -d -s eurobot-script

# Split to 3 panes
tmux split-window -h
tmux split-window -v

# Echo
echo -e "Running the localization, navigation and main program..."

# Run localization
tmux send-keys -t 0 "echo 'ditrobotics' | sudo -S docker compose -f /home/localization/Eurobot-2024/src/Eurobot-2024-Localization-envs/compose-run.yaml run -T --rm localization" C-m

echo -e "Localization is running..."

sleep 2

# Run navigation
tmux send-keys -t 1 "echo 'ditrobotics' | sudo -S docker compose -f /home/navigation/Eurobot2024-machine-ws/src/Eurobot-2024-Navigation-Main/Eurobot-2024-Navigation-envs/deploy/docker_deploy.yaml run -T --rm navigation-run-11" C-m

echo -e "Navigation is running..."

sleep 3

echo -e "Main program is running..."

# Run main program
tmux send-keys -t 2 "echo 'ditrobotics' | sudo -S docker compose -f /home/main/Eurobot-Main/docker/docker-compose.yml run -T --rm ros1-main-program-run" C-m

# Attach to the session
tmux attach-session -d
