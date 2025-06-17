#!/bin/bash

# Function to send stop_scan service to containers
send_stop_scan_service() {
    echo "Sending /stop_scan service to running localization containers..."
    
    echo "ditrobotics" | sudo -S docker exec localization-2025-dev-ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 service call /stop_scan std_srvs/srv/Empty" 2>/dev/null || echo "Failed to send stop_scan"
    
    # Wait a moment for the service to process
    sleep 1
}

# Function to clean up all sessions and processes
cleanup_all() {
    echo "Cleaning up all sessions and processes..."
    
    # Send stop_scan service before stopping containers
    send_stop_scan_service
    
    # Kill existing sessions
    tmux kill-session -t eurobot-script-local-yellow 2>/dev/null
    tmux kill-session -t eurobot-script-local-blue 2>/dev/null
    tmux kill-session -t eurobot-script-nav 2>/dev/null
    tmux kill-session -t eurobot-script-main 2>/dev/null
    tmux kill-session -t $SESSION_NAME 2>/dev/null
    
    # Stop docker containers
    echo "Stopping docker containers..."
    # Stop localization containers
    echo "ditrobotics" | sudo -S docker compose -p localization-blue -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/blue-compose.yml down 2>/dev/null
    echo "ditrobotics" | sudo -S docker compose -p localization-yellow -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/yellow-compose.yml down 2>/dev/null
    
    # Stop navigation container
    echo "ditrobotics" | sudo -S docker compose -p navigation-run -f /home/navigation/Eurobot-2025-machine-ws/src/Eurobot-2025-Navigation2-envs/Navigation2-humble-deploy/docker-compose.yaml down 2>/dev/null
    
    # Stop any remaining containers related to our application
    BLUE_CONTAINERS=$(sudo docker ps -q --filter "name=localization-blue")
    YELLOW_CONTAINERS=$(sudo docker ps -q --filter "name=localization-yellow") 
    NAV_CONTAINERS=$(sudo docker ps -q --filter "name=navigation-run")
    
    if [ -n "$BLUE_CONTAINERS" ] || [ -n "$YELLOW_CONTAINERS" ] || [ -n "$NAV_CONTAINERS" ]; then
        echo "Stopping remaining containers..."
        [ -n "$BLUE_CONTAINERS" ] && echo "ditrobotics" | sudo -S docker stop $BLUE_CONTAINERS 2>/dev/null
        [ -n "$YELLOW_CONTAINERS" ] && echo "ditrobotics" | sudo -S docker stop $YELLOW_CONTAINERS 2>/dev/null
        [ -n "$NAV_CONTAINERS" ] && echo "ditrobotics" | sudo -S docker stop $NAV_CONTAINERS 2>/dev/null
    fi
    
    # Kill main process if running
    echo "Stopping main program gracefully..."
    echo "ditrobotics" | sudo -S -u main /home/main/Eurobot-2025-Main/main close 2>/dev/null
    sleep 1
    pkill -f "/home/main/Eurobot-2025-Main/main run" 2>/dev/null
    
    # Stop any playing music
    pkill -f "ffplay" 2>/dev/null
    
    echo "Cleanup complete."
}

# Check if already inside a tmux session
if [ -n "$TMUX" ]; then
    echo "Already inside a tmux session. Detaching first to prevent nesting issues..."
    TMUX_TEMP=$TMUX
    unset TMUX
    echo "TMUX variable temporarily unset."
fi

# Check if script is already running
SESSION_NAME="eurobot-master-session"
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "Session already exists, cleaning up..."
    cleanup_all
    
    # Restore TMUX variable if it was unset
    if [ -n "$TMUX_TEMP" ]; then
        export TMUX=$TMUX_TEMP
    fi
    exit 0
fi

# Create a new master session with mouse/touch support
tmux new-session -d -s $SESSION_NAME

# Enable mouse mode for touch scrolling
tmux set-option -t $SESSION_NAME mouse on
tmux set-option -t $SESSION_NAME history-limit 10000

# Split the window into three panes
# Top left, top right, bottom full width
tmux split-window -h -t $SESSION_NAME:0
tmux split-window -v -t $SESSION_NAME:0.0

# Check button.json for team selection
BUTTON_FILE="/home/share/data/button.json"
# If primary location doesn't exist, try alternative location
if [ ! -f "$BUTTON_FILE" ]; then
    BUTTON_FILE="/home/ditrobotics/DIT-Scripts/share/data/button.json"
    echo "Trying alternative button file location: $BUTTON_FILE"
fi
if [ ! -f "$BUTTON_FILE" ]; then
    echo "Button file not found at $BUTTON_FILE"
    tmux send-keys -t $SESSION_NAME:0.0 "echo 'Button file not found. Exiting.'" C-m
    tmux attach-session -t $SESSION_NAME
    exit 1
fi

# Read the button states
BUTTON_DATA=$(cat $BUTTON_FILE)
echo "Button data: $BUTTON_DATA"

# Function to check if a button is true
is_button_active() {
    local BUTTON_NUM=$1
    if echo "$BUTTON_DATA" | grep -q "\"$BUTTON_NUM\": true"; then
        return 0  # true
    else
        return 1  # false
    fi
}

# Determine which team to run based on buttons
TEAM=""
if is_button_active 10 || is_button_active 11 || is_button_active 13; then
    TEAM="yellow"
    echo "Yellow team selected"
elif is_button_active 15 || is_button_active 17 || is_button_active 19; then
    TEAM="blue"
    echo "Blue team selected"
else
    echo "No team buttons pressed, exiting"
    tmux send-keys -t $SESSION_NAME:0.0 "echo 'No team buttons (10, 11, 13 or 15, 17, 19) are active. Exiting.'" C-m
    tmux attach-session -t $SESSION_NAME
    exit 1
fi

# Start localization in the top-left pane
tmux send-keys -t $SESSION_NAME:0.0 "echo 'Starting $TEAM team localization...'" C-m
if [ "$TEAM" = "blue" ]; then
    tmux send-keys -t $SESSION_NAME:0.0 "echo 'ditrobotics' | sudo -S docker compose -p localization-blue -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/blue-compose.yml up" C-m
else
    tmux send-keys -t $SESSION_NAME:0.0 "echo 'ditrobotics' | sudo -S docker compose -p localization-yellow -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/yellow-compose.yml up" C-m
fi

# Wait 5 seconds for localization to initialize
sleep 5

# Start navigation in the top-right pane
tmux send-keys -t $SESSION_NAME:0.1 "echo 'Starting navigation...'" C-m
tmux send-keys -t $SESSION_NAME:0.1 "echo 'ditrobotics' | sudo -S docker compose -p navigation-run -f /home/navigation/Eurobot-2025-machine-ws/src/Eurobot-2025-Navigation2-envs/Navigation2-humble-deploy/docker-compose.yaml up navigation-run" C-m

# Wait 5 seconds for navigation to initialize
sleep 5

# Start main program in the bottom pane
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Starting main program...'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "echo 'ditrobotics' | sudo -S -u main /home/main/Eurobot-2025-Main/main run" C-m

# Find and play a random MP3
tmux new-window -t $SESSION_NAME:1 -n "Music"
tmux send-keys -t $SESSION_NAME:1 "echo 'Starting music...'" C-m

# Check if there are any mp3 files
MP3_COUNT=$(find /home/ditrobotics/Music -name "*.mp3" 2>/dev/null | wc -l)

if [ "$MP3_COUNT" -gt 0 ]; then
    # Select random MP3 file and play it
    tmux send-keys -t $SESSION_NAME:1 "RANDOM_MP3=\$(find /home/ditrobotics/Music -name '*.mp3' | shuf -n 1)" C-m
    tmux send-keys -t $SESSION_NAME:1 "echo 'Playing: \$RANDOM_MP3'" C-m
    
    # Play audio using ffplay only
    tmux send-keys -t $SESSION_NAME:1 "if command -v ffplay >/dev/null 2>&1; then" C-m
    tmux send-keys -t $SESSION_NAME:1 "    echo 'Playing with ffplay'" C-m
    tmux send-keys -t $SESSION_NAME:1 "    ffplay -nodisp -autoexit \"\$RANDOM_MP3\"" C-m
    tmux send-keys -t $SESSION_NAME:1 "else" C-m
    tmux send-keys -t $SESSION_NAME:1 "    echo 'ffplay not found. Please install ffmpeg package'" C-m
    tmux send-keys -t $SESSION_NAME:1 "    echo 'You can install it with: sudo apt-get install ffmpeg'" C-m
    tmux send-keys -t $SESSION_NAME:1 "fi" C-m
else
    tmux send-keys -t $SESSION_NAME:1 "echo 'No MP3 files found in /home/ditrobotics/Music'" C-m
fi

# Create a key binding to handle double pressing Prefix+C-c to clean up all sessions
tmux bind-key C-c run-shell "bash -c 'pkill -f \"${SESSION_NAME}\"'"

# Display info message about how to exit
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Press Ctrl+B then Ctrl+C twice to exit all sessions'" C-m

# Attach to the session
tmux select-window -t $SESSION_NAME:0
tmux attach-session -t $SESSION_NAME

# When detached, perform cleanup
cleanup_all

# Restore TMUX variable if it was unset
if [ -n "$TMUX_TEMP" ]; then
    export TMUX=$TMUX_TEMP
fi