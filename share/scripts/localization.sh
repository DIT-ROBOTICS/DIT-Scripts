#!/bin/bash
# Combined localization script for both blue and yellow teams

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
    tmux kill-session -t eurobot-local-session 2>/dev/null
    
    # Stop docker containers - try both colors
    echo "Stopping localization containers..."
    echo "ditrobotics" | sudo -S docker compose -p localization-blue -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/blue-compose.yml down 2>/dev/null
    echo "ditrobotics" | sudo -S docker compose -p localization-yellow -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/yellow-compose.yml down 2>/dev/null
    
    # Stop any remaining containers related to our application
    BLUE_CONTAINERS=$(echo "ditrobotics" | sudo -S docker ps -q --filter "name=localization-blue" 2>/dev/null)
    YELLOW_CONTAINERS=$(echo "ditrobotics" | sudo -S docker ps -q --filter "name=localization-yellow" 2>/dev/null)
    
    if [ -n "$BLUE_CONTAINERS" ] || [ -n "$YELLOW_CONTAINERS" ]; then
        echo "Stopping remaining containers..."
        [ -n "$BLUE_CONTAINERS" ] && echo "ditrobotics" | sudo -S docker stop $BLUE_CONTAINERS 2>/dev/null
        [ -n "$YELLOW_CONTAINERS" ] && echo "ditrobotics" | sudo -S docker stop $YELLOW_CONTAINERS 2>/dev/null
    fi
    
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
SESSION_NAME="eurobot-local-session"
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "Session already exists, cleaning up..."
    cleanup_all
    
    # Restore TMUX variable if it was unset
    if [ -n "$TMUX_TEMP" ]; then
        export TMUX=$TMUX_TEMP
    fi
    exit 0
fi

# Check button.json for team selection
BUTTON_FILE="/home/share/data/button.json"
# If primary location doesn't exist, try alternative location
if [ ! -f "$BUTTON_FILE" ]; then
    BUTTON_FILE="/home/ditrobotics/DIT-Scripts/share/data/button.json"
    echo "Trying alternative button file location: $BUTTON_FILE"
fi
if [ ! -f "$BUTTON_FILE" ]; then
    echo "Button file not found at $BUTTON_FILE"
    
    # Restore TMUX variable if it was unset
    if [ -n "$TMUX_TEMP" ]; then
        export TMUX=$TMUX_TEMP
    fi
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
    echo "No team buttons (10, 11, 13 or 15, 17, 19) are active. Exiting."
    
    # Restore TMUX variable if it was unset
    if [ -n "$TMUX_TEMP" ]; then
        export TMUX=$TMUX_TEMP
    fi
    exit 1
fi

# Create a new session with mouse/touch support
tmux new-session -d -s $SESSION_NAME

# Enable mouse mode for touch scrolling
tmux set-option -t $SESSION_NAME mouse on
tmux set-option -t $SESSION_NAME history-limit 10000

# Run localization based on selected team
tmux send-keys -t $SESSION_NAME:0 "echo 'Starting $TEAM team localization...'" C-m

if [ "$TEAM" = "blue" ]; then
    tmux send-keys -t $SESSION_NAME:0 "echo 'ditrobotics' | sudo -S docker compose -p localization-blue -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/blue-compose.yml up" C-m
else
    tmux send-keys -t $SESSION_NAME:0 "echo 'ditrobotics' | sudo -S docker compose -p localization-yellow -f /home/localization/Eurobot-2025-Localization/docker/testBot/OdomComm/docker/yellow-compose.yml up" C-m
fi

# Create a key binding to close the session with Ctrl+B, Ctrl+D
tmux bind-key -T prefix C-d run-shell "tmux kill-session -t $SESSION_NAME"
tmux send-keys -t $SESSION_NAME:0 "echo 'Press Ctrl+B then Ctrl+D to close this session'" C-m

# Attach to the session
tmux attach-session -t $SESSION_NAME

# When detached, perform cleanup
cleanup_all

# Restore TMUX variable if it was unset
if [ -n "$TMUX_TEMP" ]; then
    export TMUX=$TMUX_TEMP
fi
