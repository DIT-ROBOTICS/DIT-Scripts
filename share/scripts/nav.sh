#!/bin/bash
# Navigation script - simplified version

# Check if already inside a tmux session
if [ -n "$TMUX" ]; then
    echo "Already inside a tmux session. Detaching first to prevent nesting issues..."
    TMUX_TEMP=$TMUX
    unset TMUX
    echo "TMUX variable temporarily unset."
fi

# Check if script is already running
SESSION_NAME="eurobot-nav-session"
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "Session already exists, cleaning up..."
    
    # Stop docker containers
    echo "Stopping navigation..."
    echo "ditrobotics" | sudo -S docker compose -p navigation-run -f /home/navigation/Eurobot-2025-machine-ws/src/Eurobot-2025-Navigation2-envs/Navigation2-humble-deploy/docker-compose.yaml down
    
    # Kill existing session
    tmux kill-session -t $SESSION_NAME 2>/dev/null
    
    echo "Cleanup complete."
    
    # Restore TMUX variable if it was unset
    if [ -n "$TMUX_TEMP" ]; then
        export TMUX=$TMUX_TEMP
    fi
    exit 0
fi

# Create a new session with mouse/touch support
tmux new-session -d -s $SESSION_NAME

# Enable mouse mode for touch scrolling
tmux set-option -t $SESSION_NAME mouse on
tmux set-option -t $SESSION_NAME history-limit 10000

# Run navigation
tmux send-keys -t $SESSION_NAME:0 "echo 'Starting navigation...'" C-m
tmux send-keys -t $SESSION_NAME:0 "echo 'ditrobotics' | sudo -S docker compose -p navigation-run -f /home/navigation/Eurobot-2025-machine-ws/src/Eurobot-2025-Navigation2-envs/Navigation2-humble-deploy/docker-compose.yaml up navigation-run" C-m

# Create a key binding to close the session
tmux bind-key -T prefix C-d run-shell "tmux kill-session -t $SESSION_NAME"
tmux send-keys -t $SESSION_NAME:0 "echo 'Press Ctrl+B then Ctrl+D to close this session'" C-m

# Attach to the session
tmux attach-session -t $SESSION_NAME

# Restore TMUX variable if it was unset
if [ -n "$TMUX_TEMP" ]; then
    export TMUX=$TMUX_TEMP
fi
