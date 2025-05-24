#!/bin/bash
# Main program script - simplified version

# Check if already inside a tmux session
if [ -n "$TMUX" ]; then
    echo "Already inside a tmux session. Detaching first to prevent nesting issues..."
    TMUX_TEMP=$TMUX
    unset TMUX
    echo "TMUX variable temporarily unset."
fi

# Check if script is already running
SESSION_NAME="eurobot-main-session"
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "Session already exists, cleaning up..."
    
    # Stop main process
    echo "Stopping main program..."
    pkill -f "/home/main/Eurobot-2025-Main/main run"
    
    # Try graceful shutdown if available
    echo "ditrobotics" | sudo -S -u main /home/main/Eurobot-2025-Main/main close 2>/dev/null
    
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

# Run main program
tmux send-keys -t $SESSION_NAME:0 "echo 'Starting main program...'" C-m
tmux send-keys -t $SESSION_NAME:0 "echo 'ditrobotics' | sudo -S -u main /home/main/Eurobot-2025-Main/main run" C-m

# Create a key binding to close the session
tmux bind-key -T prefix C-d run-shell "tmux kill-session -t $SESSION_NAME"
tmux send-keys -t $SESSION_NAME:0 "echo 'Press Ctrl+B then Ctrl+D to close this session'" C-m

# Attach to the session
tmux attach-session -t $SESSION_NAME

# Restore TMUX variable if it was unset
if [ -n "$TMUX_TEMP" ]; then
    export TMUX=$TMUX_TEMP
fi
