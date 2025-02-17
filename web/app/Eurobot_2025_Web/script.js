// Connect to ROS2 backend using ROS2 Bridge
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090' // Adjust to your ROS2 websocket URL
});

ros.on('connection', () => {
    console.log('Connected to ROS2');
});

ros.on('error', (error) => {
    console.error('Error connecting to ROS2:', error);
});

ros.on('close', () => {
    console.log('Disconnected from ROS2');
});

// Example subscription
const scoreListener = new ROSLIB.Topic({
    ros: ros,
    name: '/score',
    messageType: 'std_msgs/Int32'
});

scoreListener.subscribe((message) => {
    document.querySelector('.score h1').textContent = message.data;
});

// Additional ROS2 integration here
