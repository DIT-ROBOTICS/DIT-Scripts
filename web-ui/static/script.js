setInterval(function() {
    fetch('/voltage')
    .then(response => response.text())
    .then(number => {
        document.getElementById('number-display').textContent = number + ' V';
    })
    .catch(error => console.error('Error fetching number:', error));
}, 1000); // The function runs every 1000 milliseconds (1 second)
