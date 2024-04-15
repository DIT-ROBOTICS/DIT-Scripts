
setInterval(function() {
    fetch('/score')
    .then(response => response.text())
    .then(number => {
        document.getElementById('score').textContent = number;
    })
    .catch(error => console.error('Error fetching number:', error));
}, 1000);

setInterval(function() {
    fetch('/voltage')
    .then(response => response.text())
    .then(number => {
        document.getElementById('number-display').textContent = number + ' V';
    })
    .catch(error => console.error('Error fetching number:', error));
}, 1000); // The function runs every 1000 milliseconds (1 second)

function updateInfo() {
    fetch('/group')
    .then(response => response.json())
    .then(data => {
        document.getElementById('main-info').textContent = `${data.main === 0 ? '⟳' : '✅'} 主程式`;
        document.getElementById('vision-info').textContent = `${data.vision === 0 ? '⟳' : '✅'} 相機組`;
        document.getElementById('navigation-info').textContent = `${data.navigation === 0 ? '⟳' : '✅'} 導航組`;
        document.getElementById('localization-info').textContent = `${data.localization === 0 ? '⟳' : '✅'} 定位組`;
    })
    .catch(error => console.error('Error loading the group data:', error));
}

setInterval(updateInfo, 1000);

