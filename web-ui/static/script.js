
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
}, 1000);

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

setInterval(function() {
    fetch('/usb')
    .then(response => response.json())
    .then(data => {
        document.getElementById('stm_00').textContent = `${data.stm_00 === 0 ? '❎' : '✅'} STM0`;
        document.getElementById('stm_01').textContent = `${data.stm_01 === 0 ? '❎' : '✅'} STM1`;
        document.getElementById('lidar').textContent = `${data.lidar === 0 ? '❎' : '✅'} LIDAR`;
        document.getElementById('esp32').textContent = `${data.esp32 === 0 ? '❎' : '✅'} ESP32`;
        document.getElementById('vive_tracker').textContent = `${data.vive_tracker === 0 ? '❎' : '✅'} VIVE`;
    })
    .catch(error => console.error('Error loading the group data:', error));
}, 1000);


document.getElementById('fullscreenButton').addEventListener('click', toggleFullScreen);
function toggleFullScreen() {
  if (!document.fullscreenElement) {
    document.documentElement.requestFullscreen().catch(err => {
      alert(`Cannot enter fullscreen mode: ${err.message}`);
    });
  } else {
    if (document.exitFullscreen) {
      document.exitFullscreen();
    }
  }
}

