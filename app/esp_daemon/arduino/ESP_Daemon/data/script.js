// Dark mode toggle functionality
document.addEventListener('DOMContentLoaded', function() {
  const modeToggle = document.getElementById('modeToggle');
  
  // Check for saved theme preference or use default (dark)
  const currentTheme = localStorage.getItem('theme') || 'dark';
  
  // Apply saved theme on page load
  if (currentTheme === 'light') {
    document.documentElement.classList.add('light-mode');
    modeToggle.textContent = 'DARK MODE';
    updateChartTheme(true);
  } else {
    modeToggle.textContent = 'LIGHT MODE';
    updateChartTheme(false);
  }
  
  // Toggle between light/dark modes
  modeToggle.addEventListener('click', function() {
    const isLightMode = document.documentElement.classList.toggle('light-mode');
    
    if (isLightMode) {
      modeToggle.textContent = 'DARK MODE';
      localStorage.setItem('theme', 'light');
      updateChartTheme(true);
    } else {
      modeToggle.textContent = 'LIGHT MODE';
      localStorage.setItem('theme', 'dark');
      updateChartTheme(false);
    }
  });
});

// Function to update chart theme
function updateChartTheme(isLightMode) {
  const backgroundColor = isLightMode ? '#f0f0f0' : '#1e1e1e';
  const textColor = isLightMode ? '#333333' : '#f5f5f5';
  const gridColor = isLightMode ? '#d0d0d0' : '#333333';
  
  if (chartT) {
    chartT.update({
      chart: {
        backgroundColor: backgroundColor
      },
      title: {
        style: {
          color: textColor
        }
      },
      xAxis: {
        labels: {
          style: {
            color: textColor
          }
        },
        gridLineColor: gridColor
      },
      yAxis: {
        labels: {
          style: {
            color: textColor
          }
        },
        title: {
          style: {
            color: textColor
          }
        },
        gridLineColor: gridColor
      },
      legend: {
        itemStyle: {
          color: textColor
        }
      }
    });
  }
}

// Enhanced function to handle chart resizing when window is resized
function handleResize() {
  if (chartT) {
    // Force chart container to take available space
    const chartContainer = document.getElementById('chart-sensor');
    const card = chartContainer.closest('.card');
    const statusContainer = document.querySelector('.status-container');
    const cardGrid = document.querySelector('.card-grid');
    const statusItemsContainer = document.querySelector('.status-items-container');
    
    // Wait a moment for DOM to settle
    setTimeout(() => {
      // Calculate available height (card height minus title and padding)
      const titleHeight = card.querySelector('.card-title').offsetHeight;
      const cardPadding = parseInt(window.getComputedStyle(card).padding) * 2;
      const availableHeight = card.clientHeight - titleHeight - cardPadding;
      
      // Set chart size to available dimensions
      chartT.setSize(
        chartContainer.offsetWidth, 
        Math.max(200, availableHeight),
        false
      );
      chartT.reflow();
      
      // Size both containers with proper ratio and alignment
      if (statusContainer && cardGrid) {
        if (window.innerWidth <= 1200) {
          // Mobile view - full width, equal widths
          statusContainer.style.width = '100%';
          cardGrid.style.width = '100%';
          cardGrid.style.maxWidth = '100%';
          
          // Reset heights to auto when in stacked layout
          statusContainer.style.height = 'auto';
          cardGrid.style.height = 'auto';
        } else {
          // Desktop view - fixed ratio
          statusContainer.style.width = '35%';
          cardGrid.style.width = '60%';
          
          // Get actual content height of status items
          let statusTitle = statusContainer.querySelector('.status-title');
          let statusItemsHeight = 0;
          
          if (statusItemsContainer) {
            // Calculate the exact space needed by summing all status item heights
            Array.from(statusItemsContainer.children).forEach(item => {
              statusItemsHeight += item.offsetHeight + parseInt(window.getComputedStyle(item).marginTop) + 
                                    parseInt(window.getComputedStyle(item).marginBottom);
            });
            
            // Add additional padding
            statusItemsHeight += 20;
          }
          
          // Calculate total height needed for status container
          const titleHeight = statusTitle ? statusTitle.offsetHeight : 0;
          const statusPadding = parseInt(window.getComputedStyle(statusContainer).padding) * 2;
          const statusContentHeight = titleHeight + statusItemsHeight + statusPadding;
          
          // Set height based on the taller of the two - either content height or match card height
          const cardHeight = cardGrid.offsetHeight;
          
          // Use content height, but don't exceed card height
          statusContainer.style.height = Math.min(statusContentHeight, cardHeight) + 'px';
          
          // If status container is shorter, align it to the top
          if (statusContentHeight < cardHeight) {
            statusContainer.style.alignSelf = 'flex-start';
          } else {
            statusContainer.style.alignSelf = 'stretch';
          }
        }
      }
      
      // Update font sizes for chart based on screen size
      updateChartFontSizes();
    }, 50);
  }
}

function updateChartFontSizes() {
  const baseSize = window.innerWidth >= 2000 ? 14 : 
                  window.innerWidth >= 1600 ? 13 :
                  window.innerWidth >= 768 ? 12 : 10;
  
  if (chartT) {
    chartT.update({
      xAxis: {
        labels: {
          style: {
            fontSize: baseSize + 'px'
          }
        }
      },
      yAxis: {
        title: {
          style: {
            fontSize: (baseSize + 1) + 'px'
          }
        },
        labels: {
          style: {
            fontSize: baseSize + 'px'
          }
        }
      },
      legend: {
        itemStyle: {
          fontSize: (baseSize + 2) + 'px'
        }
      }
    }, false);
    chartT.redraw();
  }
}

// Add event listener for window resize with debouncing
let resizeTimer;
window.addEventListener('resize', function() {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(handleResize, 100);
});

// Get current sensor readings when the page loads
window.addEventListener('load', function() {
  getReadings();
  // Initial sizing after everything is loaded
  setTimeout(handleResize, 200);
  // And another check after a bit longer to ensure all resources are loaded
  setTimeout(handleResize, 500);
});

var chartT = new Highcharts.Chart({
  chart:{
    renderTo:'chart-sensor',
    backgroundColor: document.documentElement.classList.contains('light-mode') ? '#f0f0f0' : '#1e1e1e',
    reflow: true,
    animation: false,
    height: '100%',
    responsive: {
      rules: [{
        condition: {
          maxWidth: 600
        },
        chartOptions: {
          legend: {
            enabled: true,
            layout: 'horizontal',
            align: 'center',
            verticalAlign: 'bottom',
            itemMarginTop: 2,
            itemMarginBottom: 2
          },
          chart: {
            height: '75%'
          }
        }
      }]
    },
    spacing: [15, 15, 15, 15],
    style: {
      fontFamily: "'Segoe UI', 'Roboto', Arial, sans-serif",
      fontSize: '14px'
    }
  },
  series: [
    {
      name: 'Battery Voltage',
      type: 'line',
      color: '#e53935',
      marker: {
        symbol: 'circle',
        radius: 4,
        fillColor: '#e53935',
      },
      lineWidth: 3
    },
    {
      name: 'Filter Value',
      type: 'line',
      color: '#ff7043',
      marker: {
        symbol: 'square',
        radius: 4,
        fillColor: '#ff7043',
      },
      lineWidth: 3
    },
  ],
  title: {
    text: undefined
  },
  xAxis: {
    type: 'datetime',
    dateTimeLabelFormats: { second: '%H:%M:%S' },
    labels: {
      style: {
        color: document.documentElement.classList.contains('light-mode') ? '#333333' : '#f5f5f5',
        fontSize: '12px'
      }
    },
    tickLength: 5
  },
  yAxis: {
    title: {
      text: 'Voltage',
      style: {
        color: document.documentElement.classList.contains('light-mode') ? '#333333' : '#f5f5f5',
        fontSize: '13px'
      }
    },
    labels: {
      style: {
        color: document.documentElement.classList.contains('light-mode') ? '#333333' : '#f5f5f5',
        fontSize: '12px'
      }
    }
  },
  legend: {
    itemStyle: {
      color: document.documentElement.classList.contains('light-mode') ? '#333333' : '#f5f5f5',
      fontSize: '1.1rem'
    },
    itemMarginTop: 5,
    itemMarginBottom: 5,
    padding: 10
  },
  credits: {
    enabled: false
  }
});

function plotData(jsonValue) {
  var batteryStatus = jsonValue["batteryStatus"];
  document.getElementById("batteryStatus").innerText = "Battery: " + batteryStatus;
  var microROS = jsonValue["microROS"];
  document.getElementById("microROS").innerText = "micro-ROS: " + microROS;

  // Map keys to chart series indices
  const seriesMapping = {
    sensor: 0, // Corresponds to 'Battery Voltage'
    GND: 1     // Corresponds to 'Filter Value'
  };

  var keys = Object.keys(jsonValue);
  // console.log(keys);

  for (var i = 0; i < keys.length; i++) {
    const key = keys[i];
    if (seriesMapping[key] !== undefined) { // Only process valid series keys
      var x = (new Date()).getTime();
      // console.log(x);
      var y = Number(jsonValue[key]);
      // console.log(y);

      const seriesIndex = seriesMapping[key];
      if (chartT.series[seriesIndex].data.length > 600) {
        chartT.series[seriesIndex].addPoint([x, y], true, true, true);
      } else {
        chartT.series[seriesIndex].addPoint([x, y], true, false, true);
      }
    }
  }
}

// Function to get current readings on the webpage when it loads for the first time
function getReadings(){
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var myObj = JSON.parse(this.responseText);
      // console.log(myObj);

    var batteryStatus = myObj["batteryStatus"];
    document.getElementById("batteryStatus").innerText = "Battery: " + batteryStatus;
    var microROS = myObj["microROS"];
    document.getElementById("microROS").innerText = "micro-ROS: " + microROS;
    }
  };
  xhr.open("GET", "/readings", true);
  xhr.send();
}

if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function(e) {
    // console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      // console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('message', function(e) {
    // console.log("message", e.data);
  }, false);

  source.addEventListener('new_readings', function(e) {
    // console.log("new_readings", e.data);
    var myObj = JSON.parse(e.data);
    // console.log(myObj);
    plotData(myObj);
  }, false);
}

// Ensure chart redraws properly on orientation change (particularly important for mobile)
window.addEventListener('orientationchange', function() {
  setTimeout(handleResize, 300);
});
