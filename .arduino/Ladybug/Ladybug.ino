#include <WiFi.h>
#include <AsyncTCP.h>
#include <AsyncElegantOTA.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

/************************     OEM     ***************************/
#define Hostname "ladybug-01" // mDNS Hostname
#define RS 2                  // ReadySignal PINOUT (onboard LED)

int readySignal = -1, color = -1;
const char* ssid = "DIT_8C58";          // WiFi SSID
const char* password = "ditrobotics";   // WiFi PWD
/****************************************************************/

AsyncWebServer server(80);

const char* PARAM_INPUT_1 = "color";
const char* PARAM_INPUT_2 = "state";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>DIT-Ladybug</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {
      font-family: Arial;
      text-align: center;
    }
    body {
      max-width: 600px;
      margin: 0 auto;
      padding-bottom: 25px;
    }
    h2 {
      font-size: 3.0rem;
    }
    p {
      font-size: 3.0rem;
    }
    .button {
      background-color: #DE272C;
      color: white;
      padding: 15px 32px;
      font-size: 28px;
      margin: 4px 2px;
      cursor: pointer;
      border: none;
      border-radius: 6px;
      text-decoration: none;
      display: inline-block;
      text-align: center;
    }
    .switch {
      position: relative;
      display: inline-block;
      width: 120px;
      height: 68px;
    }
    .slider {
      position: absolute;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background-color: blue;
      border-radius: 6px;
      transition: .4s;
    }
    .slider:before {
      position: absolute;
      content: "";
      height: 52px;
      width: 52px;
      left: 8px;
      bottom: 8px;
      background-color: white;
      border-radius: 3px;
    }
    input:checked + .slider {
      background-color: yellow;
    }
    input:checked + .slider:before {
      transform: translateX(52px);
    }
  </style>
</head>
<body>
  <h2>Eurobot 2024</h2>
  <label class="switch">
    <input type="checkbox" id="colorToggle" onchange="toggleColor(this)">
    <span class="slider"></span>
  </label>
  <div> </div>
  <button class="button" onclick="sendState(0)">RESET</button>
  <button class="button" onclick="sendState(1)">READY</button>
<script>

function toggleColor(element) {
  var xhr = new XMLHttpRequest();
  var colorValue = element.checked ? 1 : 0;
  xhr.open("GET", "/updates?color=" + colorValue + "&state=" + state, true);
  xhr.send();
}

function sendState(state) {
  var xhr = new XMLHttpRequest();
  var color = document.getElementById('colorToggle').checked ? 1 : 0;
  xhr.open("GET", "/updates?color=" + color + "&state=" + state, true);
  xhr.send();
}
</script>
</body>
</html>
)rawliteral";

String processor(const String& var) {
  return String();
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  if (!MDNS.begin(Hostname)) {
    Serial.println("Error starting mDNS");
    return;
  }
  Serial.println(WiFi.localIP());
  /* Place Your WiFi Ready things here  */ 


  /*              END                    */
}

void setup(void) {
  pinMode(RS, OUTPUT);
  Serial.begin(115200);
  initWiFi();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/updates", HTTP_GET, [](AsyncWebServerRequest* request) {
    String inputMessage1, inputMessage2;
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      color = inputMessage1.toInt();
      readySignal = inputMessage2.toInt();
    } else {
      inputMessage1 = "No message sent";
    }
    request->send(200, "text/plain", "DIT Robotics");
  });

  AsyncElegantOTA.begin(&server);  // Start ElegantOTA
  server.begin();
}

void loop(void) {
  Serial.print("{color: ");
  Serial.print(color);
  Serial.print(", readySignal: ");
  Serial.print(readySignal);
  Serial.println("}");
}