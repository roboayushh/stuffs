// Required Libraries:
// 1. WebServer (part of ESP32 core)
// Install WebSockets from the Arduino IDE Library Manager before compiling.

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// --- WiFi AP (Access Point) Credentials ---
// The ESP32 will create its own WiFi network with this name.
const char* ssid = "auv";
const char* password = "12345678";

// --- Global Objects ---
// Create WebServer object on port 80
WebServer server(80);
// Create a WebSocket server on port 81
WebSocketsServer webSocket = WebSocketsServer(81);

// --- HTML Content in PROGMEM ---
// This is the exact layout you provided, with embedded CSS for offline use.
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>ESP32 Universal Controller</title>
    <style>
        /* Embedded Tailwind CSS for offline use */
        *,::before,::after{box-sizing:border-box;border-width:0;border-style:solid;border-color:#e5e7eb}html{line-height:1.5;-webkit-text-size-adjust:100%;-moz-tab-size:4;tab-size:4;font-family:ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,"Noto Sans",sans-serif,"Apple Color Emoji","Segoe UI Emoji","Segoe UI Symbol","Noto Color Emoji"}body{margin:0;line-height:inherit;font-family:ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,"Noto Sans",sans-serif;touch-action:manipulation}h1,p{margin:0}button{color:inherit;background-color:transparent;background-image:none;cursor:pointer}.bg-gray-800{background-color:rgb(31 41 55)}.bg-gray-900{background-color:rgb(17 24 39)}.bg-teal-500{background-color:rgb(20 184 166)}.hover\:bg-teal-600:hover{background-color:rgb(13 148 136)}.active\:bg-teal-700:active{background-color:rgb(15 118 110)}.rounded-lg{border-radius:.5rem}.shadow-lg{box-shadow:0 10px 15px -3px rgba(0,0,0,.1),0 4px 6px -4px rgba(0,0,0,.1)}.shadow-md{box-shadow:0 4px 6px -1px rgba(0,0,0,.1),0 2px 4px -2px rgba(0,0,0,.1)}.transition-all{transition-property:all;transition-timing-function:cubic-bezier(.4,0,.2,1);transition-duration:.15s}.duration-150{transition-duration:.15s}.transform{transform:translate(var(--tw-translate-x),var(--tw-translate-y)) rotate(var(--tw-rotate)) skewX(var(--tw-skew-x)) skewY(var(--tw-skew-y)) scaleX(var(--tw-scale-x)) scaleY(var(--tw-scale-y))}.active\:scale-95:active{--tw-scale-x:.95;--tw-scale-y:.95;transform:translate(var(--tw-translate-x),var(--tw-translate-y)) rotate(var(--tw-rotate)) skewX(var(--tw-skew-x)) skewY(var(--tw-skew-y)) scaleX(var(--tw-scale-x)) scaleY(var(--tw-scale-y))}.gap-1{gap:.25rem}.gap-2{gap:.5rem}.gap-3{gap:.75rem}.gap-4{gap:1rem}.gap-6{gap:1.5rem}.gap-8{gap:2rem}.grid{display:grid}.grid-cols-2{grid-template-columns:repeat(2,minmax(0,1fr))}.grid-cols-3{grid-template-columns:repeat(3,minmax(0,1fr))}.grid-cols-6{grid-template-columns:repeat(6,minmax(0,1fr))}.w-full{width:100%}.min-h-screen{min-height:100vh}.mx-auto{margin-left:auto;margin-right:auto}.mb-1{margin-bottom:.25rem}.mb-2{margin-bottom:.5rem}.mb-3{margin-bottom:.75rem}.mt-1{margin-top:.25rem}.mt-2{margin-top:.5rem}.mt-3{margin-top:.75rem}.flex{display:flex}.flex-col{flex-direction:column}.flex-row{flex-direction:row}.items-center{align-items:center}.justify-center{justify-content:center}.max-w-6xl{max-width:72rem}.break-all{word-break:break-all}.text-center{text-align:center}.text-xs{font-size:.75rem;line-height:1rem}.text-sm{font-size:.875rem;line-height:1.25rem}.text-base{font-size:1rem;line-height:1.5rem}.text-lg{font-size:1.125rem;line-height:1.75rem}.text-xl{font-size:1.25rem;line-height:1.75rem}.text-2xl{font-size:1.5rem;line-height:2rem}.text-3xl{font-size:1.875rem;line-height:2.25rem}.font-bold{font-weight:700}.font-mono{font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,"Liberation Mono","Courier New",monospace}.font-semibold{font-weight:600}.text-cyan-400{color:rgb(34 211 238)}.text-gray-400{color:rgb(156 163 175)}.text-green-400{color:rgb(74 222 128)}.text-red-500{color:rgb(239 68 68)}.text-white{color:rgb(255 255 255)}.p-1{padding:.25rem}.p-2{padding:.5rem}.p-3{padding:.75rem}.p-4{padding:1rem}@media (min-width:640px){.sm\:mb-2{margin-bottom:.5rem}.sm\:mb-3{margin-bottom:.75rem}.sm\:mb-4{margin-bottom:1rem}.sm\:mt-2{margin-top:.5rem}.sm\:mt-3{margin-top:.75rem}.sm\:gap-2{gap:.5rem}.sm\:gap-3{gap:.75rem}.sm\:gap-4{gap:1rem}.sm\:gap-6{gap:1.5rem}.sm\:gap-8{gap:2rem}.sm\:grid-cols-3{grid-template-columns:repeat(3,minmax(0,1fr))}.sm\:grid-cols-6{grid-template-columns:repeat(6,minmax(0,1fr))}.sm\:p-2{padding:.5rem}.sm\:p-3{padding:.75rem}.sm\:p-4{padding:1rem}.sm\:text-sm{font-size:.875rem;line-height:1.25rem}.sm\:text-base{font-size:1rem;line-height:1.5rem}.sm\:text-lg{font-size:1.125rem;line-height:1.75rem}.sm\:text-xl{font-size:1.25rem;line-height:1.75rem}}@media (min-width:768px){.md\:mb-3{margin-bottom:.75rem}.md\:mb-4{margin-bottom:1rem}.md\:mt-3{margin-top:.75rem}.md\:gap-4{gap:1rem}.md\:gap-6{gap:1.5rem}.md\:gap-8{gap:2rem}.md\:grid-cols-6{grid-template-columns:repeat(6,minmax(0,1fr))}.md\:p-3{padding:.75rem}.md\:p-4{padding:1rem}.md\:text-base{font-size:1rem;line-height:1.5rem}.md\:text-lg{font-size:1.125rem;line-height:1.75rem}.md\:text-xl{font-size:1.25rem;line-height:1.75rem}.md\:text-2xl{font-size:1.5rem;line-height:2rem}.md\:text-3xl{font-size:1.875rem;line-height:2.25rem}}@media (min-width:1024px){.lg\:gap-8{gap:2rem}.lg\:text-3xl{font-size:1.875rem;line-height:2.25rem}.lg\:text-xl{font-size:1.25rem;line-height:1.75rem}}
        
        .joystick-base{position:relative;width:120px;height:120px;background-color:#4A5568;border-radius:50%;display:flex;justify-content:center;align-items:center;box-shadow:inset 0 0 20px rgba(0,0,0,.5)}
        @media (max-width:640px){.joystick-base{width:90px;height:90px}}
        @media (min-width:641px) and (max-width:1024px){.joystick-base{width:110px;height:110px}}
        @media (min-width:1025px){.joystick-base{width:130px;height:130px}}
        .joystick-stick{position:absolute;width:48px;height:48px;background:linear-gradient(145deg,#2D3748,#4A5568);border-radius:50%;cursor:grab;border:2px solid #718096;box-shadow:0 5px 15px rgba(0,0,0,.4);display:flex;justify-content:center;align-items:center}
        @media (max-width:640px){.joystick-stick{width:36px;height:36px}}
        @media (min-width:641px) and (max-width:1024px){.joystick-stick{width:42px;height:42px}}
        @media (min-width:1025px){.joystick-stick{width:52px;height:52px}}
        .joystick-stick:active{cursor:grabbing;background:linear-gradient(145deg,#1A202C,#2D3748)}
        .joystick-base.middle{height:60px;border-radius:30px}
        @media (max-width:640px){.joystick-base.middle{height:45px;border-radius:22.5px}}
        @media (min-width:641px) and (max-width:1024px){.joystick-base.middle{height:52px;border-radius:26px}}
        @media (min-width:1025px){.joystick-base.middle{height:70px;border-radius:35px}}
        .joystick-base,.joystick-stick{-webkit-user-select:none;-moz-user-select:none;-ms-user-select:none;user-select:none}
        .pitch-button{width:80px;font-size:14px;padding:8px 12px}
        @media (max-width:640px){.pitch-button{width:60px;font-size:11px;padding:6px 8px}}
        @media (min-width:641px) and (max-width:1024px){.pitch-button{width:70px;font-size:12px;padding:7px 10px}}
        @media (min-width:1025px){.pitch-button{width:90px;font-size:15px;padding:10px 14px}}
        button{-webkit-tap-highlight-color:transparent;-webkit-touch-callout:none;-webkit-user-select:none;-khtml-user-select:none;-moz-user-select:none;-ms-user-select:none;user-select:none}
        .touch-manipulation{touch-action:manipulation}
    </style>
</head>
<body class="bg-gray-900 text-white min-h-screen p-1 sm:p-2 md:p-3">

    <div class="w-full max-w-6xl mx-auto">
        <!-- Header -->
        <div class="text-center mb-2 sm:mb-3">
            <h1 class="text-lg sm:text-xl md:text-2xl lg:text-3xl font-bold text-cyan-400">ESP32 Universal Controller</h1>
            <p class="text-xs sm:text-sm text-gray-400">Connection Status: <span id="status" class="font-semibold text-red-500">Disconnected</span></p>
        </div>

        <!-- Data Display -->
        <div class="bg-gray-800 p-2 sm:p-3 md:p-4 rounded-lg shadow-lg mb-3 sm:mb-4 text-center font-mono">
            <p class="text-xs sm:text-sm md:text-lg">Formatted Data String:</p>
            <p id="data-display" class="text-sm sm:text-base md:text-lg lg:text-xl text-green-400 break-all">-</p>
            <div class="grid grid-cols-3 sm:grid-cols-6 gap-1 sm:gap-2 mt-2 sm:mt-3 text-xs sm:text-sm">
                <div>X: <span id="val-x">0.00</span></div>
                <div>Y: <span id="val-y">0.00</span></div>
                <div>Z: <span id="val-z">0.00</span></div>
                <div>Yaw: <span id="val-yaw">0.00</span></div>
                <div>Pitch: <span id="val-pitch">0.00</span></div>
                <div>Roll: <span id="val-roll">0.00</span></div>
            </div>
        </div>

        <!-- Controls Area -->
        <div class="flex flex-col gap-3 sm:gap-4 md:gap-6">
            
            <!-- Joysticks Row -->
            <div class="flex flex-row items-center justify-center gap-3 sm:gap-4 md:gap-6 lg:gap-8">
                
                <!-- Left Joystick -->
                <div class="flex flex-col items-center">
                    <p class="mb-1 sm:mb-2 font-semibold text-xs sm:text-sm md:text-base">Pitch & Yaw</p>
                    <div id="joystick-left-base" class="joystick-base">
                        <div id="joystick-left-stick" class="joystick-stick"></div>
                    </div>
                    <div class="text-xs mt-1 sm:mt-2 text-gray-400 text-center">UP/DOWN: Pitch<br>LEFT/RIGHT: Yaw</div>
                </div>

                <!-- Right Joystick -->
                <div class="flex flex-col items-center">
                    <p class="mb-1 sm:mb-2 font-semibold text-xs sm:text-sm md:text-base">X & Z</p>
                    <div id="joystick-right-base" class="joystick-base">
                        <div id="joystick-right-stick" class="joystick-stick"></div>
                    </div>
                    <div class="text-xs mt-1 sm:mt-2 text-gray-400 text-center">UP/DOWN: X<br>LEFT/RIGHT: Z</div>
                </div>

            </div>

            <!-- Controls Row -->
            <div class="flex flex-row items-center justify-center gap-4 sm:gap-6 md:gap-8">
                
                <!-- Y Controls -->
                <div class="flex flex-col items-center">
                    <p class="mb-1 sm:mb-2 font-semibold text-xs sm:text-sm md:text-base">Y Control</p>
                    <div class="flex flex-col gap-2 sm:gap-3">
                        <button id="y-up" class="pitch-button bg-teal-500 hover:bg-teal-600 active:bg-teal-700 text-white font-bold rounded-lg shadow-md transition-all duration-150 transform active:scale-95 touch-manipulation">Up</button>
                        <button id="y-down" class="pitch-button bg-teal-500 hover:bg-teal-600 active:bg-teal-700 text-white font-bold rounded-lg shadow-md transition-all duration-150 transform active:scale-95 touch-manipulation">Down</button>
                    </div>
                </div>

                <!-- Middle Joystick -->
                <div class="flex flex-col items-center">
                    <p class="mb-1 sm:mb-2 font-semibold text-xs sm:text-sm md:text-base">Roll</p>
                    <div id="joystick-middle-base" class="joystick-base middle">
                        <div id="joystick-middle-stick" class="joystick-stick"></div>
                    </div>
                    <div class="text-xs mt-1 sm:mt-2 text-gray-400 text-center">LEFT/RIGHT: Roll</div>
                </div>

            </div>

        </div>
    </div>

    <script>
        document.addEventListener('DOMContentLoaded', () => {
            const gateway = `ws://${window.location.hostname}:81/`;
            let websocket;
            const statusEl = document.getElementById('status');
            
            function initWebSocket() {
                console.log('Trying to open a WebSocket connection...');
                websocket = new WebSocket(gateway);
                websocket.onopen    = onOpen;
                websocket.onclose   = onClose;
                websocket.onmessage = onMessage; 
            }

            function onOpen(event) {
                console.log('Connection opened');
                statusEl.textContent = 'Connected';
                statusEl.classList.remove('text-red-500');
                statusEl.classList.add('text-green-400');
            }

            function onClose(event) {
                console.log('Connection closed');
                statusEl.textContent = 'Disconnected';
                statusEl.classList.remove('text-green-400');
                statusEl.classList.add('text-red-500');
                setTimeout(initWebSocket, 2000);
            }

            function onMessage(event) {
                console.log('Message from server: ', event.data);
            }

            initWebSocket();

            const controlState = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
            const dataDisplay = document.getElementById('data-display');
            const valElements = {
                x: document.getElementById('val-x'),
                y: document.getElementById('val-y'),
                z: document.getElementById('val-z'),
                yaw: document.getElementById('val-yaw'),
                pitch: document.getElementById('val-pitch'),
                roll: document.getElementById('val-roll'),
            };

            function sendData() {
                const dataString = `${controlState.x.toFixed(2)},${controlState.y.toFixed(2)},${controlState.z.toFixed(2)},${controlState.yaw.toFixed(2)},${controlState.pitch.toFixed(2)},${controlState.roll.toFixed(2)}`;
                
                dataDisplay.textContent = dataString;
                for (const key in valElements) {
                    valElements[key].textContent = controlState[key].toFixed(2);
                }

                if (websocket && websocket.readyState === WebSocket.OPEN) {
                    websocket.send(dataString);
                }
            }
            
            const joysticks = [];
            
            function createJoystick(baseId, stickId, options) {
                const base = document.getElementById(baseId);
                const stick = document.getElementById(stickId);
                
                const joystick = {
                    base, stick, options, touchId: null, isDragging: false,
                    
                    handleMove(clientX, clientY) {
                        const baseRect = this.base.getBoundingClientRect();
                        const maxDist = this.base.offsetWidth / 2;
                        let diffX = clientX - baseRect.left - maxDist;
                        let diffY = clientY - baseRect.top - maxDist;
                        if (this.options.lockY) diffY = 0;
                        const dist = Math.sqrt(diffX * diffX + diffY * diffY);
                        const angle = Math.atan2(diffY, diffX);
                        let stickX, stickY;
                        if (dist > maxDist) {
                            stickX = maxDist * Math.cos(angle);
                            stickY = maxDist * Math.sin(angle);
                        } else {
                            stickX = diffX;
                            stickY = diffY;
                        }
                        this.stick.style.transform = `translate(${stickX}px, ${stickY}px)`;
                        const valueX = stickX / maxDist;
                        const valueY = stickY / maxDist;
                        this.options.onMove({ x: valueX, y: -valueY });
                    },

                    handleEnd() {
                        this.isDragging = false;
                        this.touchId = null;
                        this.stick.style.transition = 'transform 0.2s ease-out';
                        this.stick.style.transform = 'translate(0, 0)';
                        this.options.onMove({ x: 0, y: 0 });
                        setTimeout(() => this.stick.style.transition = '', 200);
                    },

                    handleStart() {
                        this.isDragging = true;
                        this.stick.style.transition = '';
                    }
                };
                
                joysticks.push(joystick);
                
                stick.addEventListener('mousedown', (e) => {
                    e.preventDefault();
                    joystick.handleStart();
                    const onMouseMove = (moveEvent) => {
                        if (joystick.isDragging) joystick.handleMove(moveEvent.clientX, moveEvent.clientY);
                    };
                    const onMouseUp = () => {
                        joystick.handleEnd();
                        document.removeEventListener('mousemove', onMouseMove);
                        document.removeEventListener('mouseup', onMouseUp);
                    };
                    document.addEventListener('mousemove', onMouseMove);
                    document.addEventListener('mouseup', onMouseUp);
                });
            }
            
            document.body.addEventListener('touchstart', (e) => {
                let shouldPreventDefault = false;
                for (const touch of e.changedTouches) {
                    for (const joystick of joysticks) {
                        const baseRect = joystick.base.getBoundingClientRect();
                        const isInside = touch.clientX >= baseRect.left && touch.clientX <= baseRect.right &&
                                         touch.clientY >= baseRect.top && touch.clientY <= baseRect.bottom;
                        
                        if (isInside && joystick.touchId === null) {
                            shouldPreventDefault = true;
                            joystick.touchId = touch.identifier;
                            joystick.handleStart();
                            joystick.handleMove(touch.clientX, touch.clientY);
                            break; 
                        }
                    }
                }
                if (shouldPreventDefault) e.preventDefault();
            }, { passive: false });

            document.body.addEventListener('touchmove', (e) => {
                let shouldPreventDefault = false;
                for (const touch of e.changedTouches) {
                    for (const joystick of joysticks) {
                        if (joystick.touchId === touch.identifier) {
                            shouldPreventDefault = true;
                            joystick.handleMove(touch.clientX, touch.clientY);
                            break;
                        }
                    }
                }
                if (shouldPreventDefault) e.preventDefault();
            }, { passive: false });

            document.body.addEventListener('touchend', (e) => {
                for (const touch of e.changedTouches) {
                    for (const joystick of joysticks) {
                        if (joystick.touchId === touch.identifier) {
                            joystick.handleEnd();
                            break;
                        }
                    }
                }
            });

            createJoystick('joystick-right-base', 'joystick-right-stick', {
                onMove: (values) => { controlState.x = values.y; controlState.z = values.x; }
            });

            createJoystick('joystick-left-base', 'joystick-left-stick', {
                onMove: (values) => { controlState.pitch = values.y; controlState.yaw = values.x; }
            });

            createJoystick('joystick-middle-base', 'joystick-middle-stick', {
                lockY: true, 
                onMove: (values) => { controlState.roll = values.x; }
            });

            const yUpBtn = document.getElementById('y-up');
            const yDownBtn = document.getElementById('y-down');

            function handleYButton(direction) {
                if (direction === 'up') {
                    controlState.y = Math.min(1.0, controlState.y + 0.2);
                } else {
                    controlState.y = Math.max(-1.0, controlState.y - 0.2);
                }
            }

            yUpBtn.addEventListener('mousedown', () => handleYButton('up'));
            yDownBtn.addEventListener('mousedown', () => handleYButton('down'));
            yUpBtn.addEventListener('touchstart', (e) => { e.preventDefault(); handleYButton('up'); });
            yDownBtn.addEventListener('touchstart', (e) => { e.preventDefault(); handleYButton('down'); });
            
            setInterval(sendData, 100);
        });
    </script>
</body>
</html>
)rawliteral";

// --- Data Parsing Function ---
// This function now only prints the raw data string.
void parseData(String data) {
    Serial.println(data);
}

// --- WebSocket Event Handler ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            break;
        }
        case WStype_TEXT:
            parseData((char*)payload);
            break;
    }
}

// --- Web Server Root Handler ---
void handleRoot() {
    server.send_P(200, "text/html", index_html);
}

// --- Initial Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("Booting...");

    Serial.print("Setting up AP: ");
    Serial.println(ssid);
    WiFi.softAP(ssid, password);

    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    server.on("/", handleRoot);
    server.begin();
    
    Serial.println("HTTP server started");
    Serial.println("WebSocket server started on port 81");
}

// --- Main Loop ---
void loop() {
    webSocket.loop();
    server.handleClient();
    delay(1); 
}