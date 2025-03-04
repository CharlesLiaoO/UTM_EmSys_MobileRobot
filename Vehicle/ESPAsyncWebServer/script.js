// WebSocket connection
const ws = new WebSocket(`ws://${window.location.hostname}/ws`);

ws.onopen = () => console.log("WebSocket Connected!");
ws.onmessage = (event) => {
    console.log("WebSocket Message:", event.data);
    document.getElementById("status").innerText = "WebSocket: " + event.data;
};

// Send command to ESP32 via WebSocket
function sendCommand(command) {
    ws.send(command);
}

// SSE (Server-Sent Events) connection
const sse = new EventSource("/events");

sse.onmessage = (event) => {
    console.log("SSE Event:", event.data);
    document.getElementById("status").innerText = "SSE: " + event.data;
};

// Send HTTP GET request
function sendGET() {
    fetch('/get')
        .then(response => response.text())
        .then(data => {
            console.log("GET Response:", data);
            document.getElementById("status").innerText = "GET: " + data;
        });
}

// Send HTTP POST request
function sendPOST() {
    fetch('/post', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'message=HelloESP32'
    })
    .then(response => response.text())
    .then(data => {
        console.log("POST Response:", data);
        document.getElementById("status").innerText = "POST: " + data;
    });
}
