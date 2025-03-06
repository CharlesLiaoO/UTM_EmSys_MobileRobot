// @ts-check - enable TypeScript check
/// <reference path="pidPlot.js" />
/// <reference path="joy.min.js" />

// document.addEventListener("DOMContentLoaded", function () {  // only html loaded
window.onload = function () {  // all html/css/image... loaded
    logContainer = document.getElementById("logContainer");

    new JoyStick('joystick', {}, function(d) {
        btnDeal(d.cardinalDirection);
    });

    velSels['s'] = document.getElementsByName("Straight")[0];
    velSels['t'] = document.getElementsByName("Turn")[0];
    velSels['r'] = document.getElementsByName("Rotate")[0];
    Object.values(velSels).forEach(velS => {
        while (velS.options.length > 0) {
            velS.remove(0); }  // clear
        for (var i=25; i<=255; i+=25) {
            velS.add(new Option(`${i}`, `${i}`))
        }
    })
    velSels['s'].value = 150;
    velSels['t'].value = 100;
    velSels['r'].value = 75;
};

let logContainer;
function addLog(message, level="info") {
    const logEntry = document.createElement("div");
    logEntry.textContent = message;
    logEntry.style.color = level === "error" ? "red" : level === "warn" ? "orange" : "black";

    logContainer.appendChild(logEntry);
    logContainer.scrollTop = logContainer.scrollHeight;
}

var chartTimeIdx = 0;
const eventSource = new EventSource("/events");
eventSource.onmessage = function(event) {
    /** @type string */
    var dataStr = event.data;
    // document.getElementById('debugMsg').innerText = dataStr;
    if (dataStr.startsWith("<")) {
        dataStr = dataStr.slice(1)
        plotly_updateData(JSON.parse(dataStr))
    } else if (dataStr.startsWith("{")) {
        var data = JSON.parse(dataStr)
        for (var dName in data) {
            var elm = document.getElementById(dName)
            if (elm) elm.innerText = data[dName].toFixed(3);
        }
    } else {
        addLog(dataStr)
    }
}

let velSels = {};
function btnDeal(btn)
{
    var cmd = ""
    var vs = velSels['s'].value
    var vt = velSels['t'].value
    var vr = velSels['r'].value
    // var vt = eTurnVel.options[eTurnVel.selectedIndex].text
    switch (btn) {
        case 'C': cmd = "ms=0,0"; break;
        case 'N': cmd = `ms=${vs},${vs}`; break;
        case 'S': cmd = `ms=-${vs},-${vs}`; break;
        case 'W': cmd = `ms=-${vr},${vr}`; break;
        case 'E': cmd = `ms=${vr},-${vr}`; break;
        case 'NW': cmd = `ms=${vt},${vs}`; break;
        case 'NE': cmd = `ms=${vs},${vt}`; break;
        case 'SW': cmd = `ms=-${vt},-${vs}`; break;
        case 'SE': cmd = `ms=-${vs},-${vt}`; break;
    }

    fetch("/cmd", {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: cmd,
    })
    .then((response) => {
        if (!response.ok) {
            // addLog(response.statusText, "error");
            throw new Error(`HTTP error! Status: ${response.status}`);
        }
    })
    .catch((error) => {
        alert(error);
    });
}
