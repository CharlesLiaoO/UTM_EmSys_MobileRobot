// @ts-check - enable TypeScript check
/// <reference path="pidPlot.js" />
/// <reference path="joy.min.js" />

function elmByName(name) {
    return document.getElementsByName(name)[0];
}

// document.addEventListener("DOMContentLoaded", function () {  // only html loaded
window.onload = function () {  // all html/css/image... loaded
    logContainer = document.getElementById("logContainer");

    new JoyStick('joystick', {}, function(d) {
        MovCmd(d.cardinalDirection);
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

    elmByName("pidApplyBtn").onclick = PidCmd;
};

function getLogHeader(){
    const now = new Date();
    const hours = now.getHours().toString().padStart(2,'0');
    const minutes = now.getMinutes().toString().padStart(2,'0');
    const sec = now.getSeconds().toString().padStart(2,'0');
    return `> ${hours}:${minutes}:${sec}    `;
}
let logContainer;
function addLog(message, level="info") {
    const logEntry = document.createElement("div");
    logEntry.textContent = getLogHeader() + message;
    logEntry.style.color = level === "error" ? "red" : level === "warn" ? "orange" : "black";
    logEntry.style.whiteSpace = "pre-wrap";

    logContainer.appendChild(logEntry);
    logContainer.scrollTop = logContainer.scrollHeight;
}

var chartTimeIdx = 0;
const eventSource = new EventSource("/events");
eventSource.onmessage = function(event) {
    /** @type string */
    var dataStr = event.data;
    if (dataStr.length == 0) return;  // why empty?
    // document.getElementById('debugMsg').innerText = dataStr;
    if (dataStr.startsWith("<")) {
        dataStr = dataStr.slice(1)
        plotly_updateData(JSON.parse(dataStr))
    } else if (dataStr.startsWith("N")) {
        dataStr = dataStr.slice(1)
        let obj = JSON.parse(dataStr)
        for (let dName in obj) {
            let elm = elmByName(dName)
            if (elm) elm.value  = obj[dName];
        }
    } else if (dataStr.startsWith("{")) {
        let obj = JSON.parse(dataStr)
        for (let dName in obj) {
            let elm = document.getElementById(dName)
            if (elm) elm.innerText = obj[dName].toFixed(3);
        }
    } else {
        addLog(dataStr)
    }
}

function PidCmd() {
    var cmd = "pid=" +
    elmByName("pid1_p").value + "," +
    elmByName("pid1_i").value + "," +
    elmByName("pid1_d").value + ";" +
    elmByName("pid2_p").value + "," +
    elmByName("pid2_i").value + "," +
    elmByName("pid2_d").value;
    cmdDeal(cmd);
}

let velSels = {};
let btnBf = "";
function MovCmd(dir)
{
    if (btnBf == dir) return;
    btnBf = dir;

    var cmd = ""
    var vs = velSels['s'].value
    var vt = velSels['t'].value
    var vr = velSels['r'].value
    // var vt = eTurnVel.options[eTurnVel.selectedIndex].text
    switch (dir) {
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
    cmdDeal(cmd);
}

function cmdDeal(cmd) {
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
