// @ts-check - enable TypeScript check

var chartTimeIdx = 0;
const eventSource = new EventSource("/events");
eventSource.onmessage = function(event) {
    /** @type string */
    var dataStr = event.data;
    // document.getElementById('debugMsg').innerText = dataStr;
    if (dataStr.startsWith("<")) {
        dataStr = dataStr.slice(1)
        plotly_updateData(JSON.parse(dataStr))
    } else {
        var data = JSON.parse(event.data)
        for (var dName in data) {
            var elm = document.getElementById(dName)
            if (elm) elm.innerText = data[dName].toFixed(3);
        }
    }
}

// document.addEventListener("DOMContentLoaded", function () {  // only html loaded
window.onload = function () {  // all html/css/image... loaded
};

function btnDeal(btn)
{
    var cmd = ""
    switch (btn) {
        case '0': cmd = "ms,0,0"; break;
        case 'u': cmd = "ms,150,150"; break;
        case 'd': cmd = "ms,-150,-150"; break;
        case 'l': cmd = "ms,-75,75"; break;
        case 'r': cmd = "ms,75,-75"; break;
    }

    fetch("/cmd", {
        method: 'POST',
        headers: {
          'Content-Type': 'text/plain',
        },
        body: cmd,
    })
    .then((response) => {
        if (!response.ok) {
            // document.getElementById('resp').innerText = "response";
            throw new Error(`HTTP error! Status: ${response.status}`);
        }
    })
    .catch((error) => {
        alert(error);
    });
}
