
const eventSource = new EventSource("/events");
eventSource.onmessage = function(event) {
    // document.getElementById('debugMsg').innerText = event.data;
    data = JSON.parse(event.data)
    for (varName in data) {
        document.getElementById(varName).innerText = data[varName].toFixed(3);
    }
};

function btnDeal(btn)
{
    cmd = ""
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
