// @ts-check - enable TypeScript check
/// <reference path="../dev_webpage/Chart.d.ts" />

var chartTimeIdx = 0;
const eventSource = new EventSource("/events");
eventSource.onmessage = function(event) {
    /** @type string */
    var dataStr = event.data;
    // document.getElementById('debugMsg').innerText = dataStr;
    if (dataStr.startsWith("<")) {
        dataStr = dataStr.slice(1)
        updateChartByJson(JSON.parse(dataStr))
    } else {
        var data = JSON.parse(event.data)
        for (var dName in data) {
            var elm = document.getElementById(dName)
            if (elm) elm.innerText = data[dName].toFixed(3);
        }
    }
}

// -- test value
// const maxPoints = 50;
// const maxVisiblePoints = 10;
// -- practical value
const maxPoints = 50;
const maxVisiblePoints = 50;

// Note1: options of Chart.ChartConfiguration is copied value as new Chart(), while data of Chart.ChartConfiguration is not!
/** @type Chart.ChartData */
const chartData = {};
/** @type Chart.ChartConfiguration */
const config = {
    type: 'line',
    data: chartData,
    options: {
        responsive: true,
        // animation: { duration: 10 } // default is 1000 ms
        animation: false,
        // cubicInterpolationMode: 'monotone',
        scales: {
           // @ts-ignore
            x: {
                type: 'linear',  // line point intv
                min: 0,
                max: maxVisiblePoints - 1,
                ticks: {
                    stepSize: 1,  // preferred label intv
                },
            },
            y: {
                title: { display: true, text: "setpoint, feedback" },
                beginAtZero: true,  // y label starts with 0
            },
            y1: {
                title: { display: true, text: "output" },
                suggestedMin: 0,
                suggestedMax: 250,
                position: 'right',
                grid: {
                  drawOnChartArea: false, // only want the grid lines for one axis to show up
                }
            }
        },
        // plugins: {
        //     legend: {
        //         labels: {
        //             usePointStyle: true,  // no useLineStyle... Move to Plotly.js!
        //         }
        //     },
        // },
    },
};

var chartContext;
var lineChart;
var colors = ["#36a2eb", "#ff6384", "#4bc0c0", "#ff9f40", "#9966ff", "#ffcd56", "#c9cbcf"];  // ref: https://www.chartjs.org/docs/latest/general/colors.html#default-color-palette
// document.addEventListener("DOMContentLoaded", function () {  // only html loaded
window.onload = function () {  // all html/css/image... loaded
    var motorChart = /** @type {HTMLCanvasElement} */ (document.getElementById("motorChart"));
    chartContext = /** @type {CanvasRenderingContext2D} */ (motorChart.getContext("2d"));
    lineChart = new Chart(chartContext, config);

    // Register the Colors plugin (if using Chart.js 3.9+)
    // Chart.register(Chart.Colors);  // cannot find a Chart.Colors.js on web, usually they use CDN of Chart.js
};

// test
var testIdx = 0;
// var intvId = setInterval(testChart, 100);  // When the update time is greater than or close to the animation duration, the drawing may be stuck (perhaps interrupting the animation)
function testChart() {
    // if (dIdx > maxPoints + maxVisiblePoints) {
    if (testIdx > maxPoints) {
    // if (testIdx > maxVisiblePoints) {
        clearInterval(intvId);
        return;
    }
    testIdx++

    const newValue = Number((Math.random() * 10).toFixed(0));

    updateChartByJson({"ds1": newValue, "ds2": newValue+2})
}

var chartXIdx = 0;
var dataSetNameList = []
var dataSetYAxisList = ["y", "y", "y1", "y", "y", "y1"]
// ref: https://developer.mozilla.org/en-US/docs/Web/API/CanvasRenderingContext2D/setLineDash
// var dataSetLineStyleList = [[], [], [10,10], [], [], [10,10]]  // when graph move, the dashed line offset move... use pointStyle
// var dataSetPointStyleList = [false, false, "circle", false, false, "circle"]  // when use pointStyle, legend icon disappear...
function updateChartByJson(obj) {
    chartData.labels.push(chartXIdx);
    for (var dataName in obj ) {
        var dataSetIdx = dataSetNameList.indexOf(dataName);
        if (dataSetIdx == -1) {
            dataSetIdx = dataSetNameList.length;
            dataSetNameList.push(dataName);
            chartData.datasets?.push({
                label: dataName,
                backgroundColor: colors[dataSetIdx],
                borderColor: colors[dataSetIdx],
                yAxisID: dataSetYAxisList[dataSetIdx],
                pointStyle: false,
                // pointStyle: dataSetPointStyleList[dataSetIdx],
                // borderDash: dataSetLineStyleList[dataSetIdx],
                data: [],
            })
        }

        chartData.datasets[dataSetIdx].data?.push(obj[dataName]);
    }
    chartXIdx++;

    if (chartXIdx > maxVisiblePoints) {
        lineChart.options.scales.x.min = chartXIdx - maxVisiblePoints;
        lineChart.options.scales.x.max = chartXIdx - 1;
    }
    if (chartData.labels.length > maxPoints) {
        chartData.labels?.shift();
        for (var i=0; i < chartData.datasets.length; i++) {
            chartData.datasets[i].data.shift();
        }
    }

    lineChart.update();
}

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
