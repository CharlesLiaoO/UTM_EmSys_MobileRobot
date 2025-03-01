
/// <reference types="plotly.js" />

// let updateInterval = setInterval(test, 100);

let xData = [];
let xDataList = [];
let yDataList = [];

// layout config
let maxPoints = 100;
let maxVisiblePoints = 50;
let xVisibleRange = [0, maxVisiblePoints];

let layout = {
    title: {  // title of graph
        text: "Pid Parameters",
        automargin: true,
        font: { family: "Arial"},  //default one lead to cutting of edge
    },
    xaxis: {
        // title: "",
        showgrid: false,
        type: "linear",
        range: xVisibleRange
    },
    yaxis: {
        title: { text: "setpoint, feedback"},  // cannot set title text horizontal easily
        showgrid: true,
        range: [0, 1],  // [0, null] can auto adjust upper bound, but 0 is not at same pos with 0 of y2
    },
    yaxis2: {
        title: { text: "output"},
        showgrid: false,
        overlaying: "y",  // overlay 'y' axis, otherwise override
        side: "right",
        range: [0, 255]
    },
    legend: {
        orientation: "h",
        xanchor: "center",
        yanchor: "bottom",
        x: 0.5,
        y: 1.05
    }
};

// ref: https://plotly.com/javascript/reference/scatter/#scatter-line-dash
let datasetCfgs = [
    { name: "1_setpoint",   yAxis: "y1",    dash: "solid"   },
    { name: "1_feedback",   yAxis: "y1",    dash: "solid"   },
    { name: "1_output",     yAxis: "y2",    dash: "dash"    },
    { name: "2_setpoint",   yAxis: "y1",    dash: "solid"   },
    { name: "2_feedback",   yAxis: "y1",    dash: "solid"   },
    { name: "2_output",     yAxis: "y2",    dash: "dash"    },
]

let traces = [];
for (var i=0; i<6; i++) {
    traces.push({
        x: xData,
        y: yDataList[i],
        mode: "lines",
        name: datasetCfgs[i].name,
        line: {
            // color: "red",
            // width: 2,
            dash: datasetCfgs[i].dash,
        },
        yaxis: datasetCfgs[i].yAxis,
    })
    xDataList.push(xData)
    yDataList.push([])
}

Plotly.newPlot("motorChart", traces, layout, { responsive: true });

let xIdx = 0;
function plotly_updateData(newYData) {
    xData.push(xIdx);
    for (var i=0; i<6; i++) {
        yDataList[i].push(newYData[i])
    }

    if (xData.length > maxVisiblePoints) {
        layout.xaxis.range = [xIdx - 50, xIdx];
    }
    if (xData.length > maxPoints) {
        xData.shift();
        for (var i=0; i<6; i++) {
            yDataList[i].shift()
        }
    }
    xIdx++;
    Plotly.update("motorChart",
        { x: xDataList, y: yDataList },
        layout,
    );
}

let testTimes = 0;
function test() {
    testTimes++
    if (testTimes > maxPoints) {
        clearInterval(updateInterval);
        return;
    }
    plotly_updateData([
        0.8 * layout.yaxis.range[1],
        Math.random() * layout.yaxis.range[1],
        Math.random() * layout.yaxis2.range[1],
        0.79 * layout.yaxis.range[1],
        Math.random() * layout.yaxis.range[1],
        Math.random() * layout.yaxis2.range[1],
    ])
}
