var mode = 0,
    numValues = 6,
    debug = 0;
var sensorValues = [];
var graphs = [];
var json;

var wsUri = "ws://localhost:5600/";
var output;
var websocket;

function dec2bin(dec){
    return (dec >>> 0).toString(2);
}



function sleep(milliseconds) {
  var start = new Date().getTime();
  for (var i = 0; i < 1e7; i++) {
    if ((new Date().getTime() - start) > milliseconds){
      break;
    }
  }
}

function formFloat(intPartH, intPartL, decPartL) {
    hbyte = dec2bin(parseInt(intPartH, 10));
    lbyte = dec2bin(parseInt(intPartL, 10));
    deg = (hbyte << 8) | parseInt(lbyte, 2);
    decimPart = parseInt(decPartL, 10);
    //alert(deg);
    if ((deg >= 0) && (deg <= 359)) {
        float = deg + (decimPart*0.01);
    }
    return float;
}

function Graph() {
    var line = {};
    var x = {};
    var paths = [];
    var inputValues = [];
    var colors = [];
    this.setX = function( newX ) {
        x = newX;
    };

    this.getX = function() {
        return x;
    };
    this.setLine = function( newLine ) {
        line = newLine;
    };

    this.getLine = function() {
        return line;
    };
    this.setPaths = function( newPaths ) {
        paths = newPaths;
    };

    this.getPaths = function() {
        return paths;
    };
    this.setInputValues = function( newValues ) {
        inputValues = newValues;
    };

    this.getInputValues = function() {
        return inputValues;
    };
    this.setColors = function( newColors ) {
        colors = newColors;
    };

    this.getColors = function() {
        return colors;
    };
}

graphs.push(addGraph(new Graph(), 7, 300, ["green", "red", "blue", "brown", "purple", "grey", "orange"], "graphSun"));
//alert(sunGraph.getPaths());

//tick();

function addGraph(graph, numLines, height, colors, idDiv) {
    var n = 60;
    var graphs = [], paths = [];
    var margin = {top: 20, right: 20, bottom: 20, left: 40},
        width = document.getElementById(idDiv).clientWidth - margin.left - margin.right,
        height = height - margin.top - margin.bottom;
    var x = d3.scale.linear()
        .domain([0, n - 1])
        .range([0, width]);
    var y = d3.scale.linear()
        .domain([0, 360])
        .range([height, 0]);
    var line = d3.svg.line()
        .x(function(d, i) { return x(i); })
        .y(function(d, i) { return y(d); });
    var svg = d3.select("#" + idDiv).append("svg")
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
      .append("g")
        .attr("transform", "translate(" + margin.left + "," + margin.top + ")");
    function addPath(data) {
        return svg.append("g")
        .attr("clip-path", "url(#clip)")
      .append("path")
        .datum(data)
        .attr("class", "line")
        .attr("d", line);
    }

    svg.append("defs").append("clipPath")
        .attr("id", "clip")
      .append("rect")
        .attr("width", width)
        .attr("height", height);
    svg.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0," + y(0) + ")")
        .call(d3.svg.axis().scale(x).orient("bottom"));
    svg.append("g")
        .attr("class", "y axis")
        .call(d3.svg.axis().scale(y).orient("left"));

    for (var i = 0; i < numLines; i++) {
        graphs[i] = Array.apply(null, new Array(n)).map(Number.prototype.valueOf,0);
        paths[i] = addPath(graphs[i]);
    }
    graph.setPaths(paths);
    graph.setInputValues(graphs);
    graph.setX(x);
    graph.setLine(line);
    graph.setColors(colors);

    return graph;
}


function redrawPath(path, line, x, color) {
 return path
      .attr("d", line)
      .attr("transform", null)
    .transition()
      .duration(500)
      .ease("linear")
      .attr("stroke", color)
      .attr("transform", "translate(" + x(-1) + ",0)");
}

function tick() {
  var line, x, sensorKey = 0;
  // push a new data point onto the back
  for (var i in graphs) {
      line = graphs[i].getLine();
      x = graphs[i].getX();
      for (var key in graphs[i].getInputValues()) {
        (graphs[i].getInputValues()[key]).push(sensorValues[sensorKey]);
        if ((key == graphs[i].getInputValues().length - 1) &&
            (i == graphs.length - 1)) {
            redrawPath(graphs[i].getPaths()[key], line, x, graphs[i].getColors()[key]).each("end", tick);
        } else {
            redrawPath(graphs[i].getPaths()[key], line, x, graphs[i].getColors()[key]);
        }
        (graphs[i].getInputValues()[key]).shift();
        sensorKey++;
      }

  }
}



function init()
{
    output = document.getElementById("output");
    testWebSocket();
}
function testWebSocket()
{
    websocket = new WebSocket(wsUri);
    websocket.onopen = function(evt) { onOpen(evt) };
    websocket.onclose = function(evt) { onClose(evt) };
    websocket.onmessage = function(evt) { onMessage(evt) };
    websocket.onerror = function(evt) { onError(evt) };
}
function onOpen(evt)
{
    writeToScreen("CONNECTED");
    doSend("WebSocket rocks");
}
function onClose(evt)
{
    writeToScreen("DISCONNECTED");
}
function onMessage(evt)
{
    writeToScreen('<span style="color: blue;">RESPONSE: ' + evt.data+'</span>');
    websocket.close();
}
function onError(evt)
{
    writeToScreen('<span style="color: red;">ERROR:</span> ' + evt.data);
}
function doSend(message)
{
    writeToScreen("SENT: " + message);
    websocket.send(message);
}
function writeToScreen(message)
{
    var pre = document.createElement("p");
    pre.style.wordWrap = "break-word";
    pre.innerHTML = message;
    output.appendChild(pre);
}

window.addEventListener("load", init, false);