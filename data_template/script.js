var currentTimeMs=(new Date()).getTime();
var websocket;
const Http = new XMLHttpRequest();
var output;

var dps1 = [];   //dataPoints. 
var dps2 = [];   //dataPoints. 
var dps3 = [];   //dataPoints. 

var an1 = [];   //dataPoints. 
var an2 = [];   //dataPoints.

var capfast = [];   //dataPoints. 
var capslow = [];   //dataPoints.


var x1=0;
var max1=-1000000;
var min1=1000000;
var chart1;
var chart2;
var chart3;
var chart4;

function handleEnablePID1(cb)
{
	if(!cb.checked)
		doSendCommand("enablePid1");
	else
		doSendCommand("disablePid1");
}
function handleEnablePID2(cb)
{
	if(!cb.checked)
		doSendCommand("enablePid2");
	else
		doSendCommand("disablePid2");
}

function handleChartToggle()
{
	chartsVisible = document.getElementById("chartsVisible");
	divCharts = document.getElementById("charts");
	var url;
    if(!chartsVisible.checked)
    {
		url='/toggleChartsOn';
    	divCharts.style.visibility='visible';
    }
    else
    {
		url='/toggleChartsOff';
    	divCharts.style.visibility='hidden';
	}
	Http.open("GET", url);
	Http.send();
}


function handleSearchTop(cb)
{
	if(!cb.checked)
		doSendCommand("searchtop start");
	else
		doSendCommand("searchtop stop");
}

function handleSearchBottom(cb)
{
	if(!cb.checked)
		doSendCommand("searchbottom start");
	else
		doSendCommand("searchbottom stop");
}

function keyPress(e)
{
	// look for window.event in case event isn't passed in
	e = e || window.event;
	if (e.keyCode == 13)
	{
		doSendCommand(e.srcElement.id + "#" + e.srcElement.value);
		return false;
	}
	return true;
}		

function init()
{
	output = parent.document.getElementById("output");
	testWebSocket();
	

	if (!!window.EventSource) {
	  var source = new EventSource('/events');

	  source.addEventListener('open', function(e) {
	    console.log("Events Connected");
	  }, false);

	  source.addEventListener('error', function(e) {
	    if (e.target.readyState != EventSource.OPEN) {
	      console.log("Events Disconnected");
	    }
	  }, false);

	  source.addEventListener('message', function(e) {
	    console.log("message", e.data);
	  }, false);

	  source.addEventListener('myevent', function(e) {
	    //console.log("myevent", e.data);
        onMessage(e);
	  }, false);
	}

}

function testWebSocket()
{
	if (location.host != "")
	{
		//var wsUri = "ws://" + location.host + "/ws";
		var wsUri = "ws://" + location.host.split(":")[0] + ":81/ws";
		websocket = new WebSocket(wsUri);
		websocket.onopen = function(evt) { onOpen(evt) };
		websocket.onclose = function(evt) { onClose(evt) };
		websocket.onmessage = function(evt) { onMessage(evt) };
		websocket.onerror = function(evt) { onError(evt) };		
	}
}
function onOpen(evt)
{
	writeToScreen("CONNECTED");
}
function onClose(evt)
{
	writeToScreen("DISCONNECTED");
	//writeToScreen("RECONNECT...");
	//testWebSocket();
}

function onMessage(evt)
{
	if(evt.data.startsWith("{"))
		{
			var obj = JSON.parse(evt.data);
			//document.getElementById("encoder1_value").innerHTML = obj.encoder1_value;
			//document.getElementById("encoder2_value").innerHTML = obj.encoder2_value;
			//document.getElementById("esp32_heap").innerHTML = obj.esp32_heap;
			
			for(var propertyName in obj) {
			   // propertyName is what you want
			   // you can get the value like this: myObject[propertyName]
			   if(document.getElementById(propertyName) != null)
			   {
			   		if(document.getElementById(propertyName).nodeName == "INPUT" && document.getElementById(propertyName).nodeType == 0)
			   			document.getElementById(propertyName).value = obj[propertyName];
					else if(document.getElementById(propertyName).nodeName == "INPUT" && document.getElementById(propertyName).nodeType == 1){
							//checkbox
							if(obj[propertyName] == 1)
								document.getElementById(propertyName).checked = true;
							else
								document.getElementById(propertyName).checked = false;
						}
			   		else	
						document.getElementById(propertyName).innerHTML = obj[propertyName];
			   }
			   
			   // chart data
			}
			drawChart(obj);
		}
	else
	{
		var allData = evt.data.split('\n');
		//writeToScreen('<span style="color: blue;">Time: ' + ((new Date()).getTime()-currentTimeMs) +'ms Received:</span>');
		for(i=0; i<allData.length; i++)
		{
			if(allData[i].startsWith("wifi ", 0))
			{
				writeToScreen("<span id='wifi" + i +"'>" + allData[i].split(' ')[1] + "</span><input id='pass" + i + "' width='300px'></input><button onclick='wifiConnect(" + i + ")'>Connect</button>");
			}
			else
			{
				writeToScreen('<span style="color: blue;">'+allData[i]+'</span>');
			}
		}
		var res = evt.data.split(' ');
		if (res.length==2 && res[0] == 'motor1_pos')
		{
			document.getElementById("motor1_pos").textContent = res[1];
		}
		if (res.length==2 && res[0] == 'motor2_pos')
		{		
			document.getElementById("motor2_pos").textContent = res[1];
		}
	}
	websocket.send('ok');
}

function onError(evt)
{
	writeToScreen('<span style="color: red;">ERROR:</span> ' + evt.data);
}
function wifiConnect(i)
{
	textToSend = "wificonnect " + document.getElementById("wifi" + i).textContent + " " + document.getElementById("pass" + i).value;
	websocket.send(textToSend);
}
function doSend(element)
{
	//writeToScreen("SENT: " + message); 
	textToSend = element.id + "#" + element.value;
	if(element.id == 'gCodeCmd1' || element.id == 'gCodeCmd2')
	{
		const url= '/gcode?gcode=' + encodeURIComponent(textToSend);
		Http.open("GET", url);
		Http.send();
	}
	else
		websocket.send(textToSend);
}

function moveTo(element)
{
	textToSend = element.id.split("_")[0] + "=" + element.value;
	const url= '/' + element.id.split("_")[0] + '?' + textToSend;
	Http.open("GET", url);
	Http.send();
}



function doSendParentElementId(element)
{

if(websocket!=null)
	websocket.send(element.id);
}
function doSendCommand(textToSend)
{

if(websocket!=null)
	websocket.send(textToSend);
}

function doSendCommand2(textToSend, element)
{

if(websocket!=null)
	websocket.send(textToSend + " " + element.id);
}

function writeToScreen(message)
{
	output.innerHTML = 'Time: ' + (((new Date()).getTime()-currentTimeMs)/1000) +'s Received: ' + message + "<br>\n" + output.innerHTML;		
}
function doDisconnect()
{
	var disconnect = document.getElementById("disconnect");
	disconnect.disabled = true;
	websocket.close();
}

function jogClick(a)
{
	writeToScreen("SENT: " + a); 
	doSendCommand(a);		
}

function endsWith(str, suffix) {
	return str.indexOf(suffix, str.length - suffix.length) !== -1;
}

function drawChart(obj)
{
	if(chart1 != null && chart2 != null)
	{
		x1 = x1+1;
		if(obj.hasOwnProperty("encoder1_value"))
		{
			//config.data.labels.push(newDate(config.data.labels.length));
			dps1.push({x: x1,y: parseFloat(obj.encoder1_value)});
			dps2.push({x: x1,y: parseFloat(obj.encoder2_value)});
			dps3.push({x: x1,y: parseFloat(obj.actual_diff)});

			if (dps1.length >  100 )
			{
				dps1.shift();					
			}
			if (dps2.length >  100 )
			{
				dps2.shift();					
			}
			if (dps3.length >  100 )
			{
				dps3.shift();					
			}
			
			var max_01 = maxValue(chart1.options.data[0].dataPoints);
			var max_02 = maxValue(chart1.options.data[1].dataPoints);
			chart1.options.axisY.maximum = Math.max(max_01, max_02)+20;
			
			var min_01 = minValue(chart1.options.data[0].dataPoints);
			var min_02 = minValue(chart1.options.data[1].dataPoints);
			chart1.options.axisY.minimum = Math.min(min_01, min_02)-20;

			chart4.options.axisY.maximum = maxValue(chart4.options.data[0].dataPoints)+5;
			chart4.options.axisY.minimum = minValue(chart4.options.data[0].dataPoints)-5;
			chart4.render();
			chart1.render();
		}

		if(obj.hasOwnProperty("an1"))
		{
			//config.data.labels.push(newDate(config.data.labels.length));
			an1.push({x: x1,y: parseFloat(obj.an1)});
			an2.push({x: x1,y: parseFloat(obj.an2)});

			if (an1.length >  100 )
			{
				an1.shift();					
			}
			if (an2.length >  100 )
			{
				an2.shift();					
			}
			
			var max_01 = maxValue(chart2.options.data[0].dataPoints);
			var max_02 = maxValue(chart2.options.data[1].dataPoints);
			chart2.options.axisY.maximum = Math.max(max_01, max_02);
			
			var min_01 = minValue(chart2.options.data[0].dataPoints);
			var min_02 = minValue(chart2.options.data[1].dataPoints);
			
			chart2.options.axisY.minimum =Math.min(min_01, min_02)-5;
			chart2.render();
		}
		
		if(obj.hasOwnProperty("capfast"))
		{
			//config.data.labels.push(newDate(config.data.labels.length));
			capfast.push({x: x1,y: parseFloat(obj.capfast)});
			capslow.push({x: x1,y: parseFloat(obj.capslow)});

			if (capfast.length >  100 )
			{
				capfast.shift();					
			}
			if (capslow.length >  100 )
			{
				capslow.shift();					
			}
			
			var max_01 = maxValue(chart3.options.data[0].dataPoints);
			var max_02 = maxValue(chart3.options.data[1].dataPoints);
			chart3.options.axisY.maximum = Math.max(max_01, max_02);
			
			var min_01 = minValue(chart3.options.data[0].dataPoints);
			var min_02 = minValue(chart3.options.data[1].dataPoints);
			
			chart3.options.axisY.minimum =Math.min(min_01, min_02);
			chart3.render();
		}		
	}
}

function now() {
	return moment().toDate();
}

var maxValue = function(dataPoints) {
	var max1 = dataPoints[0].y;
	for (i = 0; i < dataPoints.length; i++) {
		if (dataPoints[i].y > max1) {
		  max1 = dataPoints[i].y;
		}
	}
	//return Math.round(maximum);
	return Math.ceil(max1/10)*10;
};


var minValue = function(dataPoints) {
	var min1 = dataPoints[0].y;
	for (i = 0; i < dataPoints.length; i++) {
		if (dataPoints[i].y < min1) {
		  min1 = dataPoints[i].y;
		}
	}
	//return Math.round(maximum);
	return Math.ceil(min1/10)*10;
};  



window.addEventListener("load", init, false);

