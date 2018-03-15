var serialport = require('serialport');//require serialport dependence
var WebSocketServer = require('ws').Server;//require websocket library

// configure the webSocket server:
var SERVER_PORT = 8081;//Set port number:8081
var wss = new WebSocketServer({port: SERVER_PORT});//create a new websocket server with the server port 8081
var connections = new Array;//create an array  to support every connection in the server
//configure the serial port to operate at 115200 bps in the specified connection as second parameter
SerialPort = serialport.SerialPort,
    portName = process.argv[2];
var serialOptions = {
      baudRate: 115200,
      parser: serialport.parsers.readline('\r')//read until carry return
    };

//fail message for undefined port
if (typeof portName === "undefined") {
  console.log("Specify the port name\n");
  process.exit(1);
}

//create the serial port with the serial options defined earlier
var myPort = new SerialPort(portName, serialOptions);

//Establish serial events in order to execute functions. The serial events are: open port,data reception,close port, error at reading port
myPort.on('open', showPortOpen);
myPort.on('data', sendSerialData);
myPort.on('close', showPortClose);
myPort.on('error', showError);

//function for open port
function showPortOpen() {
  console.log('Baudrate:' + myPort.options.baudRate);//print the baudrate
}

//function to send serial data  to the webserver
function sendSerialData(data) {

  console.log(data);
  if (connections.length > 0) {
    broadcast(data);//broadcast or flood all web server connections. In this case, if port 8081 is the only one that is open, then this port receives the information
  }
}
//function for closed port
function showPortClose() {
   console.log('The port was closed');//close port message
}
//Function for port communication error
function showError(error) {
  console.log('REPORT CLOSE PORT:' + error);//close port error message
}
//function for sending information to serial port, not essential if the project  relies on data reception from the microcontroller
function sendToSerial(data) {
  console.log("sending to serial: " + data);//transmission to serial message

}
//function for multiple clients in the server
wss.on('connection', handleConnection);

function handleConnection(client) {

  console.log("New Connection"); 
  //push into conections array the new client
  connections.push(client); 
  //transmit to serial that a new connection has just been made
  client.on('message', sendToSerial);

  client.on('close', function() {//In cas of a closing event:
    console.log("connection closed");//message for client connection closed
    var position = connections.indexOf(client); //search for the client in the connection list and delete it
    connections.splice(position, 1);
  });
}
//broadcast function to flood all the connections
function broadcast(data) {
  for (c in connections) {
    connections[c].send(JSON.stringify(data));
  }
}