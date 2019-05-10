var express = require('express')
var path = require('path');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var bodyParser = require('body-parser');

// parse application/x-www-form-urlencoded
app.use(bodyParser.urlencoded())
app.use(express.static(__dirname + '/src'));

app.get('/', function(req, res){
  res.sendFile(express.static(__dirname + '/src/index.html'));
});

app.post("/data", function(req, res) {
  io.sockets.emit("data", req.body);
  console.log(req.body)
  res.send({});
});

io.on('connection', function(socket){
  console.log('a user connected');
});


http.listen(5000, function(){
  console.log('listening on *:5000');
});
