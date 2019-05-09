var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var bodyParser = require('body-parser')

// parse application/x-www-form-urlencoded
app.use(bodyParser.urlencoded())

app.get('/', function(req, res){
  res.sendFile(__dirname + '/src/index.html');
});

app.post("/data", function(req, res) {
  io.sockets.emit("data", req.body);
  console.log(req.body)
  res.send({});
});

io.on('connection', function(socket){
  console.log('a user connected');

  // socket.on('custom-event', function(value){
  //   io.emit('custom-event', value);
  // });
});


http.listen(5000, function(){
  console.log('listening on *:5000');
});
