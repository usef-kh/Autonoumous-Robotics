var socket = io.connect('http://localhost:5000');
let data;

socket.on('data', function (data) {
    data = data;
    console.log(data)
});

function setup() {
    createCanvas(windowWidth, windowHeight);
}

function draw() {
    background(255);
    text("Hello",10, frameCount % height);
}

function windowResized() {
    resizeCanvas(windowWidth, windowHeight);
}