var socket = io.connect('http://localhost:5000');

class Data{
    constructor(width, height, id, timestep) {
        this.height = height;
        this.width = width;
        this.name = id
        this.data = [{x: 0, y: 0}];
        this.graph = this.makeGraph();
        this.x = this.makeXAxis();
        this.y = this.makeYAxis();
        this.created = this.createGraph()
        this.timestep = timestep;
    }

    extractValues(d){
        this.data.push({x: this.data.length * this.timestep, y: parseFloat(d[`${this.name}`])})
        this.updateGraph()
    }

    makeGraph(){
        var graph = new Rickshaw.Graph( {
            element: document.getElementById(this.name), 
            renderer: 'line',
            width: this.width,
            height: this.height,
            min: 'auto',
            series: [{
                color: 'steelblue',
                data: this.data
            }]
        });
        return graph
    }

    makeXAxis(){
        var xAxis = new Rickshaw.Graph.Axis.X({
            graph: this.graph
        });
        xAxis.render()
        return xAxis;
    }

    makeYAxis(){
        var yAxis = new Rickshaw.Graph.Axis.Y({
            graph: this.graph
        });
        yAxis.render()
        return yAxis;
    }

    createGraph(){
        this.graph.render()
        this.x.render()
        this.y.render()
        return true
    }

    updateGraph(){
        this.graph.update();
    }
}


// ts = new Data(400, 240, 'TS');
x = new Data(400, 240, 'x');
y = new Data(400, 240, 'y');
vl = new Data(400, 240, 'vl');
vr = new Data(400, 240, 'vr');
v1 = new Data(400, 240, 'V1');
v2 = new Data(400, 240, 'V2');
v = new Data(400, 240, 'V Actual');
ax = new Data(400, 240, 'AX');
ay = new Data(400, 240, 'AY');
az = new Data(400, 240, 'AZ');

socket.on('data', function (data) {
    let ts = 
    x.extractValues(data)
    y.extractValues(data)
    vl.extractValues(data)
    vr.extractValues(data)
    v.extractValues(data)
    v1.extractValues(data)
    v2.extractValues(data)
    ax.extractValues(data)
    ay.extractValues(data)
    az.extractValues(data)
});