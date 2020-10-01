#! /usr/bin/nodejs

var clargs = process.argv.slice(2);
for (let i=0; i<clargs.length; i++){
	if(clargs[i] == '--help'){
		console.log('agrs: /path/to/pkg:telepresence/ host port_telepresence port_websocket move_base_instance cmd_vel_topic port_stream image_topic');
		return;
	}
}

var host               = clargs[1];
var port               = clargs[2];
var port_ws            = clargs[3];
var move_base_instance = clargs[4];
var cmd_vel_topic      = clargs[5];
var port_stream        = clargs[6];
var image_topic        = clargs[7];
var websocket_address = "ws://" + host + ":" + port_ws;

const express = require('express');
const path = require('path');

const app = express();

app.use(express.json());
app.use(express.static(path.join(clargs[0], 'scripts')));

app.set('view engine', 'pug');
// app.set('views', path.join(__dirname, 'views'));
app.set('views', path.join(clargs[0], 'views'));


app.get('/',
	(req, res)=>{ res.render('telepresence'); });
app.get('/websocket_address',
	(req, res)=>{ res.send(websocket_address); });
app.get('/move_base_instance',
	(req, res)=>{ res.send(move_base_instance); });
app.get('/cmd_vel_topic',
	(req, res)=>{ res.send(cmd_vel_topic); });
var stream_url = "\"" + "http://" + host +":"+ port_stream + "/stream?topic=" + image_topic + "\"";
app.get('/stream_url',
	(req, res)=>{ res.send(stream_url); });


app.listen(port, host, ()=>console.log("Server ready on "+host+":"+port+" (host:port)"));
