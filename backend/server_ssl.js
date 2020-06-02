var express = require('express');
var path = require('path');
var cookieParser = require('cookie-parser');
var logger = require('morgan');
var https = require("https");
var fs = require("fs");
var io = require("socket.io");
var easyrtc = require("open-easyrtc");

var frontendRouter = require('./routes/frontend');
var apiIndexRouter = require('./routes/api/index');

var app = express();

app.use(logger('dev'));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

//Setup routes
app.use('/', frontendRouter);
app.use('/api/', apiIndexRouter);

//Setup https webserver
var webServer = https.createServer({
    key:  fs.readFileSync("ssl/key.pem"),
    cert: fs.readFileSync("ssl/cert.pem")
  }, app).listen(40000);

// Start Socket.io so it attaches itself to Express server
var socketServer = io.listen(webServer, {"log level":1});

//Setup easyRTC
var appIceServers = [
  {
    "url":"stun:telesante.3it.usherbrooke.ca:3478"
  },
  {
    "url":"turn:telesante.3it.usherbrooke.ca:50000", 
    "username":"teraplus",
    "credential":"teraplus"
  },
  {
    "url":"turn:telesante.3it.usherbrooke.ca:50000?transport=tcp",
    "username":"teraplus",
    "credential":"teraplus"
  }
];

  easyrtc.setOption("appIceServers", appIceServers);
  easyrtc.setOption("logLevel", "debug");
  
  // Start EasyRTC server
  var rtc = easyrtc.listen(app, socketServer);

