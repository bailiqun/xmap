var Clients = [];
var address = {'ip':'*', 'port':80};

var express = require('express');
var app = express();
var http = require('http').Server(app);
var uuid = require('node-uuid');
var io = require('socket.io')(http, {port: address.port});

require('events').EventEmitter.defaultMaxListeners = Infinity;
app.use('/map',express.static(__dirname+'/public/map'));
app.use('/img',express.static(__dirname+'/public/img'));
app.use('/lib',express.static(__dirname+'/lib'));
app.use('/lib/leaflet/',     express.static(__dirname+'/lib/leaflet/'));
app.use('/lib/jquery/',      express.static(__dirname+'/lib/jquery-1.12.3/'));
app.use('/lib/bootstrap/',   express.static(__dirname+'/lib/bootstrap-3.3.5/'));
app.use('/lib/font-awesome/',express.static(__dirname+'/lib/font-awesome-4.7.0/'));
app.get('/', function (req, res){ res.sendFile(__dirname+'/index.html') });

http.listen(address.port);

io.on('connection', function (socket) {
    var client_uuid = uuid.v4(socket);
    Clients.push({ "id": client_uuid, "socket": socket });
    console.log('[%s] client connected, remain %d users', client_uuid,Clients.length);
    io.emit('/notification/robot',{
        'topic':'mapinfo_request'
    });

    var heart_beat_timer = setTimeout(function(){
        closeSocket();
        clearTimeout(heart_beat_timer);
    },5000);
    socket.on('heart_beat', function(){
        clearTimeout(heart_beat_timer);
        heart_beat_timer = setTimeout(function(){
            closeSocket();
            clearTimeout(heart_beat_timer);
        },5000);
    });

    socket.on('disconnect', function () {
        closeSocket();
    });

    var closeSocket = function () {
        var offline_uuid;
        for (var i = 0; i < Clients.length; ++i)
        {
            if (Clients[i].id == client_uuid)
            {
                offline_uuid = Clients[i].id;
                Clients.splice(i, 1);
                console.log('[%s] client disconnect, remain %d users', client_uuid, Clients.length);
            }
        }
    };

    process.on('SIGINT', function () {
        console.log("Closing things");
        closeSocket('Server has disconnected');
        process.exit();
    });

    socket.on('/notification/web', function (message) {
        switch(message.topic)
        {
            case 'goal_set':{
                //send all web clients
                io.emit("/notification/web",{
                    "topic": 'goal_info',
                    "id"   :  message.id,
                    "pose": {
                        "px":  message.pose.px,
                        "py":  message.pose.py,
                        "head":message.pose.head
                    }
                });
                //send to robot for goal
                io.emit("/notification/robot",{
                    "topic": 'goal_info',
                    "id"   :  message.id,
                    "pose": {
                        "px":  message.pose.px,
                        "py":  message.pose.py,
                        "head":message.pose.head
                    }
                });
                break;
            }
            default:{
                break;
            }
        }

    });

    socket.on('/notification/robot', function (message) {
        switch(message.topic)
        {
            case "map_info":
                io.emit("/notification/web",{
                    'topic': 'map_info',
                    'id':     message.id,
                    'from':   message.from,
                    'info': {
                        'width':      message.info.width,
                        'height':     message.info.height,
                        'origin_x':   message.info.origin_x,
                        'origin_y':   message.info.origin_y,
                        'goal_x':     message.info.goal_x/message.info.resolution,
                        'goal_y':     message.info.goal_y/message.info.resolution,
                        'resolution': message.info.resolution
                    }
                });
                break;
            case "goal_info":
                io.emit("/notification/web",{
                "topic": 'goal_info',
                "id"   :  message.id,
                "pose": {
                    "px":  message.pose.px,
                    "py":  message.pose.py,
                    "head":message.pose.head
                }
            });break;

            case "path_info":
                io.emit("/notification/web",{
                    "topic": 'path_info',
                    "id"   :  message.id,
                    "size" :  message.size,
                    "path" :  message.path
                });break;

            case "pose_vel_info":
                io.emit("/notification/web",{
                    'topic': 'pose_vel_info',
                    'status': message.status,
                    "id"   :  message.id,
                    'pose':   message.pose, //{'px': x, 'py': y, 'head': yaw}
                    'vel' :   message.vel  //{'vx':vx, 'vy':vy, 'vth' :vth}
                });break;

            default:
                break;
        }
    });
});