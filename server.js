var express = require('express');
var app = express();
var http = require('http').Server(app);
var uuid = require('node-uuid');
var io = require('socket.io')(http, {
    port:3000
});

require('events').EventEmitter.defaultMaxListeners = Infinity;

app.use('/map',express.static(__dirname+'/public/map'));
app.use('/img',express.static(__dirname+'/public/img'));
app.use('/lib',express.static(__dirname+'/lib'));
app.use('/lib/leaflet/',     express.static(__dirname+'/lib/leaflet/'));
app.use('/lib/jquery/',      express.static(__dirname+'/lib/jquery-1.12.3/'));
app.use('/lib/bootstrap/',   express.static(__dirname+'/lib/bootstrap-3.3.5/'));
app.use('/lib/font-awesome/',express.static(__dirname+'/lib/font-awesome-4.7.0/'));

app.get('/', function (req, res) {
    res.sendFile(__dirname + '/index.html');
});

function io_send_respones(event, type) {
    io.emit(event,{
        "type": type
    });
}

function io_send_pose(event, type, client_uuid, px, py, head) {
    io.emit(event,{
        "type": type,
        "id": client_uuid,
        "pose": {
            "px":px,
            "py":py,
            "head":head
        }
    });
}

var target_pose={};
var clients = [];
var clientIndex = 1;

io.on('connection', function (socket) {
    var heart_beat_timer;
    var client_uuid = uuid.v4(socket);

    clientIndex += 1;
    clients.push({ "id": client_uuid, "socket": socket });
    console.log('[%s] client connected, remain %d users', client_uuid,clients.length);
    io_send_respones('notification','robot_map_info_req');

    /*-------只是连接还没开始发送心跳----------*/
    heart_beat_timer = setTimeout(function(){
        closeSocket();
        clearTimeout(heart_beat_timer);
    },5000);

    /*--------------开始发送心跳-------------*/
    socket.on('heart_beat', function(){
        clearTimeout(heart_beat_timer);
        heart_beat_timer = setTimeout(function(){
            closeSocket();
            clearTimeout(heart_beat_timer);
        },5000);
    });

    //机器人端发送位置信息到pose_get
    socket.on('pose_upload', function (message) {
        io.emit('location',message);
    });

    socket.on('path', function (message) {
        io.emit('path',message);
    });

    socket.on('notification', function (message) {
        switch(message.type)
        {
            case "robot_map_info": {
                io.emit("notification",{
                    "type": "robot_map_info",
                    "id": client_uuid,
                    "info": {
                        "resolution": message.info.resolution,
                        "width": message.info.width,
                        "height": message.info.height,
                        "origin_x": message.info.origin_x,
                        "origin_y": message.info.origin_y,
                        "goal_x":message.info.goal_x,
                        "goal_y":message.info.goal_y
                    }
                });
                break;
            }
            case "goal_set":
            {
                var px  =  message.pose.px;
                var py  =  message.pose.py;
                var head = message.pose.head;
                target_pose['px']=px;
                target_pose['py']=py;
                target_pose['head'] = head;

                io.emit("notification",{
                "type": 'goal_set',
                    "id": client_uuid,
                    "pose": {
                        "px":target_pose['px'],
                        "py":target_pose['py'],
                        "head":target_pose['head']
                    }
                });
            }
            default:
                break;
        }
    });


    socket.on('disconnect', function () {
        closeSocket();
    });

    var closeSocket = function ()
    {
        var offline_uuid;
        for (var i = 0; i < clients.length; i++)
        {
            if (clients[i].id == client_uuid)
            {
                offline_uuid = clients[i].id;
                clients.splice(i, 1);
                console.log('[%s] client disconnect, remain %d users', client_uuid,clients.length);
            }
        }
    };

    process.on('SIGINT', function () {
        console.log("Closing things");
        closeSocket('Server has disconnected');
        process.exit();
    });
});

http.listen(3000, function () {
    console.log('listening on *:3000');
});
