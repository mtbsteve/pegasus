'use strict';
const spawn = require("child_process").spawn;
const path = require("path");

const ATTRS = {
    id: "ZEDstart_panel",
    // Name/description
    name: "ZED Start Panel",
    description: "ZED ROS Buttons on the start panel",
    // Does this worker want to loop?
    looper: false,
    // Mavlink messages we're interested in
    mavlinkMessages: []
};

function d(str) {
    ATTRS.log(ATTRS.id, str);
}

/*
Return an object describing this worker. If looper is true, this module must expose a loop() export.
*/
function getAttributes() {
    return ATTRS;
}

let loopIterations = 0;

// Called from dispatch.loop()
function loop() {
}

// Called when this worker is loaded.
function onLoad() {
    d("onLoad()");
}

// Called when unloading
function onUnload() {
    d("onUnload()");
}

// Called when a Mavlink message arrives
function onMavlinkMessage(msg) {
    d(`onMavlinkMessage(): msg.name=$msg.name`);
}

// Called when the GCS sends a message to this worker. Message format is 
// entirely dependent on agreement between the FCS and worker implementation.
function onGCSMessage(msg) {
    d(`onGCSMessage(): msg.id=${msg.id}`);

    const result = {
        ok: true
    };
    d(`message received is ${JSON.stringify(msg)}`);

    switch(msg.id) {
        case "ros_stereodnn_start": {
            startstereodnnnode();
            break;
        }
        case "ros_stereodnn_stop": {
            stopstereodnnnode();
            break;
        }
        case "ros_trailnet_start": {
            starttrailnetnode();
            break;
        }
        case "ros_trailnet_stop": {
            stoptrailnetnode();
            break;
        }
        case "ros_yolo_start": {
            startyolonode();
            break;
        }
        case "ros_yolo_stop": {
            stopyolonode();
            break;
        }

        case "ros_mavros_start": {
            startmavrosnode();
            break;
        }

        case "ros_mavros_stop": {
            stopmavrosnode();
            break;
        }


        case "show_dialog1": {
            sendShowDialogMessage1();
            break;
        }

        case "show_dialog2": {
            sendShowDialogMessage2();
            break;
        }

        case "show_dialog3": {
            sendShowDialogMessage3();
            break;
        }

        case "show_dialog4": {
            sendShowDialogMessage4();
            break;
        }
        case "show_dialog5": {
            sendShowDialogMessage5();
            break;
        }

        case "shutdown_ok": {
            d(`message is ${JSON.stringify(msg)}`);
            shutdowntx2();    
        break;
        }

        default: {
            result.ok = false;
            result.message = `No message with id ${msg.id}`;
            break;
        }
    }

    return result;
}

var stereodnnstart = false;
var trailnetstart = false;
var yolostart = false;
var mavrosstart = false;

// Return a UI for the specified screen.
function onScreenEnter(screen) {
    switch(screen) {
        case ATTRS.api.WorkerUI.Const.SCREEN_START: {
            const buttonColor1 = (stereodnnstart) ? "#ff39aa00" : "black";            
            const body = loadLayoutFor(ATTRS.api.WorkerUI.Const.PANEL_WORKER_BUTTONS);
            d(`Screen entered start`);
            if(body) {
                //body.children[1].text = buttonText;
                body.children[1].background = buttonColor1;
 
                return {
                    screen_id: screen,
                    worker_buttons: body
                };
            } else {
                return null;
            }
        }

        default: {
            return null;
        }
    }
}

function onScreenExit(screen) {

}

function sendZEDButtonUpdate1() {
    const buttonColor = (stereodnnstart)? "#ff39aa00": "black";

    ATTRS.sendGCSMessage(ATTRS.id, {
        id: "screen_update",
        screen_id: "start",
        panel_id: "worker_buttons",
        values: {
            do_dialog1: {
                background: buttonColor
            }
        }
    });
}

function sendZEDButtonUpdate2() {
    const buttonColor = (trailnetstart)? "#ff39aa00": "black";

    ATTRS.sendGCSMessage(ATTRS.id, {
        id: "screen_update",
        screen_id: "start",
        panel_id: "worker_buttons",
        values: {
            do_dialog2: {
                background: buttonColor
            }
        }
    });
}

function sendZEDButtonUpdate3() {
    const buttonColor = (yolostart)? "#ff39aa00": "black";

    ATTRS.sendGCSMessage(ATTRS.id, {
        id: "screen_update",
        screen_id: "start",
        panel_id: "worker_buttons",
        values: {
            do_dialog3: {
                background: buttonColor
            }
        }
    });
}

function sendZEDButtonUpdate4() {
    const buttonColor = (mavrosstart)? "#ff39aa00": "black";

    ATTRS.sendGCSMessage(ATTRS.id, {
        id: "screen_update",
        screen_id: "start",
        panel_id: "worker_buttons",
        values: {
            do_dialog5: {
                background: buttonColor
            }
        }
    });
}

// Serve an image if it exists.
function onImageDownload(name) {
    return ATTRS.api.WorkerUI.serveImage(__dirname, name);
}

// Start and stop od the StereoDNN node. This includes ZED ROS, resnet18-2D and the color coded depth images
var mChildProcess1 = null;
var mChildProcess2 = null;

function startstereodnnnode(msg) {
    stereodnnstart = true;
    d(`message received is start stereodnn node!`);
    if(mChildProcess1) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
    }

    const server = path.join(__dirname, "startstopStereoDNNnodes.sh start");
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    d(`child=${child}`);

    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout: ${data.toString('utf-8')}`);
        if(data.toString('utf-8').includes ("ZED camera launched")) {
           ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "ZED camera initiated", ATTRS.api.WorkerUI.SpeechType.TEXT);
           sendZEDButtonUpdate1();
        }
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
        mChildProcess1 = null;
    });

    mChildProcess1 = child;

    return {ok: true, message: "started"};
}

function stopstereodnnnode(msg) {
    stereodnnstart = false;
    d(`message received is stop StereoDNN node!`);
    if(mChildProcess2) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
    }

    const server = path.join(__dirname, "startstopStereoDNNnodes.sh stop");
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    d(`child=${child}`);

    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout: ${data.toString('utf-8')}`);
        sendZEDButtonUpdate1();
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
        mChildProcess2 = null;
    });

    mChildProcess2 = child;

    return {ok: true, message: "started"};
}

// Start and stop of the ZED to ROS to mavlink node, 
var mChildProcess3 = null;
var mChildProcess4 = null;

function starttrailnetnode(msg) {
    trailnetstart = true;
    d(`message received is start ZED2Mavlink node!`);
    if(mChildProcess3) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
    }

    const server = path.join(__dirname, "startstoptrailnetnodes.sh start");
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    d(`child=${child}`);

    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout: ${data.toString('utf-8')}`);
        if(data.toString('utf-8').includes ("ROS2Mavlink launched")) {
           ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "ZED to Mavlink node initiated", ATTRS.api.WorkerUI.SpeechType.TEXT);
           sendZEDButtonUpdate2();
        }
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
        mChildProcess3 = null;
    });

    mChildProcess3 = child;

    return {ok: true, message: "started"};
}

function stoptrailnetnode(msg) {
    trailnetstart = false;
    d(`message received is stop Trailnet node!`);
    if(mChildProcess4) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
    }

    const server = path.join(__dirname, "startstoptrailnetnodes.sh stop");
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    d(`child=${child}`);

    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout: ${data.toString('utf-8')}`);
        sendZEDButtonUpdate2();
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
        mChildProcess4 = null;
    });

    mChildProcess4 = child;

    return {ok: true, message: "started"};
}

// Start and stop of the YOLO node, requires previous launch of StereoDNN or trailnet nodes
var mChildProcess5 = null;
var mChildProcess6 = null;

function startyolonode(msg) {
    yolostart = true;
    d(`message received is start YOLO node!`);
    if(mChildProcess5) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
    }

    const server = path.join(__dirname, "startstopdarknetyolonodes.sh start");
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    d(`child=${child}`);

    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout: ${data.toString('utf-8')}`);
        if(data.toString('utf-8').includes ("YOLO node launched")) {
           ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "YOLO DNN initiated", ATTRS.api.WorkerUI.SpeechType.TEXT);
           sendZEDButtonUpdate3();
        }
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
        mChildProcess5 = null;
    });

    mChildProcess5 = child;

    return {ok: true, message: "started"};
}

function stopyolonode(msg) {
    yolostart = false;
    d(`message received is stop YOLO node!`);
    if(mChildProcess6) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
    }

    const server = path.join(__dirname, "startstopdarknetyolonodes.sh stop");
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    d(`child=${child}`);

    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout: ${data.toString('utf-8')}`);
        sendZEDButtonUpdate3();
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
        mChildProcess6 = null;
    });

    mChildProcess6 = child;

    return {ok: true, message: "started"};
}

// start of MAVROS and the PX4 controller nodes
var mChildProcess7 = null;
var mChildProcess8 = null;

function startmavrosnode(msg) {
    mavrosstart = true;
    d(`message received is start MAVROS node!`);
    if(mChildProcess7) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
    }

    const server = path.join(__dirname, "startstopMavros_PX4nodes.sh start");
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    d(`child=${child}`);

    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout: ${data.toString('utf-8')}`);
        if(data.toString('utf-8').includes ("MAVROS node launched")) {
           ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "MAVROS and PX4 initiated", ATTRS.api.WorkerUI.SpeechType.TEXT);
           sendZEDButtonUpdate4();
        }
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
        if(data.toString('utf-8').includes ("Trailnet is not running")) {
           ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "Trailnet is not running must be started first", ATTRS.api.WorkerUI.SpeechType.TEXT);
        }
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
        mChildProcess7 = null;
    });

    mChildProcess7 = child;

    return {ok: true, message: "started"};
}

function stopmavrosnode(msg) {
    mavrosstart = false;
    d(`message received is stop MAVROS node!`);
    if(mChildProcess8) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
    }

    const server = path.join(__dirname, "startstopMavros_PX4nodes.sh stop");
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    d(`child=${child}`);

    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout: ${data.toString('utf-8')}`);
        sendZEDButtonUpdate4();
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
        mChildProcess8 = null;
    });

    mChildProcess8 = child;

    return {ok: true, message: "started"};
}


// shutdown of the TX2
function shutdowntx2() {
    d(`message received is shutdown`);

    const server = path.join(__dirname, "shutdowntx2.sh");
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    d(`child=${child}`);

    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout RECEIVED: ${data.toString('utf-8')}`);
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
    });
    return {ok: true, message: "started"};
}

// functions to display the pop up menus 
function sendShowDialogMessage1() {
    const body = loadLayoutFor("display_dialog1");

    if(body) {
        ATTRS.sendGCSMessage(ATTRS.id, { id: "display_dialog", content: body });
    }
}

function sendShowDialogMessage2() {
    const body = loadLayoutFor("display_dialog2");

    if(body) {
        ATTRS.sendGCSMessage(ATTRS.id, { id: "display_dialog", content: body });
    }
}

function sendShowDialogMessage3() {
    const body = loadLayoutFor("display_dialog3");

    if(body) {
        ATTRS.sendGCSMessage(ATTRS.id, { id: "display_dialog", content: body });
    }
}

function sendShowDialogMessage4() {
    const body = loadLayoutFor("display_dialog4");

    if(body) {
        ATTRS.sendGCSMessage(ATTRS.id, { id: "display_dialog", content: body });
    }
}

function sendShowDialogMessage5() {
    const body = loadLayoutFor("display_dialog5");

    if(body) {
        ATTRS.sendGCSMessage(ATTRS.id, { id: "display_dialog", content: body });
    }
}


function loadLayoutFor(panel) {
    return ATTRS.api.WorkerUI.loadLayout(__dirname, panel);
}

exports.getAttributes = getAttributes;
exports.loop = loop;
exports.onLoad = onLoad;
exports.onUnload = onUnload;
exports.onMavlinkMessage = onMavlinkMessage;
exports.onGCSMessage = onGCSMessage;
exports.onScreenEnter = onScreenEnter;
exports.onScreenExit = onScreenExit;
exports.onImageDownload = onImageDownload;


