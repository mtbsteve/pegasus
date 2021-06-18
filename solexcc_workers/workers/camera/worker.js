'use strict';
const spawn = require("child_process").spawn;
const path = require("path");

const ATTRS = {
    id: "camera",
    // Name/description
    name: "Camera",
    description: "ZED Camera control",
    // Does this worker want to loop?
    looper: false,
    // Mavlink messages we're interested in
    mavlinkMessages: []
};

let api = null;

function d(str) {
    ATTRS.log(ATTRS.id, str);
}

/*
Return an object describing this worker. If looper is true, this module must expose a loop() export.
*/
function getAttributes() {
    return ATTRS;
}

// Called from dispatch.loop()
function loop() {
}

// Called when this worker is loaded.
function onLoad() {
    d("onLoad()");
    api = ATTRS.api;
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
    d(`onGCSMessage(): msg.id=${JSON.stringify(msg)}`);

    const result = {
        ok: true
    };

    switch(msg.id) {
        case "take_picture": {
            d("now call camera to take a picture");
            var strg=JSON.stringify(msg);
            if(strg.includes("videocolor")) {
                   //d("now call camera take_picture video");
                   takePicture("real");

            } else if(strg.includes("depthgrey")) {
                   takePicture("depth");

            } else if(strg.includes("yolocolor")) {
                   takePicture("yolo");
            } else {
                   //d("now call camera take_picture all images");
                   takePicture("color");
            }
            break;
        }

        case "toggle_video": {
            var strg=JSON.stringify(msg);
            d(`now call toggle video with: ${strg}`);
            if(strg.includes("videocolor") && mRecordingVideo == false) {
                   toggleVideo("real");
                   ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "Video recording started", ATTRS.api.WorkerUI.SpeechType.TEXT);
            } else if (strg.includes("depthcolor") && mRecordingVideo == false) {
                   toggleVideo("color");
                   ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "Recording of all images started", ATTRS.api.WorkerUI.SpeechType.TEXT);
            } else if (strg.includes("yolocolor") && mRecordingVideo == false) {
                   toggleVideo("yolo");
                   ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "YOLO object recognition recording started", ATTRS.api.WorkerUI.SpeechType.TEXT);
            } else if (strg.includes("depthgrey") && mRecordingVideo == false) {
                   toggleVideo("depth");
                   ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "Recording Depth Map started", ATTRS.api.WorkerUI.SpeechType.TEXT);
            } else if (mRecordingVideo == true) {
                   toggleVideo("stop");
                   ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "Recording stopped", ATTRS.api.WorkerUI.SpeechType.TEXT);
            } else {
                   sendCameraError("Error controlling camera");
                   break;
            }
            sendUpdateRecordingStatus(mRecordingVideo);
            break;
        }

        case "open_settings": {
            sendSettingsDialogMessage();
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

//
// Return a UI for the specified screen.
//
function onScreenEnter(screen) {
    switch(screen) {
        case api.WorkerUI.Const.SCREEN_FLIGHT: {
            const body = api.WorkerUI.loadLayout(__dirname, api.WorkerUI.Const.PANEL_CAMERA);

            return (body)? {
                screen_id: screen, 
                camera_panel: body
            }: null;
        }

        default: {
            return null;
        }
    }
}

function onScreenExit(screen) {

}

// Serve an image if it exists.
function onImageDownload(name) {
    return api.WorkerUI.serveImage(__dirname, name);
}

function sendCameraError(str) {
    api.WorkerUI.sendSpeechMessage(ATTRS, str, api.WorkerUI.SpeechType.ERROR);
}

// routine to take a picture
var mChildProcess = null;

function takePicture(picparam) {
    if(mChildProcess) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
        ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "Process already running!", ATTRS.api.WorkerUI.SpeechType.ERROR);
    };
    d(`parameter: ${picparam}`);
   
    sendUpdatePictureStatus(true);
    d(`parameter vor übergabe: ${picparam}`);
    const server = path.join(__dirname, `takepicture.sh ${picparam}`);
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout RECEIVED: ${data.toString('utf-8')}`);
        if(data.toString('utf-8').includes ("taken")) {
           ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, `picture ${data.toString('utf-8')}`, ATTRS.api.WorkerUI.SpeechType.TEXT);
           mChildProcess = null;
           sendUpdatePictureStatus(false);
           return {ok: true, message: "picture taken ok"};
        }
    });

    child.stderr.on("data", function(data) {
        d(`child.stderr: ${data.toString('utf-8')}`);
        if(data.toString('utf-8').includes ("err")) {
           ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, `ROS Error: Failed to contact RTSP video node`, ATTRS.api.WorkerUI.SpeechType.ERROR);
           sendUpdatePictureStatus(false);
           const server = path.join(__dirname, `takepicture.sh stop`);
           d(`server=${server}`);
           spawn("sh", [ server ], { shell: true });
           return {ok: false, message: "no access to RTSP server"};
        }
    });


//return {ok: true, message: "picture taken ok"};
}

var mChildProcess2 = null;
var mRecordingVideo = false;

function toggleVideo(vidparam) {
    if(mChildProcess2) {
        d(`Child process is already running`);
        return {ok: false, message: "Child process is already running"};
        ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "Process already running!", ATTRS.api.WorkerUI.SpeechType.ERROR);
    };
    d(`parameter received: ${vidparam}`);
    if(vidparam == "stop") {
        mRecordingVideo = false;
    } else {
        mRecordingVideo = true;
    };
    const server = path.join(__dirname, `startstop_video.sh ${vidparam}`);
    d(`server=${server}`);

    const child = spawn("sh", [ server ], { shell: true });
    child.stdin.setEncoding("utf-8");
    child.on("error", function (error) {
        d(`Error starting child process: ${error}`);
    });

    child.stdout.on("data", function(data) {
        d(`child.stdout RECEIVED: ${data.toString('utf-8')}`);
    });

    child.stderr.on("data", function(data) {
        d(`child.stderror received: ${data.toString('utf-8')}`);
        if(data.toString('utf-8').includes ("ERROR")) {
          
          ATTRS.api.WorkerUI.sendSpeechMessage(ATTRS, "ROS Error: Failed to contact RTSP video node", ATTRS.api.WorkerUI.SpeechType.ERROR);
          const server = path.join(__dirname, `startstop_video.sh stop`);
          d(`server=${server}`);
          spawn("sh", [ server ], { shell: true });
          mRecordingVideo = false;
          sendUpdateRecordingStatus(mRecordingVideo);
          return {ok: false, message: "no access to RTSP server"};
        }
    });

    child.on("close", function(code) {
        d(`Child closed with ${code}`);
        mChildProcess = null;
    });
    mChildProcess = child;
    return {ok: true, message: "video taken"};
}



function sendUpdateRecordingStatus(recording) {
    const imgName = (recording) ? "record_on.png" : "record_off.png";

    ATTRS.sendGCSMessage(ATTRS.id, {
        id: "screen_update",
        screen_id: "flight",
        panel_id: "camera_panel",
        values: {
            btn_video: {
                icon: `$(img)/${ATTRS.id}/${imgName}`
            },
            spin_image_node: { enabled: !recording }
        }
    });
}

function sendUpdatePictureStatus(recording) {
    const imgName = (recording) ? "shutter_close2.png" : "shutter.png";

    ATTRS.sendGCSMessage(ATTRS.id, {
        id: "screen_update",
        screen_id: "flight",
        panel_id: "camera_panel",
        values: {
            take_picture: {
                icon: `$(img)/${ATTRS.id}/${imgName}`
            },
            spin_image_node: { enabled: !recording }
        }
    });
}

function sendSettingsDialogMessage() {
    const body = api.WorkerUI.loadLayout(__dirname, "camera_settings");
    if (body) {
        ATTRS.sendGCSMessage(ATTRS.id, { id: "display_dialog", content: body });
    }
}


function onBroadcastRequest(msg) {
    switch(msg.type) {
        case "mission_item_support": {
            return {
                id: ATTRS.id,
                name: ATTRS.name,
                actions: [
                    { 
                        id: "video_start", 
                        name: "Start Video", 
                        msg_id: "video_start", 
                        params: [
                            {id: "image_mode", name: "Image mode", type: "enum", values: [
                                {id: "video", name: "video"},
                                {id: "confidence", name: "confidence"},
                                {id: "depthcolor", name: "colormap"}
                            ], 
                            default: "med"}
                        ]
                    },
                    { id: "video_stop", name: "Stop Video", msg_id: "video_stop" },
                    { id: "photo", name: "Take Photo", msg_id: "take_photo" },
                ]
            }
        }

        default: {
            return null;
        }
    }
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
exports.onBroadcastRequest = onBroadcastRequest;
