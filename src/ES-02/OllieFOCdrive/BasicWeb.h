#include <WiFi.h>

/***********html&javascript Web page construction**********/
const char BasicWeb[] PROGMEM = R"=====(

<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Legged Wheel Robot Web Control</title>
    <style>
        h2 {
            width: auto;
            height: 60px;
            line-height: 60px;
            text-align: center;
            font-family: serif;
            color: white;
            background-color: cornflowerblue;
            border-radius: 12px;
        }

        input {
            width: 160px;
            height: 30px;
            margin: 0px;
        }

        .sliderLabel {
            float: left;
            text-align: center;
            line-height: 30px;
            height: 30px;
            width: 200px;
        }

        .sliders {
            width: 400px;
            height: 150px;
            margin: 10px auto;
            position: relative;
        }

        .view2 {
            width: 140px;
            height: 30px;
            padding: 0px;
            margin: 15px auto;
            vertical-align: middle;
        }

        .switch-container {
            display: flex;
            flex-direction: column;
            align-items: center;
            margin: 15px auto;
            width: 205px;
        }

        .switch-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            width: 100%;
            margin: 5px 0;
        }

        .switch-label {
            margin-right: 10px;
            font-family: Arial, sans-serif;
        }

        /* Switch style */
        input[type='checkbox'].switch {
            outline: none;
            appearance: none;
            -webkit-appearance: none;
            -moz-appearance: none;
            position: relative;
            width: 40px;
            height: 20px;
            background: #ccc;
            border-radius: 10px;
            transition: border-color .3s, background-color .3s;
        }

        input[type='checkbox'].switch::after {
            content: '';
            display: inline-block;
            width: 1rem;
            height: 1rem;
            border-radius: 50%;
            background: #fff;
            box-shadow: 0, 0, 2px, #999;
            transition: .4s;
            top: 2px;
            position: absolute;
            left: 2px;
        }

        input[type='checkbox'].switch:checked {
            background: rgb(78, 78, 240);
        }

        input[type='checkbox'].switch:checked::after {
            content: '';
            position: absolute;
            left: 55%;
            top: 2px;
        }

        /* Three-state switch styles */
        input[type='range'].three-state {
            width: 80px;
            height: 20px;
            -webkit-appearance: none;
            background: #ccc;
            border-radius: 10px;
            margin-left: 10px;
        }

        input[type='range'].three-state::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 20px;
            height: 20px;
            background: rgb(78, 78, 240);
            border-radius: 50%;
            cursor: pointer;
        }

        .state-indicator {
            display: flex;
            justify-content: space-between;
            width: 80px;
            margin-left: 10px;
            font-size: 12px;
        }

        * {
            -webkit-touch-callout: none;
            -webkit-user-select: none;
            -khtml-user-select: none;
            -moz-user-select: none;
            -ms-user-select: none;
            user-select: none;
        }

        /* Joystick content */
        .row {
            display: flex;
            justify-content: center;
        }

        #joystick {
            border: 0px solid #FF0000;
        }
    </style>
</head>


<body onload="socket_init()">
<h2>WL-PRO WiFi Control Mode</h2>
<div class="view2">
    <input type="checkbox" id="stable" class="switch" onclick="is_stable()" style="vertical-align:middle">
    <label for="stable">Robot Go!</label>
</div>

<div class="switch-container">
    <div class="switch-item">
        <span class="switch-label">Balance Mode</span>
        <div style="display: flex; flex-direction: column; align-items: center;">
            <input type="range" id="balance_mode" class="three-state" min="0" max="2" step="1" value="0"
                   onchange="updateThreeState('balance_mode')">
            <div class="state-indicator">
                <span>off</span>
                <span>1</span>
                <span>2</span>
            </div>
        </div>
    </div>
    <div class="switch-item">
        <span class="switch-label">Servo Reset</span>
        <input type="checkbox" id="servo_reset" class="switch" onclick="updateToggle('servo_reset')">
    </div>
    <div class="switch-item">
        <span class="switch-label">Ball Handler</span>
        <input type="checkbox" id="ball_handler" class="switch" onclick="updateToggle('ball_handler')">
    </div>
    <div class="switch-item">
        <span class="switch-label">Posture Mode</span>
        <div style="display: flex; flex-direction: column; align-items: center;">
            <input type="range" id="posture_mode" class="three-state" min="0" max="2" step="1" value="0"
                   onchange="updateThreeState('posture_mode')">
            <div class="state-indicator">
                <span>off</span>
                <span>1</span>
                <span>2</span>
            </div>
        </div>
    </div>
</div>

<div class="row">
    <div id="joy1Div" style="width:200px;height:200px;margin:10px"></div>
</div>

<div class="sliders">
    <div>
        <input type="range" min="0" max="100" value="50" id="hSlider" oninput="setHeight()"/>
        <label class="sliderLabel" for="hSlider" id="hLabel">BaseHeight: 38%</label>
    </div>
    <div>
        <input type="range" min="-100" max="100" value="0" id="rollSlider" oninput="setroll()"/>
        <label class="sliderLabel" for="rollSlider" id="rollLabel">Roll: 0%</label>
    </div>
</div>

<script>
    var socket;
    var g_roll = 0;
    var g_h = 38;
    var g_stable = 0;
    var joyX = 0;
    var joyY = 0;
    var sendData = {};

    // Updated toggle states with three-state options
    var toggleStates = {
        balance_mode: 0,
        servo_reset: 0,
        ball_handler: 0,
        posture_mode: 0
    };

    function socket_init() {
        socket = new WebSocket('ws://' + window.location.hostname + ':81/'); // sta mode

        socket.onopen = function () {
            console.log("WebSocket connection established");
        };

        socket.onerror = function (error) {
            console.log("WebSocket error: ", error);
        };

        socket.onclose = function () {
            console.log("WebSocket connection closed");
        };
    }

    function updateThreeState(toggleId) {
        var slider = document.getElementById(toggleId);
        toggleStates[toggleId] = parseInt(slider.value);
        send_data();
    }

    function updateToggle(toggleId) {
        var checkbox = document.getElementById(toggleId);
        toggleStates[toggleId] = checkbox.checked ? 1 : 0;
        send_data();
    }

    function setroll() {
        let val = document.getElementById("rollSlider").value;
        val = parseInt(val);
        document.getElementById("rollLabel").innerHTML = "Roll: " + val + "%";
        g_roll = val;
        send_data();
    }

    function setHeight() {
        let val = document.getElementById("hSlider").value;
        val = parseInt(val);
        document.getElementById("hLabel").innerHTML = "Height: " + val + "%";
        g_h = (val - 50) * 2;
        send_data();
    }

    function is_stable() {
        var obj = document.getElementById("stable");
        g_stable = obj.checked ? 1 : 0;
        send_data();
    }

    function send_data() {
        let data = {
            'mode': "basic",
            'roll': g_roll,
            'height': g_h,
            'stable': g_stable,
            'joy_y': joyY,
            'joy_x': joyX,
            'balance_mode': toggleStates.balance_mode,
            'servo_reset': toggleStates.servo_reset,
            'ball_handler': toggleStates.ball_handler,
            'posture_mode': toggleStates.posture_mode
        };

        if (JSON.stringify(data) === JSON.stringify(sendData)) return;

        sendData = data;

        if (socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify(data));
            console.log("Sent:", data);
        }
    }

    var JoyStick = (function (container, parameters) {
        parameters = parameters || {};
        var title = parameters.title || "joystick",
            width = parameters.width || 0,
            height = parameters.height || 0,
            internalFillColor = parameters.internalFillColor || "#00979C",
            internalLineWidth = parameters.internalLineWidth || 2,
            internalStrokeColor = parameters.internalStrokeColor || "#00979C",
            externalLineWidth = parameters.externalLineWidth || 2,
            externalStrokeColor = parameters.externalStrokeColor || "#0097BC",
            autoReturnToCenter = parameters.autoReturnToCenter !== false;

        var objContainer = document.getElementById(container);
        var canvas = document.createElement("canvas");
        canvas.id = title;
        if (width === 0) width = objContainer.clientWidth;
        if (height === 0) height = objContainer.clientHeight;
        canvas.width = width;
        canvas.height = height;
        objContainer.appendChild(canvas);
        var context = canvas.getContext("2d");

        var isPressing = false;
        var circumference = 2 * Math.PI;
        var internalRadius = (canvas.width - ((canvas.width / 2) + 10)) / 2;
        var maxMoveStick = internalRadius + 5;
        var externalRadius = internalRadius + 30;
        var centerX = canvas.width / 2;
        var centerY = canvas.height / 2;
        var movedX = centerX;
        var movedY = centerY;

        if ("ontouchstart" in document.documentElement) {
            canvas.addEventListener("touchstart", onTouchStart, true);
            canvas.addEventListener("touchmove", onTouchMove, true);
            document.addEventListener("touchend", onTouchEnd, true);
        } else {
            canvas.addEventListener("mousedown", onMouseDown, true);
            canvas.addEventListener("mousemove", onMouseMove, true);
            document.addEventListener("mouseup", onMouseUp, true);
        }

        drawExternal();
        drawInternal();

        function drawExternal() {
            context.beginPath();
            context.arc(centerX, centerY, externalRadius, 0, circumference, false);
            context.lineWidth = externalLineWidth;
            context.strokeStyle = externalStrokeColor;
            context.stroke();
        }

        function drawInternal() {
            context.beginPath();
            if (movedX < internalRadius) movedX = maxMoveStick;
            if ((movedX + internalRadius) > canvas.width) movedX = canvas.width - maxMoveStick;
            if (movedY < internalRadius) movedY = maxMoveStick;
            if ((movedY + internalRadius) > canvas.height) movedY = canvas.height - maxMoveStick;

            context.arc(movedX, movedY, internalRadius, 0, circumference, false);
            var grd = context.createRadialGradient(centerX, centerY, 5, centerX, centerY, 200);
            grd.addColorStop(0, internalFillColor);
            grd.addColorStop(1, internalStrokeColor);
            context.fillStyle = grd;
            context.fill();
            context.lineWidth = internalLineWidth;
            context.strokeStyle = internalStrokeColor;
            context.stroke();
        }

        function postCoordinate() {
            joyX = (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
            joyY = ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
            send_data();
        }

        function releaseControl() {
            joyX = 0;
            joyY = 0;
            send_data();
        }

        function onTouchStart(event) {
            isPressing = true;
            event.preventDefault();
        }

        function onTouchMove(event) {
            if (isPressing && event.targetTouches[0].target === canvas) {
                event.preventDefault();
                movedX = event.targetTouches[0].pageX;
                movedY = event.targetTouches[0].pageY;

                if (canvas.offsetParent.tagName.toUpperCase() === "BODY") {
                    movedX -= canvas.offsetLeft;
                    movedY -= canvas.offsetTop;
                } else {
                    movedX -= canvas.offsetParent.offsetLeft;
                    movedY -= canvas.offsetParent.offsetTop;
                }

                context.clearRect(0, 0, canvas.width, canvas.height);
                drawExternal();
                drawInternal();
                postCoordinate();
            }
        }

        function onTouchEnd() {
            isPressing = false;
            if (autoReturnToCenter) {
                movedX = centerX;
                movedY = centerY;
            }
            context.clearRect(0, 0, canvas.width, canvas.height);
            drawExternal();
            drawInternal();
            releaseControl();
        }

        function onMouseDown(event) {
            isPressing = true;
        }

        function onMouseMove(event) {
            if (isPressing) {
                movedX = event.pageX;
                movedY = event.pageY;

                if (canvas.offsetParent.tagName.toUpperCase() === "BODY") {
                    movedX -= canvas.offsetLeft;
                    movedY -= canvas.offsetTop;
                } else {
                    movedX -= canvas.offsetParent.offsetLeft;
                    movedY -= canvas.offsetParent.offsetTop;
                }

                context.clearRect(0, 0, canvas.width, canvas.height);
                drawExternal();
                drawInternal();
                postCoordinate();
            }
        }

        function onMouseUp() {
            isPressing = false;
            if (autoReturnToCenter) {
                movedX = centerX;
                movedY = centerY;
            }
            context.clearRect(0, 0, canvas.width, canvas.height);
            drawExternal();
            drawInternal();
            releaseControl();
        }

        this.GetX = function () {
            return (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
        };

        this.GetY = function () {
            return ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
        };
    });

    var joy1 = new JoyStick('joy1Div', {"title": "joystick"});

    // Regular position update in case of continuous movement
    setInterval(function () {
        joyX = joy1.GetX();
        joyY = joy1.GetY();
        send_data();
    }, 150);
</script>
</body>
</html>



)=====";

