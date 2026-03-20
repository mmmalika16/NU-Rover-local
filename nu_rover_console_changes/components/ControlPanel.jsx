import React, { useEffect, useState, useCallback, useRef } from 'react'
import {
  Button,
  Typography
} from "@material-tailwind/react";

import { WebsocketCtx } from 'src/contexts/WebsocketContext';
import { LoraCtx } from 'src/contexts/LoraContext';
import { RosWebsocketCtx } from 'src/contexts/RosWebsocketContext';
import { WheelsWebsocketCtx } from 'src/contexts/WheelsWebsocketContext';
import ROSLIB from 'roslib';
import {useRecoilValue} from "recoil";

const JoystickDisplay = ({ position }) => {
    return (
        <div
            className="
        relative
        w-[200px] h-[200px]
        border-2
        border-secondary dark:border-white
        mx-auto my-5
      "
        >
            <div
                style={{
                    width: 0,
                    height: 0,
                    borderLeft: '10px solid transparent',
                    borderRight: '10px solid transparent',
                    borderBottom: '20px solid #29EB18',
                    position: 'absolute',
                    left: `calc(${50 + position.x * 50}%)`,
                    top: `calc(${50 + position.y * 50}%)`,
                    transform: `translate(-50%, -50%) rotate(${(position.z * 360) / Math.PI}deg)`,
                    transformOrigin: '50% 50%',
                }}
            />
        </div>
    );
};

const ControlPanel = () => {
    const { serialState, sendTextMessage } = LoraCtx();
    const { readyState, sendMessage } = WebsocketCtx();
    const { publishMessage, rosState } = RosWebsocketCtx();
    const { publishMessage: publishWheelsMessage, rosState: wheelsRosState } = WheelsWebsocketCtx();
    const keyCodes = new Map([
        ["W", 87],
        ["A", 65],
        ["S", 83],
        ["D", 68],
        ["U", 85],
        ["J", 74]
    ]);

    // roboarm control joystick
    const [joystickPosition, setJoystickPosition] = useState({ x: 0, y: 0, z: 0 });

    // sending the position through LoRa or Websocket
    const handleMove = (msg, emergencySend = false) => {
        // let dir = vector;
        if (!emergencySend)
            return;

        // dir = dir + "\n";

        if (wheelsRosState.connected)
            publishWheelsMessage(msg);
        else
            console.log("Please turn on WebSocket or LoRa communication to send message");
    }

    let moveInterval = null;
    let currentKeyCode = null;
    let isMoving = false; 
    const keydownHandler = (e) => {
        let keyCode = e.keyCode;
        let direction = "";

        switch (keyCode) {
        case keyCodes.get("W"):
            direction = 119;
            break;
        case keyCodes.get("A"):
            direction = 97;
            break;
        case keyCodes.get("S"):
            direction = 115;
            break;
        case keyCodes.get("D"):
            direction = 100;
            break;
        case keyCodes.get("U"):
            direction = 117;
            break;
        case keyCodes.get("J"):
            direction = 106;
            break;
        default:
            return; // ignore other keys
        }

        // setMovePos(pos);
        if (keyCode === currentKeyCode) return;
        currentKeyCode = keyCode;

        if (direction === "u" || direction === "j") {
            // Handle up/down only once
            handleMove(direction, true);
            return;
        }

        // Clear previous interval if any
        if (moveInterval) {
            clearInterval(moveInterval);
            moveInterval = null;
        }

        isMoving = true;
        // Call immediately

        const msg = new ROSLIB.Message({
            data: direction
        });

        handleMove(msg, true);

        // Then every 2 seconds
        // moveInterval = setInterval(() => {
        //     handleMove(direction, true);
        // }, 2000);
    }

    // when joystick position needs to be neutral at center
    const keyupHandler = (e) => {
        let keyCode = e.keyCode;
        if ((keyCode === 87 || keyCode === 65 || keyCode === 83 || keyCode === 68) && isMoving && keyCode === currentKeyCode) {
            // setMovePos({ x: 0, y: 0 });
            clearInterval(moveInterval);
            moveInterval = null;
            currentKeyCode = null;
            isMoving = false;
            const msg = new ROSLIB.Message({
                data: 104
            });
            handleMove(msg, true);
        }
    }

    const prevJoystickRef = useRef({ x: null, y: null, buttons: [] });

    const updateJoystick = useCallback(() => {
        const gamepads = navigator.getGamepads();
        const gp = gamepads[0];
        if (gp) {
            const joystickX = gp.axes[0];
            const joystickY = gp.axes[1];
            const joystickZ = gp.axes[3];
            const joystickJ5 = gp.axes[4];
            const joystickJ4 = gp.axes[5];

            const button1 = gp.buttons[0].pressed; // A button
            const button2 = gp.buttons[1].pressed; // B button
            const button3 = gp.buttons[2].pressed; // C button

            // const buttons = gp.buttons.map((b) => b.pressed);
            const buttons = [button1, button2, button3];

            const prev = prevJoystickRef.current;
            const positionChanged = joystickX !== prev.x || joystickY !== prev.y || joystickZ !== prev.z || joystickJ5 !== prev.j5 || joystickJ4 !== prev.j4;
            const buttonsChanged = buttons.some((b, i) => b !== prev.buttons[i]);

            if (positionChanged || buttonsChanged) {
                // Save current state to ref
                prevJoystickRef.current = {
                    x: joystickX,
                    y: joystickY,
                    z: joystickZ,
                    j5: joystickJ5,
                    j4: joystickJ4,
                    buttons: [...buttons],
                };

                setJoystickPosition({ x: joystickX, y: joystickY, z: joystickZ });

                const armData = new ROSLIB.Message({
                    x : joystickX,
                    y : joystickY,
                    z : joystickZ,
                    j5 : joystickJ5,
                    j4 : joystickJ4,
                    buttons : [...buttons]
                });

                console.log(gp);

                if (rosState.connected) {
                    publishMessage(armData);
                }

            }
        }
        requestAnimationFrame(updateJoystick);
    }, [readyState, sendMessage]);

    useEffect(() => {
        const animationFrame = requestAnimationFrame(updateJoystick);
        return () => cancelAnimationFrame(animationFrame);
    }, [updateJoystick]);

    const gamepadConnectHandler = (e) => {
        console.log(
            "Gamepad connected at index %d: %s. %d buttons, %d axes.",
            e.gamepad.index,
            e.gamepad.id,
            e.gamepad.buttons.length,
            e.gamepad.axes.length
        );
    };

    const gamepadDisconnectHandler = (e) => {
        console.log(
            "Gamepad disconnected from index %d: %s",
            e.gamepad.index,
            e.gamepad.id
        );
    };

    // adding handlers for keyboard buttons, e.g. WASD, gamepad connection
    useEffect(() => {
        document.addEventListener('keydown', keydownHandler);
        document.addEventListener('keyup', keyupHandler);
        window.addEventListener("gamepadconnected", gamepadConnectHandler);
        window.addEventListener("gamepaddisconnected", gamepadDisconnectHandler);
        return () => {
            document.removeEventListener('keydown', keydownHandler);
            document.removeEventListener('keyup', keyupHandler);
            window.removeEventListener("gamepadconnected", gamepadConnectHandler);
            window.removeEventListener("gamepaddisconnected", gamepadDisconnectHandler);
        }
    }, [])



    return (
        <>
            <Typography variant="h4" className="mb-4 text-primary font-heading">Manual Control</Typography>
            <div className='flex flex-col gap-4 items-center my-2 pt-4'>
                <div className='w-80 text-center'>
                    <div className='flex justify-center'>
                        <div className="w-20 p-3 text-primary border-2 border-secondary dark:border-white rounded-md" onClick={() => handleMove("W")}>
                            <p>W</p>
                        </div>
                    </div>
                    <div className='flex justify-center my-1'>
                        <div className="w-20 p-3 text-primary border-2 border-secondary dark:border-white rounded-md" onClick={() => handleMove("A")}>
                            <p>A</p>
                        </div>
                        <div className="w-20 p-3 text-primary border-2 border-secondary dark:border-white rounded-md mx-1" onClick={() => handleMove("S")}>
                            <p>S</p>
                        </div>
                        <div className="w-20 p-3 text-primary border-2 border-secondary dark:border-white rounded-md" onClick={() => handleMove("D")}>
                            <p>D</p>
                        </div>
                    </div>
                    <Typography variant="h5" className={`text-black dark:text-white`}>Rover movement</Typography>
                    <JoystickDisplay position={joystickPosition} />
                    <Typography variant="h5" className={`text-black dark:text-white`}>Arm control</Typography>
                </div>
                <Button onClick={() => handleMove("H", true)} color='red' variant='filled'>
                    Emergency Stop
                </Button>

            </div>
        </>
    )
}

export default ControlPanel