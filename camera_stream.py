#!/usr/bin/env python3

import asyncio
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer, RTCIceCandidate, MediaStreamTrack
# from aiortc.contrib.signaling import smthng
from aiortc.contrib.media import MediaPlayer
import json
import socket

import pyudev
import subprocess
import os

# Store active peer connections
pcs = set()
camera_tracks = []


def is_camera_busy(device="/dev/video0"):
    try:
        # Check if any process is using the device
        output = subprocess.check_output(["fuser", device], stderr=subprocess.DEVNULL)
        return bool(output.strip())  # if output contains any PID, it's busy
    except subprocess.CalledProcessError:
        return False  # fuser returns non-zero when no process uses it

class CameraVideoTrack(MediaStreamTrack):
    def __init__(self, id):
        super().__init__()
        # Determine real device path
        self.device_path = f"/dev/video{id}"

        # Get USB product name using pyudev
        try:
            context = pyudev.Context()
            device = pyudev.Devices.from_device_file(context, self.device_path)
            product = device.properties.get("ID_MODEL", "Unknown")
        except Exception as e:
            print(f"Error getting USB product for {self.device_path}: {e}")
            product = "Unknown"
        
        self.player = None
        
        if product == "USB_Camera":
            self.player = MediaPlayer(
                self.device_path,
                format="v4l2",
                options={
                    "video_size": "640x480",
                    "framerate": "15",
                    "input_format": "mjpeg",
                    "pixel_format": "yuv420p",  # or "yuyv422" depending on your camera
                    "video_range": "full" 
                }
            )
        else:
            self.player = MediaPlayer(self.device_path)  # Webcam on Jetson device

        self.kind = "video"

    async def recv(self):
        frame = await self.player.video.recv()
        return frame

async def websocket_handler(websocket):
    """Handle WebSocket signaling."""
    global camera_tracks, pcs
    print("Client connected")
    configuration = RTCConfiguration(iceServers=[RTCIceServer(urls='stun:stun.l.google.com:19302')])
    # configuration = RTCConfiguration(iceServers=[])
    # configuration = RTCConfiguration(iceServers=[RTCIceServer(urls='turn:openrelay.metered.ca:80', username='openrelayproject', credentials='openrelayproject')])
    pc = RTCPeerConnection(configuration=configuration)
    pcs.add(pc)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print(f"ICE connection state: {pc.iceConnectionState}")
        if pc.iceConnectionState in ["closed", "failed", "disconnected"]:
            print("ICE connection closed or failed. Cleaning up...")
            await pc.close()
            pcs.discard(pc)

    try:
        for i in range(4):
            if not is_camera_busy(f"/dev/video{i*2}") and os.path.exists(f"/dev/video{i*2}"):
                camera_track = CameraVideoTrack(i*2)
                camera_tracks.append(camera_track)
                print(f"Camera {i*2} is available.")
            else:
                print(f"Camera {i*2}is currently busy.")
    
        for track in camera_tracks:
            pc.addTrack(track)

        # Create SDP offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        # Send the offer to the client
        offer_json = {"webrtc_offer": {
            "type": offer.type, "sdp": offer.sdp
            }
        }

        await websocket.send(json.dumps(offer_json))

        # Wait for the client's SDP answer
        answer_json = await websocket.recv()
        parsed_answer_json = json.loads(answer_json)  # First parse the JSON string
        answer1 = parsed_answer_json["message"]
        answer2 = json.loads(answer1)
        answer = answer2["webrtc_answer"]
        
        answer = RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
        await pc.setRemoteDescription(answer)

        # Keep connection open to process ICE candidates
        # Handle ICE candidates from the client
        while True:
            try:
                message = await websocket.recv()
                print(f"First Received: {message}")
                data = json.loads(message)  # Parse the top-level JSON object
                data1 = data["message"]
                data2 = json.loads(data1)  # Parse the nested JSON in "message"
                data3 = data2["webrtc_candidate"]  # This is already a dictionary
                print(f"Second Received: {data3}")

                if pc.iceConnectionState not in ["closed", "failed"]:
                    if "candidate" in data3:
                        candidate = candidate_from_sdp(data3)
                        await pc.addIceCandidate(candidate)
                    else:
                        print("No valid candidate found.")
                else:
                    print(f"Skipping candidate; ICE connection state is {pc.iceConnectionState}.")
            except Exception as e:
                print(f"Error while handling ICE candidate: {e}")
                break


    except websockets.ConnectionClosed:
        print("Client disconnected")
    finally:
        await pc.close()
        pcs.discard(pc)

def candidate_from_sdp(cand) -> RTCIceCandidate:
    print("candidate recieved: ", cand)
    sdp = cand["candidate"]
    bits = sdp.split()
    assert len(bits) >= 8
    ip = bits[4]
    resolved_ip = socket.gethostbyname(ip)

    print(f"IP: {ip}\n")
    print(f"RESOLVED_IP: {resolved_ip}")

    candidate = RTCIceCandidate(
        component=int(bits[1]),
        foundation=bits[0],
        ip=resolved_ip,
        port=int(bits[5]),
        priority=int(bits[3]),
        protocol=bits[2],
        type=bits[7],
        sdpMid=cand["sdpMid"],
        sdpMLineIndex=cand["sdpMLineIndex"],
    )

    for i in range(8, len(bits) - 1, 2):
        if bits[i] == "raddr":
            candidate.relatedAddress = bits[i + 1]
        elif bits[i] == "rport":
            candidate.relatedPort = int(bits[i + 1])
        elif bits[i] == "tcptype":
            candidate.tcpType = bits[i + 1]
    print("candidate after processing: ", candidate)
    return candidate

async def cleanup():
    """Cleanup peer connections on shutdown."""
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

async def async_main():
    print("Starting WebSocket server on ws://0.0.0.0:8080")
    async with websockets.serve(websocket_handler, "0.0.0.0", 8080):
        await asyncio.Future()  # Run forever

def main(args=None):
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        print("Shutting down...")
        asyncio.run(cleanup())

if __name__ == '__main__':
    main()