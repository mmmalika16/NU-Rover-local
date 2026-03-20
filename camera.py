
# import asyncio
# import websockets
# from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer, RTCIceCandidate, MediaStreamTrack
# from aiortc.contrib.media import MediaPlayer
# import json
# import socket
# import pyudev
# import subprocess
# import os
# import threading

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# # Store active peer connections
# pcs = set()
# camera_tracks = []


# def is_camera_busy(device="/dev/video0"):
#     try:
#         # Check if any process is using the device
#         output = subprocess.check_output(["fuser", device], stderr=subprocess.DEVNULL)
#         return bool(output.strip())  # if output contains any PID, it's busy
#     except subprocess.CalledProcessError:
#         return False  # fuser returns non-zero when no process uses it


# class CameraVideoTrack(MediaStreamTrack):
#     def __init__(self, id):
#         super().__init__()
#         # Determine real device path
#         self.device_path = f"/dev/video{id}"

#         # Get USB product name using pyudev
#         try:
#             context = pyudev.Context()
#             device = pyudev.Devices.from_device_file(context, self.device_path)
#             product = device.properties.get("ID_MODEL", "Unknown")
#         except Exception as e:
#             print(f"Error getting USB product for {self.device_path}: {e}")
#             product = "Unknown"
        
#         self.player = None
        
#         if product == "USB_Camera":
#             self.player = MediaPlayer(
#                 self.device_path,
#                 format="v4l2",
#                 options={
#                     "video_size": "640x480",
#                     "framerate": "15",
#                     "input_format": "mjpeg",
#                     "pixel_format": "yuv420p",
#                     "video_range": "full" 
#                 }
#             )
#         else:
#             self.player = MediaPlayer(self.device_path)  # Webcam on Jetson device

#         self.kind = "video"

#     async def recv(self):
#         frame = await self.player.video.recv()
#         return frame


# class WebRTCCameraNode(Node):
#     def __init__(self):
#         super().__init__('webrtc_camera_node')
        
#         # Declare parameters
#         self.declare_parameter('websocket_host', '0.0.0.0')
#         self.declare_parameter('websocket_port', 8080)
#         self.declare_parameter('stun_server', 'stun:stun.l.google.com:19302')
        
#         # Get parameters
#         self.ws_host = self.get_parameter('websocket_host').value
#         self.ws_port = self.get_parameter('websocket_port').value
#         self.stun_server = self.get_parameter('stun_server').value
        
#         # Publisher for status updates
#         self.status_publisher = self.create_publisher(String, 'webrtc_status', 10)
        
#         # Start WebSocket server in a separate thread
#         self.ws_thread = threading.Thread(target=self.run_websocket_server, daemon=True)
#         self.ws_thread.start()
        
#         self.get_logger().info(f'WebRTC Camera Node started on ws://{self.ws_host}:{self.ws_port}')
#         self.publish_status('WebRTC server started')

#     def publish_status(self, message):
#         """Publish status message to ROS topic."""
#         msg = String()
#         msg.data = message
#         self.status_publisher.publish(msg)
#         self.get_logger().info(f'Status: {message}')

#     def run_websocket_server(self):
#         """Run the WebSocket server in asyncio event loop."""
#         asyncio.run(self.start_server())

#     async def start_server(self):
#         """Start the WebSocket server."""
#         async with websockets.serve(self.websocket_handler, self.ws_host, self.ws_port):
#             await asyncio.Future()  # Run forever

#     async def websocket_handler(self, websocket):
#         """Handle WebSocket signaling."""
#         global camera_tracks, pcs
#         self.get_logger().info("Client connected")
#         self.publish_status("Client connected")
        
#         configuration = RTCConfiguration(iceServers=[RTCIceServer(urls=self.stun_server)])
#         pc = RTCPeerConnection(configuration=configuration)
#         pcs.add(pc)

#         @pc.on("iceconnectionstatechange")
#         async def on_iceconnectionstatechange():
#             self.get_logger().info(f"ICE connection state: {pc.iceConnectionState}")
#             if pc.iceConnectionState in ["closed", "failed", "disconnected"]:
#                 self.get_logger().warn("ICE connection closed or failed. Cleaning up...")
#                 self.publish_status(f"ICE connection {pc.iceConnectionState}")
#                 await pc.close()
#                 pcs.discard(pc)

#         try:
#             # Clear previous camera tracks
#             camera_tracks.clear()
            
#             for i in range(4):
#                 if i == 1:
#                     continue

#                 device_path = f"/dev/video{i*2}"
#                 if not is_camera_busy(device_path) and os.path.exists(device_path):
#                     camera_track = CameraVideoTrack(i*2)
#                     camera_tracks.append(camera_track)
#                     self.get_logger().info(f"Camera {i*2} is available.")
#                 else:
#                     self.get_logger().warn(f"Camera {i*2} is busy or not available.")
        
#             for track in camera_tracks:
#                 pc.addTrack(track)

#             # Create SDP offer
#             offer = await pc.createOffer()
#             await pc.setLocalDescription(offer)

#             # Send the offer to the client
#             offer_json = {"webrtc_offer": {
#                 "type": offer.type, "sdp": offer.sdp
#                 }
#             }

#             await websocket.send(json.dumps(offer_json))
#             self.get_logger().info("Sent WebRTC offer to client")

#             # Wait for the client's SDP answer
#             answer_json = await websocket.recv()
#             parsed_answer_json = json.loads(answer_json)
#             answer1 = parsed_answer_json["message"]
#             answer2 = json.loads(answer1)
#             answer = answer2["webrtc_answer"]
            
#             answer = RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
#             await pc.setRemoteDescription(answer)
#             self.get_logger().info("Received and set remote description")

#             # Handle ICE candidates from the client
#             while True:
#                 try:
#                     message = await websocket.recv()
#                     data = json.loads(message)
#                     data1 = data["message"]
#                     data2 = json.loads(data1)
#                     data3 = data2["webrtc_candidate"]

#                     if pc.iceConnectionState not in ["closed", "failed"]:
#                         if "candidate" in data3:
#                             candidate = self.candidate_from_sdp(data3)
#                             await pc.addIceCandidate(candidate)
#                             self.get_logger().debug("Added ICE candidate")
#                         else:
#                             self.get_logger().warn("No valid candidate found.")
#                     else:
#                         self.get_logger().warn(f"Skipping candidate; ICE connection state is {pc.iceConnectionState}.")
#                 except Exception as e:
#                     self.get_logger().error(f"Error while handling ICE candidate: {e}")
#                     break

#         except websockets.ConnectionClosed:
#             self.get_logger().info("Client disconnected")
#             self.publish_status("Client disconnected")
#         except Exception as e:
#             self.get_logger().error(f"Error in websocket handler: {e}")
#         finally:
#             await pc.close()
#             pcs.discard(pc)

#     def candidate_from_sdp(self, cand) -> RTCIceCandidate:
#         """Parse ICE candidate from SDP."""
#         sdp = cand["candidate"]



#         bits = sdp.split()
#         assert len(bits) >= 8
#         ip = bits[4]
#         resolved_ip = socket.gethostbyname(ip)

#         self.get_logger().debug(f"Resolved IP: {ip} -> {resolved_ip}")

#         candidate = RTCIceCandidate(
#             component=int(bits[1]),
#             foundation=bits[0],
#             ip=resolved_ip,
#             port=int(bits[5]),
#             priority=int(bits[3]),
#             protocol=bits[2],
#             type=bits[7],
#             sdpMid=cand["sdpMid"],
#             sdpMLineIndex=cand["sdpMLineIndex"],
#         )

#         for i in range(8, len(bits) - 1, 2):
#             if bits[i] == "raddr":
#                 candidate.relatedAddress = bits[i + 1]
#             elif bits[i] == "rport":
#                 candidate.relatedPort = int(bits[i + 1])
#             elif bits[i] == "tcptype":
#                 candidate.tcpType = bits[i + 1]
        
#         return candidate


# def main(args=None):
#     rclpy.init(args=args)
#     node = WebRTCCameraNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # Cleanup
#         node.get_logger().info("Shutting down WebRTC Camera Node...")
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import asyncio
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer, RTCIceCandidate, MediaStreamTrack
from aiortc.contrib.media import MediaPlayer
import json
import socket
import pyudev
import subprocess
import os
import threading
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
        self.track_id = f"camera_{id}"
        self.kind = "video"
        self.player = None 

        # Get USB product name using pyudev
        try:
            context = pyudev.Context()
            device = pyudev.Devices.from_device_file(context, self.device_path)
            product = device.properties.get("ID_MODEL", "Unknown")
        except Exception as e:
            print(f"Error getting USB product for {self.device_path}: {e}")
            product = "Unknown"
        
        if product == "USB_Camera":
            self.player = MediaPlayer(
                self.device_path,
                format="v4l2",
                options={
                    "video_size": "640x480",
                    "framerate": "15",
                    "input_format": "mjpeg",
                    "pixel_format": "yuv420p",
                    "video_range": "full" 
                }
            )
        else:
            self.player = MediaPlayer(self.device_path)  # Webcam on Jetson device

    async def recv(self):
        if not self.player or not self.player.video:
            raise RuntimeError("Camera player not initialized or video track missing")
        frame = await self.player.video.recv()
        return frame
    
    async def stop(self):
        if self.player:
            try:
                await self.player.stop()
            except Exception as e:
                print(f"[CameraVideoTrack] Error stopping MediaPlayer: {e}")
            self.player = None

class WebRTCCameraNode(Node):
    def __init__(self):
        super().__init__('webrtc_camera_node')
        
        # Declare parameters
        self.declare_parameter('websocket_host', '0.0.0.0')
        self.declare_parameter('websocket_port', 8080)
        self.declare_parameter('stun_server', 'stun:stun.l.google.com:19302')
        
        # Get parameters
        self.ws_host = self.get_parameter('websocket_host').value
        self.ws_port = self.get_parameter('websocket_port').value
        self.stun_server = self.get_parameter('stun_server').value
        
        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, 'webrtc_status', 10)
        
        # Start WebSocket server in a separate thread
        self.ws_thread = threading.Thread(target=self.run_websocket_server, daemon=True)
        self.ws_thread.start()
        
        self.get_logger().info(f'WebRTC Camera Node started on ws://{self.ws_host}:{self.ws_port}')
        self.publish_status('WebRTC server started')

    def publish_status(self, message):
        """Publish status message to ROS topic."""
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Status: {message}')

    def run_websocket_server(self):
        """Run the WebSocket server in asyncio event loop."""
        asyncio.run(self.start_server())

    async def start_server(self):
        """Start the WebSocket server."""
        async with websockets.serve(self.websocket_handler, self.ws_host, self.ws_port):
            await asyncio.Future()  # Run forever

    async def websocket_handler(self, websocket):
        """Handle WebSocket signaling."""
        global camera_tracks, pcs
        self.get_logger().info("Client connected")
        self.publish_status("Client connected")
        
        configuration = RTCConfiguration(iceServers=[RTCIceServer(urls=self.stun_server)])
        pc = RTCPeerConnection(configuration=configuration)
        pcs.add(pc)

        @pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            self.get_logger().info(f"ICE connection state: {pc.iceConnectionState}")
            if pc.iceConnectionState in ["closed", "failed", "disconnected"]:
                self.get_logger().warn("ICE connection closed or failed. Cleaning up...")
                self.publish_status(f"ICE connection {pc.iceConnectionState}")
                await pc.close()
                pcs.discard(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            self.get_logger().info(f"Connection state: {pc.connectionState}")
            if pc.connectionState == "failed":
                self.get_logger().error("Connection failed")
                await pc.close()
                pcs.discard(pc)

        @pc.on("icegatheringstatechange")
        async def on_icegatheringstatechange():
            self.get_logger().info(f"ICE gathering state: {pc.iceGatheringState}")

        try:
            # Clear previous camera tracks
            camera_tracks.clear()
            
            # Add available camera tracks
            tracks_added = False
            for i in range(4):
                if i == 1:
                    continue

                device_path = f"/dev/video{i*2}"
                if not is_camera_busy(device_path) and os.path.exists(device_path):
                    camera_track = CameraVideoTrack(i*2)
                    camera_tracks.append(camera_track)
                    pc.addTrack(camera_track)
                    tracks_added = True
                    self.get_logger().info(f"Camera {i*2} is available and added to track.")
                else:
                    self.get_logger().warn(f"Camera {i*2} is busy or not available.")
            
            if not tracks_added:
                self.get_logger().error("No cameras available to stream")
                return

            # Create SDP offer
            self.get_logger().info("Creating SDP offer...")
            offer = await pc.createOffer()
            
            # Set local description
            self.get_logger().info("Setting local description...")
            await pc.setLocalDescription(offer)
            
            # Wait for ICE gathering to complete (important for BUNDLE grouping)
            self.get_logger().info("Waiting for ICE gathering to complete...")
            
            # Wait for ice gathering state to be complete
            while pc.iceGatheringState != "complete":
                await asyncio.sleep(0.1)
            
            self.get_logger().info("ICE gathering complete")
            
            # Get the final local description after ICE gathering
            final_offer = pc.localDescription
            
            # Log SDP for debugging
            self.get_logger().debug(f"Final offer SDP: {final_offer.sdp}")
            
            # Send the offer to the client
            offer_json = {
                "webrtc_offer": {
                    "type": final_offer.type, 
                    "sdp": final_offer.sdp
                }
            }

            await websocket.send(json.dumps(offer_json))
            self.get_logger().info("Sent WebRTC offer to client")

            # Wait for the client's SDP answer
            self.get_logger().info("Waiting for client answer...")
            answer_json = await websocket.recv()
            parsed_answer_json = json.loads(answer_json)
            
            self.get_logger().info(f"Received answer structure: {parsed_answer_json.keys()}")
            
            # Handle different possible answer structures
            answer_data = None
            
            if "message" in parsed_answer_json:
                try:
                    answer1 = parsed_answer_json["message"]
                    answer2 = json.loads(answer1)
                    if "webrtc_answer" in answer2:
                        answer_data = answer2["webrtc_answer"]
                    else:
                        self.get_logger().error(f"No webrtc_answer in nested message: {answer2.keys()}")
                        return
                except json.JSONDecodeError as e:
                    self.get_logger().error(f"Failed to parse answer message: {e}")
                    return
            elif "webrtc_answer" in parsed_answer_json:
                answer_data = parsed_answer_json["webrtc_answer"]
            else:
                self.get_logger().error(f"Unknown answer structure: {parsed_answer_json.keys()}")
                return
                
            self.get_logger().info(f"Received answer of type: {answer_data['type']}")
            
            # Create RTCSessionDescription from answer
            remote_desc = RTCSessionDescription(sdp=answer_data["sdp"], type=answer_data["type"])
            
            # Set remote description
            self.get_logger().info("Setting remote description...")
            await pc.setRemoteDescription(remote_desc)
            self.get_logger().info("Successfully set remote description")

            # Handle ICE candidates from the client
            while True:
                try:
                    message = await websocket.recv()
                    data = json.loads(message)
                    
                    # Log the full message structure for debugging
                    self.get_logger().debug(f"Received message: {json.dumps(data)}")
                    
                    # Handle messages with action field
                    if "action" in data:
                        action = data["action"]
                        self.get_logger().debug(f"Received action: {action}")
                        
                        if action == "candidate":
                            # This is an ICE candidate message
                            if "message" in data:
                                try:
                                    candidate_data = json.loads(data["message"])
                                    
                                    if pc.iceConnectionState not in ["closed", "failed"]:
                                        if "candidate" in candidate_data and candidate_data["candidate"]:
                                            try:
                                                candidate = self.candidate_from_sdp(candidate_data)
                                                await pc.addIceCandidate(candidate)
                                                self.get_logger().debug("Successfully added ICE candidate")
                                            except Exception as e:
                                                self.get_logger().error(f"Failed to add ICE candidate: {e}")
                                        else:
                                            # Check for end-of-candidates
                                            if "candidate" in candidate_data and candidate_data["candidate"] == "":
                                                self.get_logger().debug("Received end-of-candidates signal")
                                            else:
                                                self.get_logger().warn(f"No valid candidate in: {candidate_data}")
                                    else:
                                        self.get_logger().warn(f"Skipping candidate; ICE state: {pc.iceConnectionState}")
                                except json.JSONDecodeError as e:
                                    self.get_logger().error(f"Failed to parse candidate message: {e}")
                            else:
                                self.get_logger().warn("Candidate action message has no 'message' field")
                        else:
                            self.get_logger().warn(f"Unknown action: {action}")
                    else:
                        self.get_logger().warn(f"Message without action field: {data.keys()}")
                        
                except websockets.ConnectionClosed:
                    self.get_logger().info("WebSocket connection closed")
                    break
                except Exception as e:
                    self.get_logger().error(f"Error while handling message: {e}")
                    traceback.print_exc()
                    # Don't break on error, continue listening
                    continue

        except websockets.ConnectionClosed:
            self.get_logger().info("Client disconnected")
            self.publish_status("Client disconnected")
        except Exception as e:
            self.get_logger().error(f"Error in websocket handler: {e}")
            traceback.print_exc()
        finally:
            self.get_logger().info("Cleaning up peer connection")
            for track in camera_tracks:
                await track.stop()
            camera_tracks.stop()
            await pc.close()
            pcs.discard(pc)

    def candidate_from_sdp(self, cand) -> RTCIceCandidate:
        """Parse ICE candidate from SDP with robust IP/hostname handling."""
        try:
            sdp = cand["candidate"]
            bits = sdp.split()
            
            if len(bits) < 8:
                self.get_logger().error(f"Invalid candidate format: {sdp}")
                raise ValueError(f"Invalid candidate format: too few fields")
                
            ip = bits[4]
            
            # Try to resolve the IP, but handle failures gracefully
            resolved_ip = self._resolve_ip(ip)
            
            self.get_logger().debug(f"Using IP: {resolved_ip} for candidate")

            candidate = RTCIceCandidate(
                component=int(bits[1]),
                foundation=bits[0],
                ip=resolved_ip,
                port=int(bits[5]),
                priority=int(bits[3]),
                protocol=bits[2],
                type=bits[7],
                sdpMid=cand.get("sdpMid", ""),
                sdpMLineIndex=cand.get("sdpMLineIndex", 0),
            )

            # Parse additional attributes
            for i in range(8, len(bits) - 1, 2):
                if bits[i] == "raddr":
                    candidate.relatedAddress = bits[i + 1]
                elif bits[i] == "rport":
                    candidate.relatedPort = int(bits[i + 1])
                elif bits[i] == "tcptype":
                    candidate.tcpType = bits[i + 1]
            
            return candidate
            
        except Exception as e:
            self.get_logger().error(f"Error parsing candidate from SDP: {e}")
            raise

    def _resolve_ip(self, ip):
        """Resolve hostname to IP address, handling IPv4, IPv6, and resolution failures."""
        # Check if it's already a valid IPv4 address
        try:
            socket.inet_pton(socket.AF_INET, ip)
            return ip  # It's already a valid IPv4 address
        except socket.error:
            pass
        
        # Check if it's a valid IPv6 address
        try:
            socket.inet_pton(socket.AF_INET6, ip)
            return ip  # It's already a valid IPv6 address
        except socket.error:
            pass
        
        # It's a hostname, try to resolve it
        try:
            # Try IPv4 resolution first
            resolved_ip = socket.gethostbyname(ip)
            self.get_logger().debug(f"Resolved hostname: {ip} -> {resolved_ip}")
            return resolved_ip
        except socket.gaierror as e:
            self.get_logger().warn(f"Could not resolve hostname {ip}: {e}")
            # Return the original hostname as fallback
            return ip


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.get_logger().info("Shutting down WebRTC Camera Node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()