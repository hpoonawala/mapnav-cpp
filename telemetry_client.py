#!/usr/bin/env python3
"""
Telemetry Client for Robot Navigation System

Connects to the TelemetryServer running on the robot (Pi) and provides:
- Live visualization of occupancy grid map
- Robot pose marker
- Planned path overlay
- Click-to-set-goal functionality
- Stop/Resume controls

Usage:
    python telemetry_client.py --host <PI_IP> [--port 8765]

Dependencies:
    pip install numpy matplotlib
"""

import socket
import json
import base64
import threading
import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow, Circle
from matplotlib.widgets import Button

class TelemetryClient:
    def __init__(self, host, port=8765):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.running = False

        # Data storage
        self.pose = None
        self.scan_count = 0
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.02
        self.path = []

        # Threading
        self.lock = threading.Lock()
        self.recv_thread = None

    def connect(self):
        """Connect to the telemetry server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(0.1)  # Non-blocking reads
            self.connected = True
            print(f"Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from server."""
        self.running = False
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)

    def start_receiving(self):
        """Start background thread to receive data."""
        self.running = True
        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()

    def _receive_loop(self):
        """Background thread that receives data from server."""
        buffer = ""
        while self.running and self.connected:
            try:
                data = self.socket.recv(65536).decode('utf-8')
                if not data:
                    self.connected = False
                    break
                buffer += data

                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        self._process_message(line.strip())

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Receive error: {e}")
                break

        print("Receive thread ended")

    def _process_message(self, message):
        """Process a single JSON message from server."""
        try:
            data = json.loads(message)
            msg_type = data.get('type')

            with self.lock:
                if msg_type == 'pose':
                    self.pose = data['pose']
                    self.scan_count = data.get('scan_count', 0)

                elif msg_type == 'map':
                    self.map_width = data['width']
                    self.map_height = data['height']
                    self.map_resolution = data['resolution']
                    # Decode base64 map data
                    raw = base64.b64decode(data['data'])
                    self.map_data = np.frombuffer(raw, dtype=np.uint8).reshape(
                        (self.map_height, self.map_width))

                elif msg_type == 'path':
                    self.path = [(wp['x'], wp['y']) for wp in data.get('waypoints', [])]

        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
        except Exception as e:
            print(f"Message processing error: {e}")

    def send_command(self, command_dict):
        """Send a command to the server."""
        if not self.connected:
            return False
        try:
            msg = json.dumps(command_dict) + '\n'
            self.socket.sendall(msg.encode('utf-8'))
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False

    def set_goal(self, x, y):
        """Send a SET_GOAL command."""
        return self.send_command({'command': 'set_goal', 'x': x, 'y': y})

    def stop(self):
        """Send a STOP command."""
        return self.send_command({'command': 'stop'})

    def resume(self):
        """Send a RESUME command."""
        return self.send_command({'command': 'resume'})

    def request_map(self):
        """Request the current map."""
        return self.send_command({'command': 'request_map'})


class TelemetryVisualization:
    def __init__(self, client):
        self.client = client
        self.fig = None
        self.ax = None
        self.map_img = None
        self.robot_marker = None
        self.robot_arrow = None
        self.path_line = None
        self.goal_marker = None

        # Goal set by clicking
        self.pending_goal = None

    def setup(self):
        """Setup the matplotlib figure and widgets."""
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Robot Telemetry')

        # Initialize empty map (gray)
        empty_map = np.ones((100, 100), dtype=np.uint8) * 128
        self.map_img = self.ax.imshow(empty_map, cmap='gray', vmin=0, vmax=255,
                                       origin='upper', extent=[-5, 5, -5, 5])

        # Robot marker (circle)
        self.robot_marker = Circle((0, 0), 0.15, color='blue', fill=True, alpha=0.7)
        self.ax.add_patch(self.robot_marker)

        # Robot direction arrow
        self.robot_arrow = FancyArrow(0, 0, 0.3, 0, width=0.1,
                                       head_width=0.2, head_length=0.1,
                                       fc='red', ec='darkred')
        self.ax.add_patch(self.robot_arrow)

        # Path line
        self.path_line, = self.ax.plot([], [], 'g-', linewidth=2, alpha=0.7)

        # Goal marker
        self.goal_marker, = self.ax.plot([], [], 'r*', markersize=15)

        # Title and labels
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Robot Telemetry - Click to set goal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')

        # Add buttons
        ax_stop = plt.axes([0.7, 0.01, 0.1, 0.04])
        ax_resume = plt.axes([0.81, 0.01, 0.1, 0.04])
        ax_reqmap = plt.axes([0.59, 0.01, 0.1, 0.04])

        self.btn_stop = Button(ax_stop, 'Stop')
        self.btn_resume = Button(ax_resume, 'Resume')
        self.btn_reqmap = Button(ax_reqmap, 'Get Map')

        self.btn_stop.on_clicked(self._on_stop)
        self.btn_resume.on_clicked(self._on_resume)
        self.btn_reqmap.on_clicked(self._on_request_map)

        # Click handler for setting goals
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)

        plt.tight_layout()

    def _on_click(self, event):
        """Handle mouse clicks on the map."""
        if event.inaxes != self.ax:
            return
        if event.button == 1:  # Left click
            x, y = event.xdata, event.ydata
            print(f"Setting goal to ({x:.2f}, {y:.2f})")
            self.pending_goal = (x, y)
            self.client.set_goal(x, y)

    def _on_stop(self, event):
        """Handle Stop button click."""
        print("Sending STOP command")
        self.client.stop()

    def _on_resume(self, event):
        """Handle Resume button click."""
        print("Sending RESUME command")
        self.client.resume()

    def _on_request_map(self, event):
        """Handle Get Map button click."""
        print("Requesting map")
        self.client.request_map()

    def update(self):
        """Update the visualization with latest data."""
        with self.client.lock:
            # Update map
            if self.client.map_data is not None:
                w = self.client.map_width
                h = self.client.map_height
                res = self.client.map_resolution
                # Calculate extent (centered at origin)
                half_w = (w * res) / 2
                half_h = (h * res) / 2
                self.map_img.set_data(self.client.map_data)
                self.map_img.set_extent([-half_w, half_w, -half_h, half_h])

            # Update robot pose
            if self.client.pose is not None:
                x = self.client.pose['x']
                y = self.client.pose['y']
                theta = self.client.pose['theta']

                # Update robot marker
                self.robot_marker.center = (x, y)

                # Update direction arrow
                self.robot_arrow.remove()
                dx = 0.3 * np.cos(theta)
                dy = 0.3 * np.sin(theta)
                self.robot_arrow = FancyArrow(x, y, dx, dy, width=0.08,
                                               head_width=0.15, head_length=0.08,
                                               fc='red', ec='darkred')
                self.ax.add_patch(self.robot_arrow)

                # Update title
                self.ax.set_title(f'Scan: {self.client.scan_count} | '
                                  f'Pose: ({x:.2f}, {y:.2f}, {np.degrees(theta):.1f}deg)')

            # Update path
            if self.client.path:
                xs = [p[0] for p in self.client.path]
                ys = [p[1] for p in self.client.path]
                self.path_line.set_data(xs, ys)
                # Show goal at end of path
                if xs:
                    self.goal_marker.set_data([xs[-1]], [ys[-1]])

            # Show pending goal
            if self.pending_goal:
                self.goal_marker.set_data([self.pending_goal[0]], [self.pending_goal[1]])

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run(self, update_interval=0.2):
        """Main visualization loop."""
        self.setup()

        try:
            while plt.fignum_exists(self.fig.number):
                self.update()
                plt.pause(update_interval)
        except KeyboardInterrupt:
            pass
        finally:
            plt.close(self.fig)


def main():
    parser = argparse.ArgumentParser(description='Robot Telemetry Client')
    parser.add_argument('--host', default='localhost',
                        help='Hostname or IP of the robot (default: localhost)')
    parser.add_argument('--port', type=int, default=8765,
                        help='Port number (default: 8765)')
    args = parser.parse_args()

    client = TelemetryClient(args.host, args.port)

    if not client.connect():
        print("Failed to connect to server")
        sys.exit(1)

    client.start_receiving()

    # Request initial map
    #client.request_map()

    viz = TelemetryVisualization(client)

    try:
        viz.run()
    finally:
        client.disconnect()
        print("Client disconnected")


if __name__ == '__main__':
    main()
