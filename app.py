#!/usr/bin/env python3
"""
STM32 LED Control Flask Backend
Handles web interface requests and STM32 USB serial communication.
This version includes corrected routing to properly serve the web UI.
"""

import os
import time
import json
import threading
from typing import Optional, Dict, Any
from dataclasses import dataclass
from flask import Flask, request, jsonify, send_from_directory, render_template
from flask_cors import CORS

import serial
import serial.tools.list_ports
import logging
from werkzeug.serving import make_server

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class SerialConfig:
    """Serial communication configuration"""
    baudrate: int = 9600
    timeout: float = 1.0
    write_timeout: float = 1.0
    bytesize: int = serial.EIGHTBITS
    parity: str = serial.PARITY_NONE
    stopbits: int = serial.STOPBITS_ONE

class STM32SerialManager:
    """Manages STM32 USB serial communication with automatic recovery"""
    
    def __init__(self, config: SerialConfig):
        self.config = config
        self.serial_port: Optional[serial.Serial] = None
        self.port_path: Optional[str] = None
        self.is_connected = False
        self.lock = threading.Lock()
        self.retry_delay = 2.0
        
    def find_stm32_port(self) -> Optional[str]:
        """Automatically detect STM32 USB CDC port"""
        stm32_identifiers = [
            'STMicroelectronics',
            'STM32',
            'USB Serial Device',
            'CDC'
        ]
        
        for port in serial.tools.list_ports.comports():
            port_desc = (port.description or '').lower()
            port_mfg = (port.manufacturer or '').lower()
            
            for identifier in stm32_identifiers:
                if identifier.lower() in port_desc or identifier.lower() in port_mfg:
                    logger.info(f"Found STM32 device: {port.device} - {port.description}")
                    return port.device
                    
        # Fallback for common ports
        common_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1', 'COM3', 'COM4', 'COM9']
        for port_path in common_ports:
            try:
                # Try to open the port to check if it exists and is available
                s = serial.Serial(port_path)
                s.close()
                logger.info(f"Found available common port: {port_path}")
                return port_path
            except (OSError, serial.SerialException):
                pass
                
        return None
    
    def connect(self) -> bool:
        """Establish serial connection to STM32"""
        with self.lock:
            if self.is_connected:
                return True
            try:
                if not self.port_path:
                    self.port_path = self.find_stm32_port()
                if not self.port_path:
                    logger.error("No STM32 device found.")
                    return False
                
                self.serial_port = serial.Serial(
                    port=self.port_path,
                    baudrate=self.config.baudrate,
                    timeout=self.config.timeout,
                    write_timeout=self.config.write_timeout,
                    bytesize=self.config.bytesize,
                    parity=self.config.parity,
                    stopbits=self.config.stopbits
                )
                time.sleep(2.0) # Wait for the connection to stabilize
                self.is_connected = True
                logger.info(f"Connected to STM32 on {self.port_path}")
                return True
            except serial.SerialException as e:
                logger.error(f"Serial connection failed: {e}")
                self.cleanup()
                return False
            except Exception as e:
                logger.error(f"An unexpected error occurred during connection: {e}")
                self.cleanup()
                return False
    
    def disconnect(self):
        """Safely disconnect from STM32"""
        with self.lock:
            self.cleanup()
    
    def cleanup(self):
        """Clean up serial resources"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                logger.error(f"Error closing serial port: {e}")
        self.serial_port = None
        self.is_connected = False
        self.port_path = None # Reset port path to allow re-detection
        logger.info("Serial connection cleaned up.")
    
    def send_command(self, command: str, max_retries: int = 3) -> Dict[str, Any]:
        """Send command to STM32 with retry logic"""
        for attempt in range(max_retries):
            try:
                with self.lock:
                    if not self.is_connected and not self.connect():
                        raise serial.SerialException("Connection to STM32 is not available.")
                    
                    cmd_bytes = (command + '\n').encode('utf-8')
                    self.serial_port.write(cmd_bytes)
                    self.serial_port.flush()
                    
                    response = self.serial_port.readline().decode('utf-8').strip()
                    if not response:
                        raise serial.SerialTimeoutException("No response from STM32.")
                    
                    logger.info(f"Command '{command}' sent, response: '{response}'")
                    return {'success': True, 'response': response, 'led_state': (response == 'OK_ON')}
            except (serial.SerialException, serial.SerialTimeoutException, ValueError) as e:
                logger.warning(f"Attempt {attempt + 1} failed: {e}. Cleaning up connection.")
                self.cleanup() # Force reconnection on the next attempt
                if attempt < max_retries - 1:
                    time.sleep(self.retry_delay * (attempt + 1)) # Wait before retrying
                else:
                    logger.error(f"Command '{command}' failed after {max_retries} attempts.")
                    return {'success': False, 'error': str(e)}
        return {'success': False, 'error': 'Max retries exceeded'}

class LEDControlServer:
    """Main Flask application for LED control"""
    
    def __init__(self):
        # This tells Flask to look for the 'templates' and 'static' folders
        # in the same directory as the script, which matches your folder structure.
        self.app = Flask(__name__)
        
        CORS(self.app) # Allow all origins for simplicity in local development
        
        self.serial_manager = STM32SerialManager(SerialConfig())
        self.led_state = False # Default state
        self.setup_routes()
        
        threading.Thread(target=self.serial_manager.connect, daemon=True).start()
    
    def setup_routes(self):
        """Configure Flask routes"""
        
        @self.app.route('/')
        def index():
            # Serves the main HTML file from the 'templates' folder.
            return render_template('index.html')
        
        # This single route handles all commands and data requests from the frontend.
        @self.app.route('/<string:endpoint>')
        def handle_requests(endpoint):
            """Handles all API requests from the frontend."""
            command_map = {'glide': 'GLIDE_ON', 'circle': 'CIRCLE_OFF'}
            
            # Handle dashboard data endpoints
            if endpoint == 'speed':
                return jsonify({'speed': 85})
            elif endpoint == 'motor_temperature':
                return jsonify({'temperature': 70})
            elif endpoint == 'battery_temperature':
                return jsonify({'temperature': 45})
            elif endpoint == 'battery_availability':
                return jsonify({'availability': 92})
            elif endpoint == 'status':
                 return "GLIDE_OK" if self.led_state else "CIRCLE_OFF"

            # Handle STM32 commands
            stm32_command = command_map.get(endpoint.lower())
            if not stm32_command:
                # If the endpoint is not a known command or data endpoint, return 404
                return jsonify({'error': 'Invalid endpoint'}), 404

            result = self.serial_manager.send_command(stm32_command)
            if result.get('success'):
                self.led_state = result['led_state']
                # Return the raw response from the STM32 (e.g., "OK_ON" or "OK_OFF")
                return result['response'], 200
            else:
                return jsonify({'error': result.get('error', 'Command failed')}), 503
            
        # --- Error Handlers ---
        @self.app.errorhandler(404)
        def not_found(error):
            return jsonify({'error': 'Not Found'}), 404
    
    def run(self, host='0.0.0.0', port=5000):
        """Run the Flask server"""
        try:
            logger.info(f"Starting server on http://{host}:{port}")
            server = make_server(host, port, self.app, threaded=True)
            server.serve_forever()
        except KeyboardInterrupt:
            logger.info("Server shutting down...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources on shutdown"""
        self.serial_manager.disconnect()

def main():
    """Main entry point"""
    import argparse
    parser = argparse.ArgumentParser(description='STM32 LED Control Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=5000, help='Port to bind to')
    args = parser.parse_args()
    
    server = LEDControlServer()
    server.run(host=args.host, port=args.port)

if __name__ == '__main__':
    main()
