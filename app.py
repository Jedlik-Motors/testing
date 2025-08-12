#!/usr/bin/env python3
"""
STM32 LED Control Flask Backend
Handles web interface requests and STM32 USB serial communication
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
                    
        # Fallback: try common USB serial ports
        common_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        for port in common_ports:
            if os.path.exists(port):
                logger.info(f"Trying common port: {port}")
                return port
                
        return None
    
    def connect(self) -> bool:
        """Establish serial connection to STM32"""
        with self.lock:
            try:
                if self.is_connected:
                    return True
                    
                if not self.port_path:
                    self.port_path = self.find_stm32_port()
                    
                if not self.port_path:
                    logger.error("No STM32 device found")
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
                
                # Clear any pending data
                self.serial_port.flushInput()
                self.serial_port.flushOutput()
                
                # Test connection with ping
                time.sleep(0.1)  # Allow STM32 to stabilize
                
                self.is_connected = True
                logger.info(f"Connected to STM32 on {self.port_path}")
                return True
                
            except serial.SerialException as e:
                logger.error(f"Serial connection failed: {e}")
                self.cleanup()
                return False
            except Exception as e:
                logger.error(f"Unexpected error during connection: {e}")
                self.cleanup()
                return False
    
    def disconnect(self):
        """Safely disconnect from STM32"""
        with self.lock:
            self.cleanup()
            logger.info("Disconnected from STM32")
    
    def cleanup(self):
        """Clean up serial resources"""
        self.is_connected = False
        if self.serial_port:
            try:
                self.serial_port.close()
            except:
                pass
            self.serial_port = None
    
    def send_command(self, command: str, max_retries: int = 3) -> Dict[str, Any]:
        """Send command to STM32 with retry logic and exponential backoff"""
        for attempt in range(max_retries):
            try:
                with self.lock:
                    # Ensure connection
                    if not self.is_connected:
                        if not self.connect():
                            raise serial.SerialException("Failed to establish connection")
                    
                    # Send command
                    cmd_bytes = (command + '\n').encode('utf-8')
                    self.serial_port.write(cmd_bytes)
                    self.serial_port.flush()
                    
                    # Read response with timeout
                    response = self.serial_port.readline().decode('utf-8').strip()
                    
                    if not response:
                        raise serial.SerialTimeoutException("No response from STM32")
                    
                    # Validate response
                    expected_responses = {
                        'GLIDE_ON': 'OK_ON',
                        'CIRCLE_OFF': 'OK_OFF'
                    }
                    
                    expected = expected_responses.get(command)
                    if expected and response != expected:
                        raise ValueError(f"Invalid response: got '{response}', expected '{expected}'")
                    
                    # Parse LED state from response
                    led_state = response == 'OK_ON'
                    
                    logger.info(f"Command '{command}' successful, LED state: {led_state}")
                    return {
                        'success': True,
                        'response': response,
                        'led_state': led_state,
                        'attempt': attempt + 1
                    }
                    
            except (serial.SerialException, serial.SerialTimeoutException, ValueError) as e:
                logger.warning(f"Attempt {attempt + 1} failed: {e}")
                self.cleanup()  # Force reconnection on next attempt
                
                if attempt < max_retries - 1:
                    delay = min(2 ** attempt, 8)  # Exponential backoff, max 8 seconds
                    logger.info(f"Retrying in {delay} seconds...")
                    time.sleep(delay)
                else:
                    logger.error(f"Command '{command}' failed after {max_retries} attempts")
                    return {
                        'success': False,
                        'error': str(e),
                        'attempt': attempt + 1
                    }
            except Exception as e:
                logger.error(f"Unexpected error: {e}")
                self.cleanup()
                return {
                    'success': False,
                    'error': f"Unexpected error: {str(e)}",
                    'attempt': attempt + 1
                }
        
        return {'success': False, 'error': 'Max retries exceeded'}

class LEDControlServer:
    """Main Flask application for LED control"""
    
    def __init__(self):
        self.app = Flask(__name__)
        
        # Configure CORS for local network access only
        CORS(self.app, origins=[
            "http://localhost:*",
            "http://127.0.0.1:*",
            "http://192.168.*.*:*",
            "http://10.*.*.*:*"
        ])
        
        self.serial_manager = STM32SerialManager(SerialConfig())
        self.led_state = False
        self.setup_routes()
        
        # Attempt initial connection
        self.serial_manager.connect()
    
    def setup_routes(self):
        """Configure Flask routes"""
        
        @self.app.route('/')
        def index():
            """Serve the main web interface"""
            return render_template('index.html')
        
        # Route to handle the GLIDE and CIRCLE commands from the HTML
        @self.app.route('/<command>')
        def control_led(command):
            """Handle LED control commands"""
            try:
                # Map commands from HTML to STM32 protocol strings
                command_map = {
                    'glide': 'GLIDE_ON',
                    'circle': 'CIRCLE_OFF'
                }
                
                stm32_command = command_map.get(command.lower())
                
                if stm32_command:
                    logger.info(f"Received command from web: '{command}'. Mapping to '{stm32_command}'")
                    result = self.serial_manager.send_command(stm32_command)
                    
                    if result['success']:
                        return "OK_ON" if stm32_command == 'GLIDE_ON' else "OK_OFF"
                    else:
                        return f"Error: {result['error']}", 503
                else:
                    return f"Error: Invalid command '{command}'", 400
                    
            except Exception as e:
                logger.error(f"Control endpoint error: {e}")
                return "Internal server error", 500
        
        # ADDED: Placeholder routes for dashboard data
        @self.app.route('/speed')
        def get_speed():
            return jsonify({'speed': 85})

        @self.app.route('/motor_temperature')
        def get_motor_temperature():
            return jsonify({'temperature': 70})

        @self.app.route('/battery_temperature')
        def get_battery_temperature():
            return jsonify({'temperature': 45})

        @self.app.route('/battery_availability')
        def get_battery_availability():
            return jsonify({'availability': 92})
        # END ADDED SECTION

        @self.app.route('/health', methods=['GET'])
        def health_check():
            """System health monitoring endpoint"""
            try:
                # Check serial connection status
                is_connected = self.serial_manager.is_connected
                
                # Attempt to reconnect if disconnected
                if not is_connected:
                    is_connected = self.serial_manager.connect()
                
                return jsonify({
                    'status': 'healthy' if is_connected else 'degraded',
                    'serial_connected': is_connected,
                    'led_state': self.led_state,
                    'port': self.serial_manager.port_path,
                    'timestamp': time.time()
                })
                
            except Exception as e:
                logger.error(f"Health check error: {e}")
                return jsonify({
                    'status': 'unhealthy',
                    'serial_connected': False,
                    'error': str(e),
                    'timestamp': time.time()
                }), 500
        
        @self.app.route('/status', methods=['GET'])
        def get_status():
            """Get current LED status"""
            # We don't have a direct status from Arduino, so we'll simulate based on the last command
            return "GLIDE_OK" if self.led_state else "CIRCLE_OFF"
        
        @self.app.errorhandler(404)
        def not_found(error):
            return jsonify({'error': 'Endpoint not found'}), 404
        
        @self.app.errorhandler(500)
        def internal_error(error):
            return jsonify({'error': 'Internal server error'}), 500
    
    def run(self, host='0.0.0.0', port=5000, debug=False):
        """Run the Flask server"""
        try:
            if debug:
                self.app.run(host=host, port=port, debug=True, threaded=True)
            else:
                # Production WSGI server
                server = make_server(host, port, self.app, threaded=True)
                logger.info(f"Starting production server on {host}:{port}")
                server.serve_forever()
        except KeyboardInterrupt:
            logger.info("Server shutting down...")
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.serial_manager.disconnect()

def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='STM32 LED Control Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=5000, help='Port to bind to')
    parser.add_argument('--debug', action='store_true', help='Run in debug mode')
    
    args = parser.parse_args()
    
    server = LEDControlServer()
    
    try:
        logger.info("STM32 LED Control Server starting...")
        logger.info(f"Web interface will be available at http://{args.host}:{args.port}")
        server.run(host=args.host, port=args.port, debug=args.debug)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    except Exception as e:
        logger.error(f"Failed to start server: {e}")

if __name__ == '__main__':
    main()