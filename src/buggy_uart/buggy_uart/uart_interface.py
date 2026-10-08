#!/usr/bin/env python3

import serial
import logging
import os
from datetime import datetime
from time import sleep
# Protocol Constants
PROTOCOL_START = 40  # '(' ASCII
PROTOCOL_END = 41    # ')' ASCII
OFFSET = -30
RESOLUTION = 0.01
DEGREE_RANGE = (-20, 20)

class UartInterface:
    """
    UART Interface for sending structured messages to hardware
    according to the specified protocol.
    """
    def __init__(self, port, baudrate=115200, timeout=1, ros_logger=None):
        """
        Initialize the UART interface.
        
        Args:
            port (str): Serial port name (e.g., /dev/ttyTHS0)
            baudrate (int): Baud rate for serial communication
            timeout (float): Read/write timeout in seconds
            ros_logger: Optional ROS 2 logger (rclpy node logger) for logging
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.ros_logger = ros_logger
        
        # Set up logging to both file and console
        log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
        os.makedirs(log_dir, exist_ok=True)
        
        log_file = os.path.join(log_dir, f"uart_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        
        # Configure root logger
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.INFO)
        
        # Remove existing handlers
        for handler in root_logger.handlers[:]:
            root_logger.removeHandler(handler)
            
        # Add file handler
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        root_logger.addHandler(file_handler)
        
        # Add console handler
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        root_logger.addHandler(console_handler)
        
        # Initialize connection
        self._connect()

    def _connect(self):
        """Establish connection to the serial port"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            if self.serial_conn.is_open:
                logging.info(f"Connected to UART port {self.port} at {self.baudrate} baud")
            else:
                logging.error(f"Failed to open UART port {self.port}")
                
        except serial.SerialException as e:
            logging.error(f"Failed to connect to UART port {self.port}: {e}")
    def create_message(self, yellow_detected=False, red_detected=False, degree=0):
        """
        Create a message following the protocol.

        6 bytes, no critical-flag byte: matches the original ROS1 protocol
        the MCU firmware actually expects (hardware-confirmed - a 7-byte
        packet with an extra DMS critical-flag byte inserted before END
        was never accepted/acted on by the buggy; a standalone non-ROS
        pyserial test with that byte removed was).

        Args:
            yellow_detected (bool): Yellow object detection status
            red_detected (bool): Red object detection status
            degree (float): Steering angle in degrees

        Returns:
            bytearray: Formatted message according to protocol
        """
        # Clamp degree value to the specified range
        degree = max(min(degree, DEGREE_RANGE[1]), DEGREE_RANGE[0])

        # Calculate the raw value using the formula
        raw_value = int((degree - OFFSET) / RESOLUTION)

        # Ensure raw value is within 16-bit signed integer range
        raw_value = max(min(raw_value, 32767), -32768)

        # Convert raw value to bytes (2 bytes for 16-bit signed value)
        raw_value_bytes = raw_value.to_bytes(2, byteorder='big', signed=True)

        yellow_zone = 1 if yellow_detected else 0
        red_zone = 1 if red_detected else 0

        # Build the message as a bytearray - 6 bytes, matching the
        # original ROS1 protocol the MCU firmware actually expects.
        message = bytearray()
        message.append(PROTOCOL_START)  # Start byte '('
        message.append(yellow_zone)     # Yellow detection
        message.append(red_zone)        # Red detection
        message.extend(raw_value_bytes) # Raw value bytes (2 bytes)
        message.append(PROTOCOL_END)    # End byte ')'

        return message

    def send(self, yellow_detected, red_detected, degree):
        """
        Send a message over UART.

        Args:
            yellow_detected (bool): Yellow object detection status
            red_detected (bool): Red object detection status
            degree (float): Steering angle in degrees

        Returns:
            bool: True if message sent successfully, False otherwise
        """
        message = self.create_message(yellow_detected, red_detected, degree)
        
        if self.ros_logger:
            self.ros_logger.info(f"Sending UART - angle: {degree:.2f}, yellow: {yellow_detected}, red: {red_detected}")
        # Log in hex and ASCII format
        ascii_str = ''.join(chr(byte) if 32 <= byte <= 126 else f'\\x{byte:02x}' for byte in message)
        logging.info(f"Message packet (hex): {message.hex()}")
        logging.info(f"Message packet (ascii): {ascii_str}")
        
        try:
            if not self.serial_conn:
                logging.warning("No serial connection, attempting to reconnect...")
                self._connect()
                
            if self.serial_conn and self.serial_conn.is_open:
                bytes_written = self.serial_conn.write(message)
                ascii_str = ''.join(chr(byte) if 32 <= byte <= 126 else f'\\x{byte:02x}' for byte in message)
                logging.info(f"Message sent ({bytes_written} bytes) - hex: {message.hex()}, ascii: {ascii_str}")
                # sleep(1/2)
                return True
            else:
                logging.error("Serial connection not open")
                return False
                
        except serial.SerialException as e:
            logging.error(f"Serial communication error: {e}")
            # Try to reconnect for next time
            try:
                self.serial_conn.close()
            except:
                pass
            self._connect()
            return False
        except Exception as e:
            logging.error(f"Unexpected error in UART send: {e}")
            return False

    def close(self):
        """Close the serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logging.info("Serial connection closed")