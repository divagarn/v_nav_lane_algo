#!/usr/bin/env python3
"""Standalone speed-preset UART test - no ROS/rclpy dependency, just
pyserial (same style as check_uart.py in this same directory).

Prompts for which of the real buggy's two firmware speed presets to run
(2 or 4 km/h - selects the yellow byte: yellow=1 -> 2 km/h, yellow=0 ->
4 km/h, matching steering_uart_bridge.py's slow_speed_preset logic and
uart_interface.py's create_message() encoding), then sends GO
continuously at that preset while you turn the steering by hand -
steering is NOT commanded by this script (held at 0 deg, neutral) since
the point here is to read back the ACTUAL steering angle from the
buggy's own feedback packets (/wheel_uart-equivalent, same 6-byte
framing decoded by uart_bridge.py) as you turn it, alongside the actual
feedback speed - not to drive steering from this tool.

Usage:
    python3 speed_preset_test.py /dev/ttyUSB0
    python3 speed_preset_test.py /dev/ttyUSB0 --baud 115200 --live

Starts in DRY-RUN (nothing physically written, just shows what would be
sent) unless you pass --live AND then type "yes" at the extra confirm
prompt - same safety pattern check_uart.py uses, since this continuously
commands GO to a real vehicle. Regardless of mode, feedback reading is
always active and always safe (read-only).

On exit (Ctrl+C) it always sends one final STOP (red=1) command if
running live, so it never leaves the vehicle commanded to GO.
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print(
        "pyserial not installed - run: pip3 install pyserial "
        "(or: sudo apt install python3-serial)",
        file=sys.stderr)
    sys.exit(1)

START_BYTE = 0x28  # '(' - shared by both directions of this protocol
END_BYTE = 0x29    # ')'
PACKET_SIZE = 6
OFFSET = -30
RESOLUTION = 0.01
DEGREE_RANGE = (-20, 20)


def find_packet_start(buf):
    for i, b in enumerate(buf):
        if b == START_BYTE:
            return i
    return -1


def decode_feedback_packet(data):
    """Same decode logic as uart_bridge.py's parse_packet()."""
    if len(data) != PACKET_SIZE or data[0] != START_BYTE or data[5] != END_BYTE:
        return None
    steering_raw = int.from_bytes(data[1:3], byteorder='big', signed=True)
    steering_deg = steering_raw * RESOLUTION + OFFSET
    steering_deg = max(min(steering_deg, DEGREE_RANGE[1]), DEGREE_RANGE[0])
    speed = data[3]
    return steering_deg, speed


def build_send_packet(yellow, red, degree=0.0):
    """Same encoding as uart_interface.py's create_message()."""
    degree = max(min(degree, DEGREE_RANGE[1]), DEGREE_RANGE[0])
    raw_value = int((degree - OFFSET) / RESOLUTION)
    raw_value = max(min(raw_value, 32767), -32768)
    raw_bytes = raw_value.to_bytes(2, byteorder='big', signed=True)
    message = bytearray()
    message.append(START_BYTE)
    message.append(1 if yellow else 0)
    message.append(1 if red else 0)
    message.extend(raw_bytes)
    message.append(END_BYTE)
    return bytes(message)


def ask_speed_preset():
    while True:
        raw = input("Enter target speed preset - 2 or 4 (km/h): ").strip()
        if raw in ('2', '2.0'):
            return 2.0
        if raw in ('4', '4.0'):
            return 4.0
        print("  Only 2 or 4 are valid real firmware presets - try again.")


def run(ser, speed_kmph, live):
    yellow = speed_kmph <= 3.0
    print(f"Selected preset: {speed_kmph:.0f} km/h -> yellow={'1' if yellow else '0'}, red=0 (GO)")

    if live:
        confirm = input(
            "--live given: this will CONTINUOUSLY command GO at this speed preset "
            "to the real vehicle over the serial port. Type 'yes' to proceed live, "
            "anything else falls back to dry-run: ").strip().lower()
        if confirm != 'yes':
            print("Not confirmed - continuing in DRY-RUN (nothing will be written).")
            live = False

    print(f"Mode: {'LIVE - commands ARE being written to ' + ser.port if live else 'DRY-RUN - nothing is written'}")
    print("Steering is NOT commanded by this script (held at 0 deg / neutral) - "
          "turn the steering by hand now and watch the RECV lines below for the "
          "actual feedback angle read back from the buggy.")
    print("Ctrl+C to stop (sends a final STOP command first if --live).\n")

    feedback_buf = bytearray()
    last_printed = None
    rate_hz = 10.0
    period = 1.0 / rate_hz
    next_send = time.time()

    try:
        while True:
            chunk = ser.read(64)
            if chunk:
                feedback_buf.extend(chunk)
            while len(feedback_buf) >= PACKET_SIZE:
                idx = find_packet_start(feedback_buf)
                if idx == -1:
                    feedback_buf.clear()
                    break
                if idx > 0:
                    del feedback_buf[:idx]
                    continue
                result = decode_feedback_packet(bytes(feedback_buf[:PACKET_SIZE]))
                del feedback_buf[:PACKET_SIZE]
                if result and result != last_printed:
                    last_printed = result
                    steer_deg, speed = result
                    print(f"  RECV <- steering={steer_deg:+6.2f} deg   speed={speed:3d} km/h")

            now = time.time()
            if now >= next_send:
                message = build_send_packet(yellow=yellow, red=False, degree=0.0)
                if live:
                    ser.write(message)
                    ser.flush()
                next_send += period
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if live:
            stop_msg = build_send_packet(yellow=False, red=True, degree=0.0)
            ser.write(stop_msg)
            ser.flush()
            print("Sent final STOP command (red=1).")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', help='Serial device, e.g. /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--live', action='store_true',
                         help='Actually write GO commands to the serial port (with an extra '
                              'typed confirmation prompt) instead of dry-run.')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"FAIL - could not open {args.port} @ {args.baud} baud: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Opened {args.port} @ {args.baud} baud OK.")
    speed_kmph = ask_speed_preset()
    run(ser, speed_kmph, args.live)
    ser.close()


if __name__ == '__main__':
    main()
