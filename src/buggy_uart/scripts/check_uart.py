#!/usr/bin/env python3
"""Standalone UART sanity checker - no ROS/rclpy dependency, just pyserial,
so it can be run directly to check the physical link before bringing up
any ROS launch file at all:

    python3 check_uart.py /dev/ttyUSB0
    python3 check_uart.py /dev/ttyUSB0 --baud 115200

Two independent checks, matching the two directions this project's real
UART protocol actually uses (buggy_uart/uart_bridge.py for feedback IN,
buggy_uart/uart_interface.py for steering OUT) - same 6-byte framing
(START 0x28 ... END 0x29) both ways, just different payload layouts:

READ CHECK (default, always safe - read-only): opens the port, listens
for wheel-feedback packets, decodes them with the EXACT same parsing
logic as uart_bridge.py (16-bit signed big-endian steering, byte-3 speed),
and reports a live count/rate plus the most recent decoded value. If
nothing ever decodes, prints a hex dump of whatever raw bytes DID arrive
(if any) so you can tell "no data at all" apart from "data arriving but
framing/parsing looks wrong."

WRITE CHECK (--send-test, opt-in, requires typing "yes" to confirm): sends
exactly ONE benign packet - steering=0.0 degrees, red=True (STOP, the
firmware's safe/stopped state per uart_sender_node.py's own default),
yellow=False - using the exact same create_message() encoding as
uart_interface.py, then exits. This is NOT gated by the ROS-side loopback
parameter (there's no ROS node here at all) - it always physically writes
those 6 bytes to the port when you pass --send-test and confirm. Only use
it when you specifically want to verify the write path/wiring, understand
it commands a real (if minimal/safe) state to the vehicle, and have
confirmed the vehicle is safe to receive it (wheels can move, area clear).

TELEOP MODE (--teleop): teleop_twist_keyboard-style continuous keyboard
control, but driving the UART protocol directly instead of publishing
geometry_msgs/Twist - no ROS involved. Sends a packet at a fixed rate
(--rate, default 10Hz, matching uart_sender_node.py's own timer_callback
rate) using whatever steering/stop state is currently held, the same
"keep sending until changed" model teleop_twist_keyboard uses for
velocity commands. Also displays live /wheel_uart-equivalent feedback
(if any arrives) next to the commanded value, the same target-vs-actual
idea as steering_uart_bridge.py's RViz markers, just printed to the
terminal instead.

Starts in DRY-RUN (nothing physically written - prints what it would send
instead) unless you pass --live AND then type "yes" at the extra prompt
that appears - continuous real actuation is higher-stakes than the single
packet --send-test sends, so it gets its own confirmation on top of the
flag. If you decline that prompt it falls back to dry-run rather than
aborting, so you can still see the tool working before committing to
--live. Starts centered (0deg) and STOPPED (red=True) regardless of mode -
you must press 'g' to go. On exit (q or Ctrl-C) it always sends one final
centered+STOP command if running live, so it never leaves the vehicle
mid-turn or mid-drive.

SPEED TEST MODE (--speed-test): non-interactive, scripted alternative to
--teleop for a specific question - "hold speed at/under a cap for N
seconds and give me a report" - without needing a human at the keyboard.
Since this protocol has no speed-command field at all (only steering +
GO/STOP), the only way to govern speed is a bang-bang loop: send GO while
feedback speed is below --speed-cap, switch to STOP the instant feedback
reports >= cap, resume GO once it drops back below, repeated continuously
for --duration seconds (not stopping the whole test at the first
breach - that was an earlier, cruder version of this idea, tried and
superseded during development). Steering is held fixed at --steer-deg
(default 0.0, centered) for the whole run - this mode is about speed,
not steering.

Same DRY-RUN-by-default + --live confirmation-prompt safety pattern as
--teleop. Prints a RECV line on every new feedback value, and ends with a
full report: sample count, min/max speed actually observed, how many
samples (if any) exceeded the cap despite the governor, how many times
the governor actually had to toggle GO->STOP, and a speed-over-time trace.

Confirmed live (2026-08-06) that vehicle speed lags several seconds
behind a GO command before increasing at all (observed: 0 km/h for the
first ~3.5-4s of continuous GO, every run) - a short --duration can
finish before the governor ever needs to intervene, which the report's
toggle count will show honestly (0 toggles doesn't necessarily mean "the
governor works", it can also mean "speed never reached the cap during
this window" - read the speed-over-time trace, not just the toggle count).

Exit code 0 if the port opened successfully, non-zero otherwise (e.g. bad
device path, permission denied) - usable in a shell script/CI-style check.
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

try:
    import termios
    import tty
    import select
    _HAVE_TTY = True
except ImportError:
    # termios/tty are POSIX-only (no Windows support) - teleop mode simply
    # isn't available there; the read/write checks above don't need them.
    _HAVE_TTY = False

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


def build_test_send_packet(yellow=False, red=True, degree=0.0):
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


def run_read_check(ser, duration_sec):
    print(f"Listening for feedback packets on {ser.port} for {duration_sec:.0f}s "
          f"(Ctrl+C to stop early)...")
    buf = bytearray()
    total_bytes = 0
    decoded_count = 0
    last_decoded = None
    t_start = time.time()
    try:
        while time.time() - t_start < duration_sec:
            chunk = ser.read(128)
            if chunk:
                total_bytes += len(chunk)
                buf.extend(chunk)
            while len(buf) >= PACKET_SIZE:
                start_idx = find_packet_start(buf)
                if start_idx == -1:
                    buf.clear()
                    break
                if start_idx > 0:
                    del buf[:start_idx]
                    continue
                packet = bytes(buf[:PACKET_SIZE])
                result = decode_feedback_packet(packet)
                del buf[:PACKET_SIZE]
                if result:
                    decoded_count += 1
                    last_decoded = result
                    if decoded_count % 10 == 1:
                        steering_deg, speed = result
                        elapsed = time.time() - t_start
                        rate = decoded_count / elapsed if elapsed > 0 else 0.0
                        print(f"  [{elapsed:5.1f}s] decoded #{decoded_count} "
                              f"(~{rate:.1f} Hz): steering={steering_deg:+.2f} deg, "
                              f"speed={speed}")
    except KeyboardInterrupt:
        print("  (stopped early)")

    print()
    print("=== READ CHECK RESULT ===")
    print(f"Raw bytes received: {total_bytes}")
    print(f"Valid packets decoded: {decoded_count}")
    if decoded_count > 0:
        steering_deg, speed = last_decoded
        print(f"Most recent: steering={steering_deg:+.2f} deg, speed={speed}")
        print("PASS - feedback packets are arriving and decoding correctly.")
        return 0
    elif total_bytes > 0:
        print(f"Raw hex sample: {buf.hex() if buf else '(buffer consumed, none left)'}")
        print("FAIL - bytes are arriving but never form a valid "
              f"START(0x{START_BYTE:02x})...END(0x{END_BYTE:02x}) 6-byte packet - "
              "check baud rate/wiring/framing.")
        return 1
    else:
        print("FAIL - no bytes received at all - check port, baud rate, wiring, "
              "and that the MCU is actually powered/transmitting.")
        return 1


def run_write_check(ser, degree, red, yellow):
    message = build_test_send_packet(yellow=yellow, red=red, degree=degree)
    print(f"About to WRITE a real packet to {ser.port}:")
    print(f"  hex: {message.hex()}  (steering={degree:+.1f} deg, red={red}, yellow={yellow})")
    print("This physically commands the vehicle's steering/stop state over the wire.")
    confirm = input("Type 'yes' to actually send this packet: ").strip().lower()
    if confirm != 'yes':
        print("Not confirmed - nothing sent.")
        return 1
    written = ser.write(message)
    ser.flush()
    print(f"Wrote {written} bytes. (This script cannot confirm the MCU acted on it - "
          "observe the vehicle/its own feedback separately.)")
    return 0 if written == len(message) else 1


TELEOP_LEGEND = """
Teleop controls (like teleop_twist_keyboard, but for the raw UART link):
    a : steer left   (-step)
    d : steer right  (+step)
    s : center steering (0 deg)
    g : GO      (release stop)
    r : STOP    (also re-centers steering)
    y : toggle yellow/caution flag
    q : quit (sends one final centered+STOP command if --live)
"""


def _get_key(settings, timeout):
    """Single non-blocking keypress read, teleop_twist_keyboard's own
    getKey() pattern: raw tty mode + select() with a timeout so the loop
    can still pace itself (and send) even with no key pressed."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def run_teleop(ser, rate_hz, step_deg, live):
    if not _HAVE_TTY:
        print("FAIL - teleop mode needs a POSIX terminal (termios/tty/select) - "
              "not available on this platform.", file=sys.stderr)
        return 1

    if live:
        confirm = input(
            "--live given: this will CONTINUOUSLY write real steering/stop commands "
            "to the serial port while teleop runs. Type 'yes' to proceed live, "
            "anything else falls back to dry-run: ").strip().lower()
        if confirm != 'yes':
            print("Not confirmed - continuing in DRY-RUN (nothing will be written).")
            live = False

    settings = termios.tcgetattr(sys.stdin)
    steering_deg = 0.0
    red = True  # fail-safe default: starts STOPPED, 'g' required to go
    yellow = False
    period = 1.0 / rate_hz
    feedback_buf = bytearray()
    mode_tag = "LIVE" if live else "DRY "

    # NOTE on "speed": this protocol has no commanded-speed field at all -
    # create_message()/build_test_send_packet() only ever encode steering
    # + yellow/red flags (see uart_interface.py). "status=GO/STOP" (the red
    # flag) is the closest thing to a speed command this wire format has;
    # the actual km/h number only ever appears in FEEDBACK (byte 3 of the
    # packets the MCU sends back), which is what RECV lines below show.
    print(TELEOP_LEGEND)
    print(f"Mode: {'LIVE - commands ARE being written to ' + ser.port if live else 'DRY-RUN - nothing is written'}")
    print("SENT lines = what this tool commands (steering + GO/STOP - no speed command exists in this protocol).")
    print("RECV lines = actual feedback the MCU reports back (steering AND speed, from /wheel_uart-equivalent packets).")
    print()

    last_sent = None  # (steering_deg, red, yellow) - print a SENT line only when this changes
    last_feedback_printed = None  # throttle RECV line spam - print only on change

    def send_state():
        return (steering_deg, red, yellow)

    try:
        while True:
            key = _get_key(settings, period)
            if key in ('q', '\x03'):
                break
            elif key == 'd':
                steering_deg = min(DEGREE_RANGE[1], steering_deg + step_deg)
            elif key == 'a':
                steering_deg = max(DEGREE_RANGE[0], steering_deg - step_deg)
            elif key == 's':
                steering_deg = 0.0
            elif key == 'g':
                red = False
            elif key == 'r':
                red = True
                steering_deg = 0.0
            elif key == 'y':
                yellow = not yellow

            # Drain any feedback bytes that arrived, non-blocking (ser was
            # opened with a short timeout - see main()) - print one RECV
            # line per NEW decoded value (not per packet - the MCU can send
            # much faster than a human needs to read).
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
                if result and result != last_feedback_printed:
                    last_feedback_printed = result
                    actual_deg, actual_speed = result
                    delta = actual_deg - steering_deg
                    print(f"  RECV <- steer={actual_deg:+6.2f}deg  speed={actual_speed:3d} km/h  "
                          f"(vs commanded steer {steering_deg:+5.1f}deg, delta={delta:+.2f}deg)")

            message = build_test_send_packet(yellow=yellow, red=red, degree=steering_deg)
            if live:
                ser.write(message)
                ser.flush()

            current = send_state()
            if current != last_sent:
                last_sent = current
                status = "STOP" if red else " GO "
                print(f"SENT -> [{mode_tag}] steer={steering_deg:+5.1f}deg  status={status}  "
                      f"yellow={'Y' if yellow else 'n'}  hex={message.hex()}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        if live:
            safe_msg = build_test_send_packet(yellow=False, red=True, degree=0.0)
            ser.write(safe_msg)
            ser.flush()
            print("Sent final centered+STOP command on exit.")
        print("Teleop ended.")
    return 0


def run_speed_test(ser, cap_kmh, duration_sec, steer_deg, rate_hz, live):
    if live:
        confirm = input(
            f"--live given: this will CONTINUOUSLY write GO/STOP commands to govern speed "
            f"to <= {cap_kmh:.1f} km/h for {duration_sec:.0f}s (steering held at "
            f"{steer_deg:+.1f}deg). Type 'yes' to proceed live, anything else falls back to "
            "dry-run: ").strip().lower()
        if confirm != 'yes':
            print("Not confirmed - continuing in DRY-RUN (nothing will be written).")
            live = False

    period = 1.0 / rate_hz
    feedback_buf = bytearray()
    history = []  # (elapsed_sec, steer_deg_actual, speed_kmh)
    stop_toggle_count = 0
    mode_tag = "LIVE" if live else "DRY "
    state = {'last_printed': None, 'last_speed': 0}

    def send(degree, red, yellow=False):
        message = build_test_send_packet(yellow=yellow, red=red, degree=degree)
        if live:
            ser.write(message)
            ser.flush()
        return message

    def drain_feedback(t_elapsed):
        chunk = ser.read(256)
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
            if result:
                state['last_speed'] = result[1]
                history.append((t_elapsed, result[0], result[1]))
                if result != state['last_printed']:
                    state['last_printed'] = result
                    print(f"  [{t_elapsed:6.2f}s] RECV <- steer={result[0]:+6.2f}deg  "
                          f"speed={result[1]:3d} km/h")
        return state['last_speed']

    print(f"Mode: {'LIVE - commands ARE being written to ' + ser.port if live else 'DRY-RUN - nothing is written'}")
    print(f"Governing speed to <= {cap_kmh:.1f} km/h for {duration_sec:.0f}s, "
          f"steering held at {steer_deg:+.1f}deg.")
    print("Bang-bang governor: GO while feedback speed < cap, STOP the instant feedback >= cap, "
          "resume GO once it drops back below - repeats for the full duration (see this script's "
          "SPEED TEST MODE docstring for why a run with 0 toggles isn't necessarily proof it works).")
    print()

    send(0.0, red=True)  # baseline STOP before starting
    drain_feedback(0.0)
    time.sleep(0.3)

    t_start = time.time()
    next_send = t_start
    red_state = True
    try:
        while True:
            t_elapsed = time.time() - t_start
            if t_elapsed >= duration_sec:
                break
            spd = drain_feedback(t_elapsed)
            desired_red = spd >= cap_kmh
            if desired_red != red_state:
                if desired_red:
                    stop_toggle_count += 1
                red_state = desired_red
            if time.time() >= next_send:
                send(steer_deg, red=red_state)
                next_send += period
            time.sleep(0.005)
    finally:
        send(0.0, red=True)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            drain_feedback(duration_sec + (time.time() - t0))
            time.sleep(0.02)

    print()
    print(f"=== SPEED TEST REPORT (cap={cap_kmh:.1f}km/h, duration={duration_sec:.0f}s, "
          f"steer={steer_deg:+.1f}deg, mode={mode_tag}) ===")
    print(f"Total feedback samples: {len(history)}")
    exit_code = 0
    if history:
        speeds = [h[2] for h in history]
        print(f"Speed range observed: min={min(speeds)} km/h, max={max(speeds)} km/h")
        over_cap = [s for s in speeds if s > cap_kmh]
        print(f"Samples exceeding cap: {len(over_cap)} of {len(speeds)} "
              f"({100.0 * len(over_cap) / len(speeds):.1f}%)")
        if over_cap:
            print(f"  overshoot values seen: {sorted(set(over_cap))}")
            exit_code = 1
        print(f"Governor GO->STOP toggles triggered: {stop_toggle_count}")
        print("Speed-over-time trace (value changes only):")
        last_s = None
        for t, deg, spd in history:
            if spd != last_s:
                print(f"  t={t:6.2f}s  steer={deg:+6.2f}deg  speed={spd} km/h")
                last_s = spd
    else:
        print("No feedback received at all during the test - check the read check "
              "(run without --speed-test) before trusting this mode.")
        exit_code = 1
    print("Done - left in STOP state.")
    return exit_code


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', help='Serial device, e.g. /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--duration', type=float, default=5.0,
                         help='Seconds to listen for feedback in the read check, or total '
                              'governed run time for --speed-test (default 5)')
    parser.add_argument('--send-test', action='store_true',
                         help='Also run the write check (sends ONE real packet, requires '
                              'typed confirmation) - see this script\'s own docstring for '
                              'exactly what it sends and why it defaults off')
    parser.add_argument('--skip-read', action='store_true',
                         help='Skip the read check (useful if you only want --send-test)')
    parser.add_argument('--teleop', action='store_true',
                         help='teleop_twist_keyboard-style continuous keyboard control '
                              'instead of the read/send-test checks - see this script\'s '
                              'own docstring\'s TELEOP MODE section')
    parser.add_argument('--speed-test', action='store_true',
                         help='Non-interactive governed speed test (bang-bang GO/STOP to hold '
                              'speed at/under --speed-cap for --duration seconds, then a full '
                              'report) - see this script\'s own docstring\'s SPEED TEST MODE section')
    parser.add_argument('--speed-cap', type=float, default=2.0,
                         help='--speed-test only: km/h cap to govern to (default 2.0)')
    parser.add_argument('--steer-deg', type=float, default=0.0,
                         help='--speed-test only: steering angle (deg) to hold for the whole '
                              'test (default 0.0, centered)')
    parser.add_argument('--live', action='store_true',
                         help='--teleop/--speed-test only: actually write commands to the '
                              'serial port (with an extra typed confirmation prompt) instead '
                              'of dry-run')
    parser.add_argument('--rate', type=float, default=10.0,
                         help='--teleop/--speed-test only: send rate in Hz (default 10, '
                              'matching uart_sender_node.py\'s own timer_callback rate)')
    parser.add_argument('--step', type=float, default=2.0,
                         help='--teleop only: degrees added/removed per a/d keypress (default 2.0)')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"FAIL - could not open {args.port} @ {args.baud} baud: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Opened {args.port} @ {args.baud} baud OK.")
    exit_code = 0

    if args.teleop:
        exit_code = run_teleop(ser, rate_hz=args.rate, step_deg=args.step, live=args.live)
        ser.close()
        sys.exit(exit_code)

    if args.speed_test:
        exit_code = run_speed_test(
            ser, cap_kmh=args.speed_cap, duration_sec=args.duration,
            steer_deg=args.steer_deg, rate_hz=args.rate, live=args.live)
        ser.close()
        sys.exit(exit_code)

    if not args.skip_read:
        exit_code = run_read_check(ser, args.duration)
        print()

    if args.send_test:
        write_code = run_write_check(ser, degree=0.0, red=True, yellow=False)
        exit_code = exit_code or write_code

    ser.close()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
