#!/usr/bin/env python3
"""Real-vehicle parameter calibration tool - standalone, no ROS/rclpy
dependency, just pyserial (same style as check_uart.py and
speed_preset_test.py in this same directory).

Measures real numbers to replace the guessed/nominal values currently
hardcoded in this workspace's config (wheelbase=1.6, max_steering_deg=20.0,
target_speed_kmph=2/4, max_steering_rate_deg_s=30.0 - see
speed_governor.py / steering_uart_bridge.py) by reading the buggy's own
UART feedback packets (same 6-byte framing uart_bridge.py decodes:
steering degrees + speed km/h, both already calibrated by the vehicle's
own MCU before the packet is sent).

Three independent tests:

  --sweep   Steering-only (never commands GO/red=0). Commands a sequence
            of steering angles, holds each, reads back the SETTLED
            feedback angle. Produces a commanded-vs-actual table and the
            true physical saturation point (may not be exactly +-20deg).

  --slew    Steering-only (never commands GO). Commands one abrupt step
            change and times how long the feedback angle takes to reach
            it. Gives a real deg/s number for max_steering_rate_deg_s
            (currently just a guess in the code).

  --speed   COMMANDS GO (drives the vehicle). For each speed preset
            (yellow=1/"2km/h" and yellow=0/"4km/h"), sends GO at 0deg
            (centered) for --speed-duration seconds, skips the first
            --speed-ramp-skip seconds (confirmed elsewhere: real speed
            lags ~3.5-4s behind a GO command before rising at all), then
            reports the actual steady-state feedback speed. Needs a
            clear, straight, safe path - the vehicle actually drives
            forward.

NOT measurable this way: wheelbase / true turning radius. This
workspace's /odom on the real vehicle (wheel_odometry.py) is dead-reckoned
FROM this same feedback using the wheelbase value you'd be trying to
verify - using it here would be circular. Wheelbase needs an independent
ground truth instead (GPS trace, or physically pacing out the turn radius
at a fixed steering angle) - not implemented here.

Safety: dry-run by default for ALL tests (nothing physically written,
just prints what would be sent) - pass --live to actually write. --sweep
and --slew only ever move the steering actuator (red=1/STOP the whole
time, drive motor never commanded) and need one typed confirmation under
--live. --speed actually drives the vehicle and needs a second, separate
typed confirmation on top of --live, same pattern as speed_preset_test.py.

Usage:
    python3 real_param_calibration.py /dev/ttyUSB0 --sweep --slew
    python3 real_param_calibration.py /dev/ttyUSB0 --sweep --slew --live
    python3 real_param_calibration.py /dev/ttyUSB0 --speed --live
    python3 real_param_calibration.py /dev/ttyUSB0 --sweep --slew --speed --live \\
        --report-out calibration_result.json

On exit (Ctrl+C or normal completion) always sends one final centered+STOP
command if --live was used, same as the other scripts in this directory.
"""
import argparse
import json
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


class FeedbackReader:
    """Non-blocking incremental feedback decoder - one instance shared
    across a whole run so no bytes get dropped between test phases."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()
        self.last = None  # (steering_deg, speed)
        self.last_time = None

    def poll(self):
        chunk = self.ser.read(128)
        if chunk:
            self.buf.extend(chunk)
        results = []
        while len(self.buf) >= PACKET_SIZE:
            idx = find_packet_start(self.buf)
            if idx == -1:
                self.buf.clear()
                break
            if idx > 0:
                del self.buf[:idx]
                continue
            result = decode_feedback_packet(bytes(self.buf[:PACKET_SIZE]))
            del self.buf[:PACKET_SIZE]
            if result:
                self.last = result
                self.last_time = time.time()
                results.append((self.last_time, result))
        return results


def confirm(prompt):
    return input(prompt).strip().lower() == 'yes'


def run_sweep(ser, reader, live, sweep_min, sweep_max, sweep_step, hold_sec, sample_frac,
              go=False, go_speed_kmph=2.0):
    print("\n=== STEERING SWEEP ===")
    red_state = not go
    yellow_state = go_speed_kmph <= 3.0
    print(f"Commanding {sweep_min:+.0f}deg to {sweep_max:+.0f}deg in {sweep_step:.0f}deg steps, "
          f"holding {hold_sec:.1f}s each "
          f"({'red=0/GO at ' + str(go_speed_kmph) + 'kmph - VEHICLE WILL DRIVE FORWARD while turning' if go else 'red=1/STOP throughout - drive motor never commanded'}).")
    if go:
        if live and not confirm(
                "--go given: this test WILL DRIVE THE VEHICLE FORWARD while steering sweeps - "
                "confirm a clear, open (not just straight) area, someone ready to stop it, and "
                "type 'yes' to proceed live: "):
            print("Not confirmed - skipping sweep.")
            return []
    elif live and not confirm("This will physically move the steering actuator. Type 'yes' to proceed: "):
        print("Not confirmed - skipping sweep.")
        return []

    angles = []
    a = sweep_min
    while a <= sweep_max + 1e-6:
        angles.append(a)
        a += sweep_step

    table = []
    for cmd_deg in angles:
        message = build_send_packet(yellow=yellow_state, red=red_state, degree=cmd_deg)
        t_start = time.time()
        samples = []
        while time.time() - t_start < hold_sec:
            if live:
                ser.write(message)
                ser.flush()
            for t, result in reader.poll():
                if t - t_start >= hold_sec * sample_frac:
                    samples.append(result[0])
            time.sleep(0.02)
        if samples:
            actual = sum(samples) / len(samples)
            print(f"  commanded {cmd_deg:+6.1f}deg -> actual {actual:+6.2f}deg "
                  f"({len(samples)} settled samples)")
            table.append((cmd_deg, actual))
        else:
            tag = "(dry-run, no feedback expected)" if not live else "(no feedback received)"
            print(f"  commanded {cmd_deg:+6.1f}deg -> no data {tag}")
            table.append((cmd_deg, None))

    if live:
        ser.write(build_send_packet(yellow=False, red=True, degree=0.0))
        ser.flush()
    return table


def run_slew(ser, reader, live, step_from, step_to, settle_tol, settle_window, timeout,
             go=False, go_speed_kmph=2.0):
    print("\n=== STEERING SLEW-RATE ===")
    red_state = not go
    yellow_state = go_speed_kmph <= 3.0
    print(f"Settling at {step_from:+.0f}deg, then stepping abruptly to {step_to:+.0f}deg and "
          f"timing the feedback ramp "
          f"({'red=0/GO at ' + str(go_speed_kmph) + 'kmph - VEHICLE WILL DRIVE FORWARD while turning' if go else 'red=1/STOP throughout'}).")
    if go:
        if live and not confirm(
                "--go given: this test WILL DRIVE THE VEHICLE FORWARD while steering steps - "
                "confirm a clear, open (not just straight) area, someone ready to stop it, and "
                "type 'yes' to proceed live: "):
            print("Not confirmed - skipping slew-rate test.")
            return None
    elif live and not confirm("This will physically move the steering actuator. Type 'yes' to proceed: "):
        print("Not confirmed - skipping slew-rate test.")
        return None

    settle_msg = build_send_packet(yellow=yellow_state, red=red_state, degree=step_from)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        if live:
            ser.write(settle_msg)
            ser.flush()
        reader.poll()
        time.sleep(0.05)
    print(f"  settled at {step_from:+.0f}deg, stepping now...")

    step_msg = build_send_packet(yellow=yellow_state, red=red_state, degree=step_to)
    trace = []
    t_step = time.time()
    last_change_time = t_step
    prev_val = step_from
    while time.time() - t_step < timeout:
        if live:
            ser.write(step_msg)
            ser.flush()
        for t, result in reader.poll():
            val = result[0]
            trace.append((t - t_step, val))
            if abs(val - prev_val) > 0.05:
                last_change_time = t
            prev_val = val
            if time.time() - last_change_time > settle_window and abs(val - step_to) < settle_tol:
                break
        else:
            time.sleep(0.02)
            continue
        break

    if live:
        ser.write(build_send_packet(yellow=False, red=True, degree=0.0))
        ser.flush()

    if not trace:
        print("  no feedback received - cannot compute slew rate" if live else
              "  (dry-run, no feedback expected)")
        return None

    print("  trace (t, actual_deg):")
    for t, val in trace:
        print(f"    t={t:5.2f}s  actual={val:+6.2f}deg")
    # Use the time of the LAST actual movement (last_change_time), not the
    # last received sample overall - the loop keeps polling for
    # settle_window seconds after arrival to confirm it's actually
    # settled, and including that dwell time in the duration would
    # understate the true rate.
    total_deg = abs(trace[-1][1] - step_from)
    total_time = last_change_time - t_step
    if total_time <= 0:
        print("  WARN: settled instantly or timing too coarse - result unreliable.")
        return None
    rate = total_deg / total_time
    print(f"  moved {total_deg:.1f}deg in {total_time:.2f}s (last movement, excluding "
          f"settle-confirmation dwell) -> ~{rate:.1f} deg/s")
    return rate


def run_speed(ser, reader, live, duration, ramp_skip, presets):
    print("\n=== SPEED PRESET CALIBRATION ===")
    print("This COMMANDS GO and actually drives the vehicle forward (0deg/centered).")
    if live and not confirm(
            "--live given: this test will drive the vehicle. Confirm a clear, straight, "
            "safe path is in front of it. Type 'yes' to proceed live: "):
        print("Not confirmed - continuing in DRY-RUN for this test.")
        live_speed = False
    else:
        live_speed = live

    results = {}
    for label, yellow in presets:
        print(f"\n  -- preset: {label} (yellow={'1' if yellow else '0'}) --")
        go_msg = build_send_packet(yellow=yellow, red=False, degree=0.0)
        t0 = time.time()
        samples = []
        while time.time() - t0 < duration:
            if live_speed:
                ser.write(go_msg)
                ser.flush()
            for t, result in reader.poll():
                elapsed = t - t0
                if elapsed >= ramp_skip:
                    samples.append(result[1])
                    print(f"    [{elapsed:5.1f}s] feedback speed={result[1]} km/h")
            time.sleep(0.02)
        stop_msg = build_send_packet(yellow=False, red=True, degree=0.0)
        if live_speed:
            ser.write(stop_msg)
            ser.flush()
        print("    STOP sent, settling 1.5s...")
        t0 = time.time()
        while time.time() - t0 < 1.5:
            reader.poll()
            time.sleep(0.02)

        if samples:
            mean_s = sum(samples) / len(samples)
            print(f"  {label}: steady-state speed samples={len(samples)}, "
                  f"min={min(samples)} max={max(samples)} mean={mean_s:.2f} km/h")
            results[label] = {'min': min(samples), 'max': max(samples), 'mean': mean_s}
        else:
            print(f"  {label}: no steady-state feedback samples "
                  f"{'(dry-run)' if not live_speed else '(no feedback received)'}")
            results[label] = None
    return results


def print_summary(sweep_table, slew_rate, speed_results):
    print("\n=== CALIBRATION SUMMARY (recommended values to feed into config) ===")
    if sweep_table:
        valid = [(c, a) for c, a in sweep_table if a is not None]
        if valid:
            max_actual = max(abs(a) for _, a in valid)
            print(f"max_steering_deg: observed actual extremes up to {max_actual:.1f}deg "
                  "(compare against the current 20.0deg default)")
            print("  commanded vs actual table:")
            for c, a in valid:
                diff = a - c
                print(f"    {c:+6.1f}deg cmd -> {a:+6.2f}deg actual (diff {diff:+5.2f}deg)")
        else:
            print("max_steering_deg: no data (sweep skipped or dry-run)")
    if slew_rate is not None:
        print(f"max_steering_rate_deg_s: ~{slew_rate:.1f} deg/s measured "
              "(compare against the current 30.0deg/s default - "
              "recommend setting slightly BELOW this measured max, not equal to it)")
    if speed_results:
        for label, r in speed_results.items():
            if r:
                print(f"target_speed_kmph ({label}): measured mean={r['mean']:.2f} km/h "
                      f"(range {r['min']}-{r['max']})")
    print("\nwheelbase: NOT measurable by this tool (would be circular via /odom - see "
          "this script's own docstring). Needs an independent ground truth "
          "(GPS trace or paced-out turn radius) if you want it calibrated too.")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', help='Serial device, e.g. /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--live', action='store_true',
                         help='Actually write commands to the serial port (each test still '
                              'has its own typed confirmation). Without this, every test runs '
                              'as a dry-run (nothing written, no feedback expected).')

    parser.add_argument('--go', action='store_true',
                         help='--sweep/--slew only: send red=0 (GO) instead of red=1 (STOP) - '
                              'tests whether the steering actuator has more authority while '
                              'the vehicle is actually driving (real-hardware finding: it '
                              'undershoots and slows down toward the extremes under STOP). '
                              'VEHICLE WILL DRIVE FORWARD the whole time - needs open, not just '
                              'straight, clear space (steering keeps changing while it drives), '
                              'and gets its own extra confirmation prompt.')
    parser.add_argument('--go-speed-kmph', type=float, choices=[2.0, 4.0], default=2.0,
                         help='--go only: which speed preset to drive at during the test '
                              '(default 2.0 - the slower one, deliberately)')

    parser.add_argument('--sweep', action='store_true', help='Run the steering sweep test.')
    parser.add_argument('--sweep-min', type=float, default=-20.0)
    parser.add_argument('--sweep-max', type=float, default=20.0)
    parser.add_argument('--sweep-step', type=float, default=5.0)
    parser.add_argument('--hold-sec', type=float, default=3.0,
                         help='Seconds to hold each sweep angle before moving on (default 3.0)')
    parser.add_argument('--sample-frac', type=float, default=0.5,
                         help='Fraction of --hold-sec to wait before starting to collect '
                              'settled samples for that step (default 0.5 - skip first half)')

    parser.add_argument('--slew', action='store_true', help='Run the steering slew-rate test.')
    parser.add_argument('--slew-from', type=float, default=-15.0)
    parser.add_argument('--slew-to', type=float, default=15.0)
    parser.add_argument('--slew-settle-tol', type=float, default=1.0,
                         help='deg - how close to --slew-to counts as "arrived" (default 1.0)')
    parser.add_argument('--slew-settle-window', type=float, default=0.3,
                         help='seconds of no further movement required to call it settled')
    parser.add_argument('--slew-timeout', type=float, default=6.0)

    parser.add_argument('--speed', action='store_true',
                         help='Run the speed preset test (COMMANDS GO - drives the vehicle).')
    parser.add_argument('--speed-kmph', type=float, choices=[2.0, 4.0], default=None,
                         help='--speed only: test just ONE preset (2 or 4) instead of both '
                              'back-to-back - halves the required clear travel space. Default '
                              '(omitted): run both presets in sequence.')
    parser.add_argument('--speed-duration', type=float, default=10.0)
    parser.add_argument('--speed-ramp-skip', type=float, default=4.0,
                         help='Seconds to skip at the start of each preset before treating '
                              'feedback as steady-state (default 4.0 - real speed lags a GO '
                              'command by ~3.5-4s before rising at all, confirmed elsewhere)')

    parser.add_argument('--report-out', type=str, default=None,
                         help='Optional path to write the summary as JSON.')
    args = parser.parse_args()

    if not (args.sweep or args.slew or args.speed):
        print("Nothing to do - pass at least one of --sweep, --slew, --speed.", file=sys.stderr)
        sys.exit(1)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"FAIL - could not open {args.port} @ {args.baud} baud: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Opened {args.port} @ {args.baud} baud OK.")
    print(f"Mode: {'LIVE (each test asks for its own confirmation)' if args.live else 'DRY-RUN (nothing will be written)'}")
    reader = FeedbackReader(ser)

    sweep_table, slew_rate, speed_results = [], None, {}
    try:
        if args.sweep:
            sweep_table = run_sweep(
                ser, reader, args.live, args.sweep_min, args.sweep_max, args.sweep_step,
                args.hold_sec, args.sample_frac, go=args.go, go_speed_kmph=args.go_speed_kmph)
        if args.slew:
            slew_rate = run_slew(
                ser, reader, args.live, args.slew_from, args.slew_to,
                args.slew_settle_tol, args.slew_settle_window, args.slew_timeout,
                go=args.go, go_speed_kmph=args.go_speed_kmph)
        if args.speed:
            all_presets = [('2kmph_preset', True), ('4kmph_preset', False)]
            if args.speed_kmph is not None:
                selected = [p for p in all_presets if p[0].startswith(f'{args.speed_kmph:.0f}kmph')]
            else:
                selected = all_presets
            speed_results = run_speed(
                ser, reader, args.live, args.speed_duration, args.speed_ramp_skip,
                presets=selected)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if args.live:
            ser.write(build_send_packet(yellow=False, red=True, degree=0.0))
            ser.flush()
            print("\nSent final centered+STOP command.")
        ser.close()

    print_summary(sweep_table, slew_rate, speed_results)

    if args.report_out:
        report = {
            'sweep_table': sweep_table,
            'max_steering_rate_deg_s_measured': slew_rate,
            'speed_results': speed_results,
        }
        with open(args.report_out, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"\nReport written to {args.report_out}")


if __name__ == '__main__':
    main()
