H753 STM32 Xbox UART drive test

Files in this bundle:
- xbox_uart_can_telemetry.py
- xbox_uart_teleop.py
- xbox_drive_config.py
- xbox_drive_core.py
- drive_test_requirements.txt
- run_xbox_uart_drive.sh

Purpose:
- Xbox controller drives STM32 through UART.
- CAN is disabled in this test runner with --no-can.
- Left stick Y controls forward/backward throttle.
- Right stick X controls rotate-in-place when stopped.
- Left stick Y and right stick X can be used at the same time.
- Stick input is converted to body velocity v,w first.
- v,w is sent directly to STM with the UART CMD_TWIST packet.
- STM runs differential/skid-steer inverse kinematics and PID speed control.
- Left stick X is ignored so forward/backward driving is stable.

Default tracked-wheel geometry:
- Track ground contact length: 0.26 m
- Track belt width: 0.075 m
- Effective left/right track-center distance: 0.45 m

The contact length and belt width are logged for calibration reference.
The STM inverse kinematics directly uses the effective track-center distance.
Every run overwrites a clean combined TX/STM session log at:

  tools/logs/xbox_drive_latest.log

This log is flushed during driving so it can be inspected while the test is running.
Only one UART drive process may run at a time. A second invocation exits instead
of sending conflicting motor commands. Ctrl+C, SIGTERM, or terminal hangup sends
a stop frame and closes the session cleanly.
The program intentionally lets process exit release pygame resources because
SDL joystick shutdown/finalization can block after the Xbox device disconnects.
The UART stop frame, port close, and log flush happen before this forced clean exit.

Python file layout:
- xbox_drive_config.py: drive parameters and UART packet constants
- xbox_drive_core.py: joystick, kinematics, and UART frame helper functions
- xbox_uart_can_telemetry.py: executable control/telemetry script

Setup on the test PC:

  cd ~/h753_drive_test
  chmod +x run_xbox_uart_drive.sh

Run with the STM UART port:

  ./run_xbox_uart_drive.sh

The runner auto-selects the STMicroelectronics STLINK /dev/serial/by-id port
when it is present. You can still force a port:

  ./run_xbox_uart_drive.sh /dev/ttyACM1

Override speed/geometry defaults:

  MAX_LINEAR=0.40 \
  MAX_ANGULAR=2.67 \
  TRACK_GAUGE=0.45 \
  MAX_TRACK_SPEED=0.40 \
  TRACK_CONTACT_LENGTH=0.26 \
  TRACK_BELT_WIDTH=0.075 \
  ./run_xbox_uart_drive.sh /dev/ttyACM1

If the STM appears on another port, check:

  ls -l /dev/serial/by-id/
  ls -l /dev/ttyACM* /dev/ttyUSB*

Manual run:

  python3 -m venv .venv
  source .venv/bin/activate
  python3 -m pip install -r drive_test_requirements.txt
  python3 xbox_uart_can_telemetry.py \
    --port /dev/ttyACM1 \
    --no-can \
    --debug \
    --max-linear 0.60 \
    --max-angular 2.67 \
    --track-gauge 0.45 \
    --max-track-speed 0.60 \
    --track-contact-length 0.26 \
    --track-belt-width 0.075

If joystick values change in debug but the robot does not move:

  1. Check the connected UART line says STMicroelectronics/STLINK, not CANable.
  2. Flash the latest STM firmware because the default protocol is CMD_TWIST.
  3. Test old firmware compatibility:

     PROTOCOL=legacy ./run_xbox_uart_drive.sh

  4. Watch STM serial text. If target(L,R) changes but pwm(L,R) stays 0 and fault is nonzero,
     the STM encoder fault guard is blocking motor output.

Notes:
- Do not use CAN options for this drive-only test.
- If the user is not in the dialout group, serial open may fail with permission denied.
  Fix on Ubuntu:

    sudo usermod -aG dialout $USER

  Then log out and log back in.
