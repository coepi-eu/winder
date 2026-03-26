#!/usr/bin/env python3
"""
Winder WebSocket Bridge

Bridges the winder serial hardware to the Flutter workstation app via WebSocket.

The winder code communicates over serial (USB) to motor controllers.
The Flutter app expects a WebSocket server on ws://winder.local:8765.
This bridge translates between the two.

Usage:
    python3 ws_bridge.py                          # real hardware
    python3 ws_bridge.py --simulation             # simulation mode (no hardware)
    python3 ws_bridge.py --config settings.yml    # custom config
    python3 ws_bridge.py --port 8765              # custom WS port

Requirements:
    pip install websockets pydantic pyyaml pyserial
"""

import asyncio
import json
import argparse
import threading
import traceback
from datetime import datetime

try:
    import websockets
except ImportError:
    print("Install websockets: pip install websockets")
    exit(1)

from src.winding import Wind

# Global state
winder: Wind = None
winder_lock = threading.Lock()
clients = set()
current_status = "idle"
current_phase = None
current_error = None
current_slot = 0
winding_thread = None
stop_requested = False


def get_motor_positions():
    """Read current motor positions."""
    with winder_lock:
        try:
            return {
                "M0": round(winder.get_motor_position(0), 3),
                "M1": round(winder.get_motor_position(1), 3),
                "M2": round(winder.get_motor_position(2), 3),
                "M3": round(winder.motor_positions[3], 3),
            }
        except Exception:
            return {
                "M0": winder.motor_positions[0],
                "M1": winder.motor_positions[1],
                "M2": winder.motor_positions[2],
                "M3": winder.motor_positions[3],
            }


def get_settings_dict():
    """Get current winder settings as dict for the Flutter app."""
    return {
        "turns_per_slot": winder.turns_per_slot,
        "winding_config": winder.config["winding"]["winding_config"],
        "starts_at": winder.starts_at,
        "wind_torque": winder.m3_wind_torque,
        "pull_wire_torque": winder.m3_pull_wire_torque,
        "m0_velocity": winder.motor_velocities[0],
        "m1_velocity": winder.motor_velocities[1],
        "m2_velocity": winder.motor_velocities[2],
        "m3_velocity": winder.motor_velocities[3],
        "wind_range_start": winder.m0_wind_range[0],
        "wind_range_end": winder.m0_wind_range[1],
        "m1_zero": winder.m1_zero,
        "m2_zero": winder.m2_zero,
        "angle_to_prevent_collision": winder.m2_angle_to_prevent_collision,
    }


def build_status_message():
    """Build the status JSON that the Flutter app expects."""
    return json.dumps({
        "type": "status",
        "motors": get_motor_positions(),
        "slot": current_slot,
        "winding": {
            "status": current_status,
            "phase": current_phase,
            "error": current_error,
        },
        "settings": get_settings_dict(),
    })


async def broadcast(message):
    """Send a message to all connected clients."""
    if clients:
        await asyncio.gather(
            *[client.send(message) for client in clients],
            return_exceptions=True,
        )


def run_continuous_winding():
    """Run continuous winding in a background thread."""
    global current_status, current_phase, current_slot, current_error, stop_requested

    try:
        current_status = "winding"
        current_phase = "continuous"
        current_error = None

        with winder_lock:
            winder.init_position(True)

        # Wind phase A
        current_phase = 0
        with winder_lock:
            winder.wind(0)
            winder.wind_wire_around_shaft(0)

        if stop_requested:
            current_status = "stopped"
            return

        # Wind phase B
        current_phase = 1
        with winder_lock:
            winder.starts_at = 0
            winder.wind(1)
            winder.wind_wire_around_shaft(1)

        if stop_requested:
            current_status = "stopped"
            return

        # Wind phase C
        current_phase = 2
        with winder_lock:
            winder.wind(2)

        current_status = "complete"
        current_phase = None

    except Exception as e:
        current_status = "error"
        current_error = str(e)
        traceback.print_exc()


def run_phase_winding(phase: int):
    """Run single phase winding in a background thread."""
    global current_status, current_phase, current_slot, current_error

    try:
        current_status = "winding"
        current_phase = phase
        current_error = None

        with winder_lock:
            winder.init_position(True)
            winder.wind(phase)

        current_status = "complete"
        current_phase = None

    except Exception as e:
        current_status = "error"
        current_error = str(e)
        traceback.print_exc()


async def handle_command(data):
    """Handle a command from the Flutter app."""
    global current_status, current_phase, current_error, current_slot
    global winding_thread, stop_requested

    action = data.get("action")

    if action == "start":
        if current_status in ("winding", "initializing"):
            return  # already running

        stop_requested = False
        phase = data.get("phase")

        if phase is not None:
            winding_thread = threading.Thread(
                target=run_phase_winding, args=(phase,), daemon=True
            )
        else:
            winding_thread = threading.Thread(
                target=run_continuous_winding, daemon=True
            )
        winding_thread.start()

    elif action == "stop":
        stop_requested = True
        current_status = "stopped"
        current_phase = None

    elif action == "estop":
        stop_requested = True
        current_status = "error"
        current_error = "Emergency stop activated"
        current_phase = None
        with winder_lock:
            try:
                winder.estop()
            except Exception:
                pass

    elif action == "init":
        current_status = "initializing"
        pull_wire = data.get("pull_wire", False)

        def do_init():
            global current_status
            try:
                with winder_lock:
                    winder.init_position(pull_wire)
                current_status = "idle"
            except Exception as e:
                current_status = "error"
                global current_error
                current_error = str(e)

        threading.Thread(target=do_init, daemon=True).start()

    elif action == "zero":
        current_status = "initializing"

        def do_zero():
            global current_status
            try:
                with winder_lock:
                    winder.back_to_zero()
                current_status = "idle"
            except Exception as e:
                current_status = "error"
                global current_error
                current_error = str(e)

        threading.Thread(target=do_zero, daemon=True).start()

    elif action == "move_to_slot":
        slot = data.get("slot", 0)
        with winder_lock:
            winder.move_to_slot(slot)
        current_slot = slot

    elif action == "update_settings":
        settings = data.get("settings", {})
        if "turns_per_slot" in settings:
            winder.turns_per_slot = settings["turns_per_slot"]
        if "starts_at" in settings:
            winder.starts_at = settings["starts_at"]
        if "m0_velocity" in settings:
            winder.motor_velocities[0] = settings["m0_velocity"]
        if "m1_velocity" in settings:
            winder.motor_velocities[1] = settings["m1_velocity"]
        if "m2_velocity" in settings:
            winder.motor_velocities[2] = settings["m2_velocity"]
        if "m3_velocity" in settings:
            winder.motor_velocities[3] = settings["m3_velocity"]

        # Broadcast updated settings
        await broadcast(json.dumps({
            "type": "settings",
            "settings": get_settings_dict(),
        }))

    elif action == "connect":
        # Flutter app sends this on connect — just acknowledge
        pass


async def handler(websocket):
    """Handle a WebSocket client connection."""
    clients.add(websocket)
    print(f"Client connected ({len(clients)} total)")

    try:
        # Send initial settings
        await websocket.send(json.dumps({
            "type": "settings",
            "settings": get_settings_dict(),
        }))

        async for message in websocket:
            try:
                data = json.loads(message)
                await handle_command(data)
            except json.JSONDecodeError:
                print(f"Invalid JSON: {message}")
            except Exception as e:
                print(f"Error handling command: {e}")
                traceback.print_exc()
    finally:
        clients.discard(websocket)
        print(f"Client disconnected ({len(clients)} total)")


async def status_broadcaster():
    """Periodically broadcast motor status to all clients."""
    while True:
        if clients:
            try:
                msg = build_status_message()
                await broadcast(msg)
            except Exception:
                pass
        await asyncio.sleep(0.1)  # 10Hz status updates


async def main(host, port):
    print(f"Winder WebSocket Bridge")
    print(f"  Listening on ws://{host}:{port}")
    print(f"  Simulation: {winder.simulation}")
    print(f"  Config: {winder.config.get('winding', {}).get('winding_config', 'N/A')}")
    print(f"  Turns/slot: {winder.turns_per_slot}")
    print()

    async with websockets.serve(handler, host, port):
        await status_broadcaster()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Winder WebSocket Bridge")
    parser.add_argument("--config", "-c", default="settings.yml", help="Config file path")
    parser.add_argument("--port", "-p", type=int, default=8765, help="WebSocket port")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--simulation", "-s", action="store_true", help="Simulation mode")
    args = parser.parse_args()

    winder = Wind(args.config, simulation=args.simulation)
    asyncio.run(main(args.host, args.port))
