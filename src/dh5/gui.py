"""
DH5 6-Axis Device - GUI Slider Control
========================================

A simple Tkinter GUI that lets you drag sliders to move each of the 6 axes
of a DH5 device in real time, using the DH5ModbusAPI class.

Features:
- One slider per axis (1-6), each with its own configurable motion range
- Live numeric readout of the slider (target) value
- Optional live feedback readout of actual position/speed/current per axis
  (polled in a background thread so the UI doesn't freeze)
- Connect / Disconnect and Initialize buttons
- Global speed & force controls applied to all axes before moving
- Slider moves are throttled (sent on release + periodic drag updates)
  so you don't flood the serial bus with a command per pixel of drag

Adjust AXIS_RANGES below to match the real motion range of your device
(the docs did not specify exact min/max values per axis, so defaults
here are placeholders based on the example in the API doc).

Run:
    python dh5_gui.py
"""

import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

from dh5.modbus import DH5ModbusAPI


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------
PORT = "COM3"
MODBUS_ID = 1
BAUD_RATE = 115200
STOP_BITS = 1
PARITY = "N"

NUM_AXES = 6
INIT_MODE = 0b10  # 0b01 closed / 0b10 open / 0b11 full-stroke search

# Motion range per axis: (min_position, max_position, default_position)
# NOTE: placeholder ranges based on the API doc's example values.
# Update these to the actual mechanical limits of your device/fingers.
AXIS_RANGES = {
    1: (0, 1000, 500),
    2: (0, 100, 10),
    3: (0, 100, 10),
    4: (0, 100, 10),
    5: (0, 100, 10),
    6: (0, 1000, 500),
}

DEFAULT_SPEED = 10
DEFAULT_FORCE = 100

FEEDBACK_POLL_INTERVAL = 0.5  # seconds between position/speed/current reads
DRAG_SEND_INTERVAL = 0.08     # min seconds between commands while dragging


class AxisControl:
    """Holds the widgets and state for a single axis's slider row."""

    def __init__(self, parent, axis, axis_range, on_move):
        self.axis = axis
        self.min_pos, self.max_pos, self.default_pos = axis_range
        self.on_move = on_move
        self._last_sent_time = 0
        self._last_sent_value = None

        self.frame = ttk.Frame(parent, padding=6)
        self.frame.grid(sticky="ew")
        self.frame.columnconfigure(2, weight=1)

        ttk.Label(self.frame, text=f"Axis {axis}", width=8).grid(row=0, column=0, sticky="w")

        self.value_var = tk.IntVar(value=self.default_pos)
        self.value_label = ttk.Label(self.frame, text=str(self.default_pos), width=6)
        self.value_label.grid(row=0, column=1, sticky="w")

        self.slider = ttk.Scale(
            self.frame,
            from_=self.min_pos,
            to=self.max_pos,
            orient="horizontal",
            variable=self.value_var,
            command=self._on_slider_change,
        )
        self.slider.grid(row=0, column=2, sticky="ew", padx=8)
        self.slider.bind("<ButtonRelease-1>", self._on_release)

        self.feedback_var = tk.StringVar(value="pos: --   speed: --   current: --")
        ttk.Label(self.frame, textvariable=self.feedback_var, foreground="#555").grid(
            row=1, column=0, columnspan=3, sticky="w"
        )

    def _on_slider_change(self, value_str):
        value = int(float(value_str))
        self.value_label.config(text=str(value))

        now = time.time()
        # Throttle: only send while dragging if enough time has passed
        # and the value actually changed.
        if (now - self._last_sent_time) >= DRAG_SEND_INTERVAL and value != self._last_sent_value:
            self._last_sent_time = now
            self._last_sent_value = value
            self.on_move(self.axis, value)

    def _on_release(self, _event):
        # Always send the final value on release, even if throttled during drag.
        value = int(self.value_var.get())
        self._last_sent_time = time.time()
        self._last_sent_value = value
        self.on_move(self.axis, value)

    def set_feedback_text(self, text):
        self.feedback_var.set(text)

    def set_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        self.slider.state(["!disabled"] if enabled else ["disabled"])


class DH5GuiApp:
    def __init__(self, root):
        self.root = root
        self.root.title("DH5 6-Axis Slider Control")

        self.api = None
        self.connected = False
        self.initialized = False

        self._poll_thread = None
        self._poll_stop = threading.Event()
        self._api_lock = threading.Lock()  # serial access from GUI + poll thread

        self._build_top_controls()
        self._build_axis_controls()
        self._build_status_bar()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------
    # UI construction
    # ---------------------------------------------------------------
    def _build_top_controls(self):
        top = ttk.Frame(self.root, padding=8)
        top.grid(row=0, column=0, sticky="ew")

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value=PORT)
        ttk.Entry(top, textvariable=self.port_var, width=10).grid(row=0, column=1, padx=(0, 12))

        ttk.Label(top, text="Baud:").grid(row=0, column=2, sticky="w")
        self.baud_var = tk.StringVar(value=str(BAUD_RATE))
        ttk.Entry(top, textvariable=self.baud_var, width=10).grid(row=0, column=3, padx=(0, 12))

        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=4, padx=4)

        self.init_btn = ttk.Button(top, text="Initialize", command=self._initialize, state="disabled")
        self.init_btn.grid(row=0, column=5, padx=4)

        ttk.Separator(top, orient="vertical").grid(row=0, column=6, sticky="ns", padx=10)

        ttk.Label(top, text="Speed:").grid(row=0, column=7, sticky="w")
        self.speed_var = tk.IntVar(value=DEFAULT_SPEED)
        ttk.Spinbox(top, from_=1, to=1000, textvariable=self.speed_var, width=6).grid(row=0, column=8)

        ttk.Label(top, text="Force:").grid(row=0, column=9, sticky="w", padx=(10, 0))
        self.force_var = tk.IntVar(value=DEFAULT_FORCE)
        ttk.Spinbox(top, from_=1, to=1000, textvariable=self.force_var, width=6).grid(row=0, column=10)

        self.apply_speed_force_btn = ttk.Button(
            top, text="Apply Speed/Force", command=self._apply_speed_force, state="disabled"
        )
        self.apply_speed_force_btn.grid(row=0, column=11, padx=8)

        self.reset_faults_btn = ttk.Button(
            top, text="Reset Faults", command=self._reset_faults, state="disabled"
        )
        self.reset_faults_btn.grid(row=0, column=12, padx=4)

    def _build_axis_controls(self):
        container = ttk.LabelFrame(self.root, text="Axis Position Control", padding=8)
        container.grid(row=1, column=0, sticky="ew", padx=8, pady=8)
        container.columnconfigure(0, weight=1)

        self.axis_controls = {}
        for axis in range(1, NUM_AXES + 1):
            axis_range = AXIS_RANGES.get(axis, (0, 100, 0))
            control = AxisControl(container, axis, axis_range, on_move=self._send_position)
            control.set_enabled(False)
            self.axis_controls[axis] = control

    def _build_status_bar(self):
        self.status_var = tk.StringVar(value="Disconnected.")
        ttk.Label(self.root, textvariable=self.status_var, anchor="w", relief="sunken").grid(
            row=2, column=0, sticky="ew", padx=8, pady=(0, 8)
        )

    # ---------------------------------------------------------------
    # Connection handling
    # ---------------------------------------------------------------
    def _toggle_connect(self):
        if not self.connected:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Invalid baud rate", "Baud rate must be an integer.")
            return

        try:
            self.api = DH5ModbusAPI(
                port=self.port_var.get(),
                modbus_id=MODBUS_ID,
                baud_rate=baud,
                stop_bits=STOP_BITS,
                parity=PARITY,
            )
            status = self.api.open_connection()
            if status != self.api.SUCCESS:
                raise RuntimeError(f"open_connection() returned {status}")
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.api = None
            return

        self.connected = True
        self.status_var.set(f"Connected on {self.port_var.get()}.")
        self.connect_btn.config(text="Disconnect")
        self.init_btn.config(state="normal")
        self.apply_speed_force_btn.config(state="normal")
        self.reset_faults_btn.config(state="normal")

        self._start_feedback_polling()

    def _disconnect(self):
        self._stop_feedback_polling()

        if self.api is not None:
            try:
                with self._api_lock:
                    self.api.close_connection()
            except Exception:
                pass

        self.connected = False
        self.initialized = False
        self.api = None

        self.connect_btn.config(text="Connect")
        self.init_btn.config(state="disabled")
        self.apply_speed_force_btn.config(state="disabled")
        self.reset_faults_btn.config(state="disabled")
        for control in self.axis_controls.values():
            control.set_enabled(False)

        self.status_var.set("Disconnected.")

    # ---------------------------------------------------------------
    # Device commands
    # ---------------------------------------------------------------
    def _initialize(self):
        if not self.connected or self.api is None:
            return

        self.status_var.set("Initializing all axes...")
        self.init_btn.config(state="disabled")

        def worker():
            try:
                with self._api_lock:
                    result = self.api.initialize(INIT_MODE)
                    if result != self.api.SUCCESS:
                        raise RuntimeError(f"initialize() returned {result}")

                    # Poll until all axes report initialized (simple bounded wait)
                    for _ in range(40):  # ~20s at 0.5s interval
                        status = self.api.check_initialization()
                        if all(v == "initialized" for v in status.values()):
                            break
                        time.sleep(0.5)
                    else:
                        raise RuntimeError(f"Timed out. Last status: {status}")

                # Apply default speed/force once axes are ready
                for axis in range(1, NUM_AXES + 1):
                    self.api.set_axis_speed(axis, self.speed_var.get())
                    self.api.set_axis_force(axis, self.force_var.get())

            except Exception as exc:
                self.root.after(0, lambda: self._on_init_error(exc))
                return

            self.root.after(0, self._on_init_success)

        threading.Thread(target=worker, daemon=True).start()

    def _on_init_success(self):
        self.initialized = True
        self.status_var.set("Initialization complete. Axes ready.")
        self.init_btn.config(state="normal")
        for control in self.axis_controls.values():
            control.set_enabled(True)

    def _on_init_error(self, exc):
        self.status_var.set(f"Initialization failed: {exc}")
        self.init_btn.config(state="normal")
        messagebox.showerror("Initialization failed", str(exc))

    def _apply_speed_force(self):
        if not self.connected or self.api is None:
            return

        def worker():
            try:
                with self._api_lock:
                    for axis in range(1, NUM_AXES + 1):
                        self.api.set_axis_speed(axis, self.speed_var.get())
                        self.api.set_axis_force(axis, self.force_var.get())
                self.root.after(0, lambda: self.status_var.set("Speed/force applied to all axes."))
            except Exception as exc:
                self.root.after(0, lambda: self.status_var.set(f"Speed/force update failed: {exc}"))

        threading.Thread(target=worker, daemon=True).start()

    def _reset_faults(self):
        if not self.connected or self.api is None:
            return

        def worker():
            try:
                with self._api_lock:
                    result = self.api.reset_faults()
                self.root.after(0, lambda: self.status_var.set(f"reset_faults() -> {result}"))
            except Exception as exc:
                self.root.after(0, lambda: self.status_var.set(f"Fault reset failed: {exc}"))

        threading.Thread(target=worker, daemon=True).start()

    def _send_position(self, axis, value):
        """Called from the slider callback (GUI thread) to send a move command
        for a single axis. Runs the actual serial call in a background thread
        so dragging the slider never freezes the UI."""
        if not self.connected or not self.initialized or self.api is None:
            return

        def worker():
            try:
                with self._api_lock:
                    self.api.set_axis_position(axis, value)
            except Exception as exc:
                self.root.after(0, lambda: self.status_var.set(f"Axis {axis} move failed: {exc}"))

        threading.Thread(target=worker, daemon=True).start()

    # ---------------------------------------------------------------
    # Background feedback polling
    # ---------------------------------------------------------------
    def _start_feedback_polling(self):
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def _stop_feedback_polling(self):
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=1.0)
        self._poll_thread = None

    def _poll_loop(self):
        while not self._poll_stop.is_set():
            if self.connected and self.api is not None:
                for axis in range(1, NUM_AXES + 1):
                    try:
                        with self._api_lock:
                            pos = self.api.get_axis_position(axis)
                            spd = self.api.get_axis_speed(axis)
                            cur = self.api.get_axis_current(axis)
                        text = f"pos: {pos}   speed: {spd}   current: {cur}"
                    except Exception as exc:
                        text = f"feedback error: {exc}"

                    control = self.axis_controls.get(axis)
                    if control is not None:
                        self.root.after(0, lambda c=control, t=text: c.set_feedback_text(t))

            self._poll_stop.wait(FEEDBACK_POLL_INTERVAL)

    # ---------------------------------------------------------------
    # Cleanup
    # ---------------------------------------------------------------
    def _on_close(self):
        self._stop_feedback_polling()
        if self.connected:
            self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = DH5GuiApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()