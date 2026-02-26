import threading
import socket
import time

try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False


class IPGLaserError(RuntimeError):
    """Raised when the laser returns an error or communication fails."""


class IPGLaserSerialController:
    """RS-232 controller for IPG YLR-200-SM-CS style command set."""

    STATUS_BITS = {
        1: "overtemperature",
        2: "emission_on",
        3: "high_backreflection",
        4: "analog_control_enabled",
        6: "module_disconnected",
        7: "module_failed",
        8: "aiming_beam_on",
        10: "optical_interlock_failed",
        11: "power_supply_off",
        12: "modulation_enabled",
        14: "laser_enable_asserted",
        15: "emission_startup_delay",
        17: "unexpected_emission",
        21: "keyswitch_remote_mode",
        22: "aiming_beam_hardware_control",
        29: "module_disabled",
        30: "collimator_disconnected",
    }

    def __init__(self):
        self._serial = None
        self._socket = None
        self._transport = None
        self._timeout = 1.0
        self._rx_buffer = b""
        self._lock = threading.Lock()

    def connect_serial(self, port, baudrate=57600, timeout=1.0):
        if not HAS_PYSERIAL:
            raise IPGLaserError("pyserial is not installed. Run: pip install pyserial")

        self.disconnect()
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                write_timeout=timeout,
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            self._timeout = timeout
            self._transport = "serial"
        except Exception as exc:
            raise IPGLaserError(f"Failed to open serial port {port}: {exc}") from exc

    def connect_tcp(self, host, port=10001, timeout=1.0):
        self.disconnect()
        try:
            self._socket = socket.create_connection((host, port), timeout=timeout)
            self._socket.settimeout(timeout)
            self._timeout = timeout
            self._rx_buffer = b""
            self._transport = "tcp"
        except Exception as exc:
            raise IPGLaserError(f"Failed to connect to laser at {host}:{port}: {exc}") from exc

    def connect(self, port, baudrate=57600, timeout=1.0):
        """Backward-compatible alias for serial connection."""
        self.connect_serial(port=port, baudrate=baudrate, timeout=timeout)

    def disconnect(self):
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None
        self._transport = None
        self._rx_buffer = b""

    def is_connected(self):
        if self._transport == "serial":
            return self._serial is not None and self._serial.is_open
        if self._transport == "tcp":
            return self._socket is not None
        return False

    @staticmethod
    def _split_line(buffer):
        idx_r = buffer.find(b"\r")
        idx_n = buffer.find(b"\n")
        delimiters = [idx for idx in (idx_r, idx_n) if idx != -1]
        if not delimiters:
            return None, buffer

        cut = min(delimiters)
        line = buffer[:cut]
        rest_start = cut
        while rest_start < len(buffer) and buffer[rest_start] in (13, 10):
            rest_start += 1
        return line, buffer[rest_start:]

    def _read_tcp_line(self):
        deadline = time.monotonic() + self._timeout
        while True:
            line, remaining = self._split_line(self._rx_buffer)
            if line is not None:
                self._rx_buffer = remaining
                return line.decode("ascii", errors="replace").strip()

            if time.monotonic() >= deadline:
                raise IPGLaserError("No response from laser (timeout).")
            try:
                chunk = self._socket.recv(1024)
            except socket.timeout:
                continue
            except Exception as exc:
                raise IPGLaserError(f"TCP communication failed: {exc}") from exc

            if not chunk:
                raise IPGLaserError("TCP connection closed by laser.")
            self._rx_buffer += chunk

    def _send(self, command):
        if not self.is_connected():
            raise IPGLaserError("Laser is not connected.")

        payload = (command.strip() + "\r").encode("ascii", errors="ignore")
        with self._lock:
            if self._transport == "serial":
                try:
                    self._serial.write(payload)
                    self._serial.flush()
                    response = self._serial.readline().decode("ascii", errors="replace").strip()
                except Exception as exc:
                    raise IPGLaserError(f"Serial communication failed: {exc}") from exc
            elif self._transport == "tcp":
                try:
                    self._socket.sendall(payload)
                    response = self._read_tcp_line()
                except IPGLaserError:
                    raise
                except Exception as exc:
                    raise IPGLaserError(f"TCP communication failed: {exc}") from exc
            else:
                raise IPGLaserError("Unsupported transport mode.")

        if not response:
            raise IPGLaserError("No response from laser (timeout).")
        if response.upper().startswith("ERR:"):
            raise IPGLaserError(response)
        return response

    @staticmethod
    def _payload(response, prefix):
        marker = f"{prefix.upper()}:"
        upper = response.upper()
        if upper == prefix.upper():
            return ""
        if upper.startswith(marker):
            return response.split(":", 1)[1].strip()
        raise IPGLaserError(f"Unexpected response for {prefix}: {response}")

    @staticmethod
    def _parse_float(response, prefix):
        value = IPGLaserSerialController._payload(response, prefix)
        try:
            return float(value)
        except ValueError as exc:
            raise IPGLaserError(f"Expected numeric response for {prefix}, got: {response}") from exc

    def set_diode_current(self, percent):
        response = self._send(f"SDC {percent:.2f}")
        return self._parse_float(response, "SDC")

    def read_current_setpoint(self):
        return self._parse_float(self._send("RCS"), "RCS")

    def read_min_current_setpoint(self):
        return self._parse_float(self._send("RNC"), "RNC")

    def read_diode_current(self):
        return self._parse_float(self._send("RDC"), "RDC")

    def read_output_power(self):
        response = self._send("ROP")
        value = self._payload(response, "ROP")
        if value in ("Off", "Low"):
            return value
        try:
            return float(value)
        except ValueError as exc:
            raise IPGLaserError(f"Unexpected ROP response: {response}") from exc

    def read_firmware_version(self):
        return self._payload(self._send("RFV"), "RFV")

    def read_temperature(self):
        return self._parse_float(self._send("RCT"), "RCT")

    def read_status_word(self):
        response = self._send("STA")
        value = self._payload(response, "STA")
        try:
            return int(value)
        except ValueError as exc:
            raise IPGLaserError(f"Unexpected STA response: {response}") from exc

    def start_emission(self):
        return self._send("EMON")

    def stop_emission(self):
        return self._send("EMOFF")

    def enable_modulation(self):
        return self._send("EMOD")

    def disable_modulation(self):
        return self._send("DMOD")

    def enable_external_control(self):
        return self._send("EEC")

    def disable_external_control(self):
        return self._send("DEC")

    def reset_errors(self):
        return self._send("RERR")

    def aiming_beam_on(self):
        return self._send("ABN")

    def aiming_beam_off(self):
        return self._send("ABF")

    def enable_external_aiming_control(self):
        return self._send("EEABC")

    def disable_external_aiming_control(self):
        return self._send("DEABC")

    def set_filter_window(self, seconds):
        response = self._send(f"SFWS {seconds:.2f}")
        return self._parse_float(response, "SFWS")

    def read_filter_window(self):
        return self._parse_float(self._send("RFWS"), "RFWS")

    def decode_status(self, status_word):
        flags = {name: False for name in self.STATUS_BITS.values()}
        for bit, name in self.STATUS_BITS.items():
            flags[name] = bool(status_word & (1 << bit))
        return flags

    def interpret_state(self, flags, output_power):
        if (
            flags["overtemperature"]
            or flags["module_failed"]
            or flags["module_disconnected"]
            or flags["unexpected_emission"]
            or flags["optical_interlock_failed"]
            or flags["collimator_disconnected"]
        ):
            return "Fault"
        if flags["emission_startup_delay"]:
            return "Starting (3s safety delay)"
        if flags["emission_on"]:
            if isinstance(output_power, float) and output_power > 0.5:
                return "Firing"
            if output_power == "Low":
                return "Emission enabled (low output)"
            return "Emission enabled"
        if flags["power_supply_off"]:
            return "Power Supply Off"
        return "Idle"

    def get_snapshot(self):
        status_word = self.read_status_word()
        flags = self.decode_status(status_word)
        output_power = self.read_output_power()
        temperature_c = self.read_temperature()
        return {
            "status_word": status_word,
            "status_hex": f"0x{status_word:08X}",
            "flags": flags,
            "output_power": output_power,
            "temperature_c": temperature_c,
            "state": self.interpret_state(flags, output_power),
        }


def _run_standalone_test_ui():
    import tkinter as tk
    from tkinter import ttk

    controller = IPGLaserSerialController()
    root = tk.Tk()
    root.title("IPG Laser Controller Test")
    root.geometry("780x520")

    connection_mode = tk.StringVar(value="serial")
    serial_port = tk.StringVar(value="/dev/ttyUSB0")
    tcp_host = tk.StringVar(value="192.168.1.100")
    tcp_port = tk.StringVar(value="10001")
    connection_status = tk.StringVar(value="Disconnected")
    state_var = tk.StringVar(value="-")
    emission_var = tk.StringVar(value="-")
    output_var = tk.StringVar(value="-")
    temp_var = tk.StringVar(value="-")
    status_word_var = tk.StringVar(value="-")
    setpoint_var = tk.StringVar(value="20.0")

    status_job = {"id": None}

    outer = tk.Frame(root)
    outer.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    outer.grid_columnconfigure(1, weight=1)
    outer.grid_rowconfigure(2, weight=1)

    connection_frame = tk.LabelFrame(outer, text="Connection")
    connection_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
    connection_frame.grid_columnconfigure(3, weight=1)

    tk.Label(connection_frame, text="Mode").grid(row=0, column=0, sticky="w", padx=6, pady=4)
    tk.Radiobutton(connection_frame, text="Serial", variable=connection_mode, value="serial").grid(
        row=0, column=1, sticky="w", padx=6, pady=4
    )
    tk.Radiobutton(connection_frame, text="TCP/IP", variable=connection_mode, value="tcp").grid(
        row=0, column=2, sticky="w", padx=6, pady=4
    )

    lbl_serial = tk.Label(connection_frame, text="Serial Port")
    lbl_serial.grid(row=1, column=0, sticky="w", padx=6, pady=4)
    ent_serial = tk.Entry(connection_frame, textvariable=serial_port)
    
    port_list = ["/dev/ttyUSB0"]
    if HAS_PYSERIAL:
        ports = serial.tools.list_ports.comports()
        if ports:
            port_list = [p.device for p in ports]

    ent_serial = ttk.Combobox(connection_frame, textvariable=serial_port, values=port_list)
    ent_serial.grid(row=1, column=1, columnspan=3, sticky="ew", padx=6, pady=4)

    lbl_ip = tk.Label(connection_frame, text="IP")
    lbl_ip.grid(row=2, column=0, sticky="w", padx=6, pady=4)
    ent_ip = tk.Entry(connection_frame, textvariable=tcp_host)
    ent_ip.grid(row=2, column=1, sticky="ew", padx=6, pady=4)

    lbl_tcp_port = tk.Label(connection_frame, text="TCP Port")
    lbl_tcp_port.grid(row=2, column=2, sticky="w", padx=6, pady=4)
    ent_tcp_port = tk.Entry(connection_frame, textvariable=tcp_port)
    ent_tcp_port.grid(row=2, column=3, sticky="ew", padx=6, pady=4)

    status_label = tk.Label(connection_frame, textvariable=connection_status, fg="blue")
    status_label.grid(row=3, column=3, sticky="w", padx=6, pady=4)

    status_frame = tk.LabelFrame(outer, text="Live Status")
    status_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
    status_frame.grid_columnconfigure(1, weight=1)

    tk.Label(status_frame, text="State").grid(row=0, column=0, sticky="w", padx=6, pady=4)
    tk.Label(status_frame, textvariable=state_var).grid(row=0, column=1, sticky="w", padx=6, pady=4)
    tk.Label(status_frame, text="Emission").grid(row=1, column=0, sticky="w", padx=6, pady=4)
    tk.Label(status_frame, textvariable=emission_var).grid(row=1, column=1, sticky="w", padx=6, pady=4)
    tk.Label(status_frame, text="Output").grid(row=2, column=0, sticky="w", padx=6, pady=4)
    tk.Label(status_frame, textvariable=output_var).grid(row=2, column=1, sticky="w", padx=6, pady=4)
    tk.Label(status_frame, text="Temperature").grid(row=3, column=0, sticky="w", padx=6, pady=4)
    tk.Label(status_frame, textvariable=temp_var).grid(row=3, column=1, sticky="w", padx=6, pady=4)
    tk.Label(status_frame, text="Status Word").grid(row=4, column=0, sticky="w", padx=6, pady=4)
    tk.Label(status_frame, textvariable=status_word_var).grid(row=4, column=1, sticky="w", padx=6, pady=4)

    controls = tk.LabelFrame(outer, text="Controls")
    controls.grid(row=1, column=1, sticky="nsew")
    controls.grid_columnconfigure(1, weight=1)

    log_frame = tk.LabelFrame(outer, text="Log")
    log_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
    log_frame.grid_columnconfigure(0, weight=1)
    log_frame.grid_rowconfigure(0, weight=1)

    log_text = tk.Text(log_frame, wrap=tk.WORD)
    log_text.grid(row=0, column=0, sticky="nsew")
    scroll = tk.Scrollbar(log_frame, command=log_text.yview)
    scroll.grid(row=0, column=1, sticky="ns")
    log_text.configure(yscrollcommand=scroll.set)

    def log(message):
        log_text.insert(tk.END, message + "\n")
        log_text.see(tk.END)

    def update_mode_fields():
        mode = connection_mode.get()
        serial_state = tk.NORMAL if mode == "serial" else tk.DISABLED
        tcp_state = tk.NORMAL if mode == "tcp" else tk.DISABLED
        lbl_serial.configure(state=serial_state)
        ent_serial.configure(state=serial_state)
        lbl_ip.configure(state=tcp_state)
        ent_ip.configure(state=tcp_state)
        lbl_tcp_port.configure(state=tcp_state)
        ent_tcp_port.configure(state=tcp_state)

    def refresh_status():
        if status_job["id"] is not None:
            root.after_cancel(status_job["id"])
            status_job["id"] = None

        if controller.is_connected():
            try:
                snap = controller.get_snapshot()
                output = snap["output_power"]
                output_txt = f"{output:.2f} W" if isinstance(output, float) else output
                state_var.set(snap["state"])
                emission_var.set("On" if snap["flags"]["emission_on"] else "Off")
                output_var.set(output_txt)
                temp_var.set(f"{snap['temperature_c']:.1f} C")
                status_word_var.set(f"{snap['status_word']} ({snap['status_hex']})")
            except IPGLaserError as exc:
                log(f"Status read failed: {exc}")
        else:
            state_var.set("-")
            emission_var.set("-")
            output_var.set("-")
            temp_var.set("-")
            status_word_var.set("-")

        status_job["id"] = root.after(1000, refresh_status)

    def run_action(action, success_message):
        try:
            value = action()
            if value is None or value == "":
                log(success_message)
            else:
                log(f"{success_message}: {value}")
            refresh_status()
        except IPGLaserError as exc:
            log(f"Command failed: {exc}")

    def toggle_connection():
        if controller.is_connected():
            controller.disconnect()
            connection_status.set("Disconnected")
            btn_connect.configure(text="Connect")
            log("Disconnected.")
            refresh_status()
            return

        try:
            if connection_mode.get() == "tcp":
                host = tcp_host.get().strip()
                port = int(tcp_port.get().strip())
                controller.connect_tcp(host, port)
                log(f"Connected to {host}:{port} (TCP).")
            else:
                port_name = serial_port.get().strip()
                controller.connect_serial(port_name)
                log(f"Connected to {port_name} (Serial, 57600-8N1).")
            connection_status.set("Connected")
            btn_connect.configure(text="Disconnect")
            refresh_status()
        except ValueError:
            connection_status.set("Connection error")
            log("Connection failed: TCP port must be an integer.")
        except IPGLaserError as exc:
            connection_status.set("Connection error")
            log(f"Connection failed: {exc}")

    def set_setpoint():
        try:
            val = float(setpoint_var.get())
        except ValueError:
            log("Invalid setpoint. Enter a numeric value.")
            return
        run_action(lambda: f"{controller.set_diode_current(val):.2f}%", "Setpoint updated")

    btn_connect = tk.Button(connection_frame, text="Connect", width=12, command=toggle_connection)
    btn_connect.grid(row=3, column=2, sticky="ew", padx=6, pady=4)

    tk.Button(controls, text="Refresh", command=refresh_status).grid(row=0, column=0, sticky="ew", padx=6, pady=4)
    tk.Button(controls, text="Reset Errors", command=lambda: run_action(controller.reset_errors, "Errors reset")).grid(
        row=0, column=1, sticky="ew", padx=6, pady=4
    )
    tk.Button(controls, text="Start Emission", bg="#9a2222", fg="white", command=lambda: run_action(controller.start_emission, "Emission start sent")).grid(
        row=1, column=0, sticky="ew", padx=6, pady=4
    )
    tk.Button(controls, text="Stop Emission", bg="#2e7d32", fg="white", command=lambda: run_action(controller.stop_emission, "Emission stop sent")).grid(
        row=1, column=1, sticky="ew", padx=6, pady=4
    )
    tk.Label(controls, text="Current Setpoint (%)").grid(row=2, column=0, sticky="w", padx=6, pady=4)
    tk.Entry(controls, textvariable=setpoint_var).grid(row=2, column=1, sticky="ew", padx=6, pady=4)
    tk.Button(controls, text="Set Setpoint", command=set_setpoint).grid(row=3, column=0, sticky="ew", padx=6, pady=4)
    tk.Button(
        controls,
        text="Read Setpoint",
        command=lambda: run_action(lambda: f"{controller.read_current_setpoint():.2f}%", "Current setpoint"),
    ).grid(row=3, column=1, sticky="ew", padx=6, pady=4)
    tk.Button(
        controls,
        text="Read Firmware",
        command=lambda: run_action(controller.read_firmware_version, "Firmware"),
    ).grid(row=4, column=0, sticky="ew", padx=6, pady=4)
    tk.Button(
        controls,
        text="Read Diode Current",
        command=lambda: run_action(lambda: f"{controller.read_diode_current():.2f} A", "Diode current"),
    ).grid(row=4, column=1, sticky="ew", padx=6, pady=4)

    def on_close():
        if status_job["id"] is not None:
            root.after_cancel(status_job["id"])
            status_job["id"] = None
        controller.disconnect()
        root.destroy()

    connection_mode.trace_add("write", lambda *_: update_mode_fields())
    update_mode_fields()
    root.protocol("WM_DELETE_WINDOW", on_close)

    if not HAS_PYSERIAL:
        log("pyserial not installed. Serial mode unavailable; TCP mode still works.")
    refresh_status()
    root.mainloop()


if __name__ == "__main__":
    _run_standalone_test_ui()
