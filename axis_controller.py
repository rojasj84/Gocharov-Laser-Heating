import tkinter as tk
import sys
import os
from ipg_laser_controller import IPGLaserSerialController, IPGLaserError, HAS_PYSERIAL
try:
    from PIL import Image, ImageDraw, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

try:
    # For Pythonnet 3.0+ on Linux/Mono, explicitly load the runtime
    try:
        from pythonnet import load
        load("mono")
    except Exception:
        pass
    import clr
    HAS_PYTHONNET = True
except ImportError:
    HAS_PYTHONNET = False

class AxisController:
    def __init__(self, root):
        self.root = root
        self.root.title("XY Axis Control")
        
        # Set a reasonable default size
        self.root.geometry("750x350")

        self.create_menu()

        # Dictionary to store entry widgets for each axis
        self.axis_entries = {}

        # Sample Holder Position Frame
        self.frm_sample = self.create_axis_panel(root, "Sample Holder Position", "Sample")
        self.frm_sample.grid(row=0, column=0, padx=10, pady=10, sticky="n")

        # Objective Lense Position Frame
        self.frm_objective = self.create_axis_panel(root, "Objective Lense Position", "Objective")
        self.frm_objective.grid(row=0, column=1, padx=10, pady=10, sticky="n")

        # Flipper Mirrors Section
        self.frm_mirrors = tk.LabelFrame(root, text="Flipper Mirrors")
        self.frm_mirrors.grid(row=0, column=2, padx=10, pady=10, sticky="ns")

        self.var_mirror1 = tk.BooleanVar()
        self.chk_mirror1 = tk.Checkbutton(self.frm_mirrors, text="Mirror 1", variable=self.var_mirror1, indicatoron=False, width=10, command=self.toggle_mirror1)
        self.chk_mirror1.pack(padx=5, pady=10)

        self.var_mirror2 = tk.BooleanVar()
        self.chk_mirror2 = tk.Checkbutton(self.frm_mirrors, text="Mirror 2", variable=self.var_mirror2, indicatoron=False, width=10, command=self.toggle_mirror2)
        self.chk_mirror2.pack(padx=5, pady=10)

        # Center the grid in the window
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_columnconfigure(2, weight=1)
        
        # Initialize Aerotech libraries
        self.aerotech = None
        self.load_aerotech_libraries()
        
        # Connect Button
        self.btn_connect = tk.Button(root, text="Connect to Controller", command=self.connect_controller, bg="green", fg="white")
        self.btn_connect.grid(row=1, column=0, columnspan=2, pady=10, sticky="ew", padx=10)

        # Raster Button
        self.btn_raster = tk.Button(root, text="Raster Laser", command=self.open_raster_window, bg="purple", fg="white")
        self.btn_raster.grid(row=1, column=2, pady=10, sticky="ew", padx=10)

        # Laser controller state
        self.laser = IPGLaserSerialController()
        self._laser_status_job = None

        # Create the secondary window for laser operations
        self.create_laser_operation_window()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def create_menu(self):
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File Menu
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Exit", command=self.on_close)
        menubar.add_cascade(label="File", menu=file_menu)

        # Settings Menu
        settings_menu = tk.Menu(menubar, tearoff=0)
        settings_menu.add_command(label="Laser Controls", command=self.show_laser_window)
        menubar.add_cascade(label="Settings", menu=settings_menu)

    def show_laser_window(self):
        self.laser_window.deiconify()

    def create_axis_panel(self, parent, title, axis_name):
        frame = tk.LabelFrame(parent, text=title)
        
        # Buttons
        btn_up = self.create_arrow_button(frame, "up", lambda: self.move(axis_name, "UP"))
        btn_down = self.create_arrow_button(frame, "down", lambda: self.move(axis_name, "DOWN"))
        btn_left = self.create_arrow_button(frame, "left", lambda: self.move(axis_name, "LEFT"))
        btn_right = self.create_arrow_button(frame, "right", lambda: self.move(axis_name, "RIGHT"))
        
        btn_stop = tk.Button(frame, text="STOP", command=lambda: self.stop(axis_name), width=6, height=1, bg="red", fg="white")

        # Grid Layout
        btn_up.grid(row=0, column=1, padx=2, pady=2)
        btn_left.grid(row=1, column=0, padx=2, pady=2)
        btn_stop.grid(row=1, column=1, padx=2, pady=2)
        btn_right.grid(row=1, column=2, padx=2, pady=2)
        btn_down.grid(row=2, column=1, padx=2, pady=2)

        # Settings inputs
        lbl_step = tk.Label(frame, text="Step Size:")
        ent_step = tk.Entry(frame)
        lbl_velocity = tk.Label(frame, text="Velocity:")
        ent_velocity = tk.Entry(frame)

        lbl_step.grid(row=3, column=0, padx=2, pady=2, sticky="e")
        ent_step.grid(row=3, column=1, columnspan=2, padx=2, pady=2, sticky="ew")
        lbl_velocity.grid(row=4, column=0, padx=2, pady=2, sticky="e")
        ent_velocity.grid(row=4, column=1, columnspan=2, padx=2, pady=2, sticky="ew")

        self.axis_entries[axis_name] = {"step": ent_step, "velocity": ent_velocity}
        return frame

    def create_arrow_button(self, parent, direction, command):
        size = 50
        bg_color = parent.cget("bg")
        
        if HAS_PIL:
            # High-quality rendering with Pillow
            scale = 4
            img_size = size * scale
            image = Image.new("RGBA", (img_size, img_size), (0, 0, 0, 0))
            draw = ImageDraw.Draw(image)
            
            pad = 2 * scale
            draw.ellipse([pad, pad, img_size-pad, img_size-pad], fill="yellow", outline="black", width=2*scale)
            
            cx, cy = img_size / 2, img_size / 2
            r_tip = 19 * scale
            r_base = 11 * scale
            r_wide = 15 * scale
            
            if direction == "up": points = [(cx, cy - r_tip), (cx - r_wide, cy + r_base), (cx + r_wide, cy + r_base)]
            elif direction == "down": points = [(cx, cy + r_tip), (cx - r_wide, cy - r_base), (cx + r_wide, cy - r_base)]
            elif direction == "left": points = [(cx - r_tip, cy), (cx + r_base, cy - r_wide), (cx + r_base, cy + r_wide)]
            elif direction == "right": points = [(cx + r_tip, cy), (cx - r_base, cy - r_wide), (cx - r_base, cy + r_wide)]
            
            draw.polygon(points, fill="blue")
            
            resample_method = Image.Resampling.LANCZOS if hasattr(Image, "Resampling") else Image.LANCZOS
            image = image.resize((size, size), resample_method)
            photo = ImageTk.PhotoImage(image)
            
            label = tk.Label(parent, image=photo, bg=bg_color, bd=0)
            label.image = photo
            label.bind("<Button-1>", lambda event: command())
            return label
        else:
            canvas = tk.Canvas(parent, width=size, height=size, bg=bg_color, highlightthickness=0)
            pad = 2
            canvas.create_oval(pad, pad, size-pad, size-pad, fill="yellow", outline="black", width=2)
            cx, cy = size / 2, size / 2
            r_tip = 19
            r_base = 11
            r_wide = 15
            if direction == "up": points = [cx, cy - r_tip, cx - r_wide, cy + r_base, cx + r_wide, cy + r_base]
            elif direction == "down": points = [cx, cy + r_tip, cx - r_wide, cy - r_base, cx + r_wide, cy - r_base]
            elif direction == "left": points = [cx - r_tip, cy, cx + r_base, cy - r_wide, cx + r_base, cy + r_wide]
            elif direction == "right": points = [cx + r_tip, cy, cx - r_base, cy - r_wide, cx - r_base, cy + r_wide]
            canvas.create_polygon(points, fill="blue", outline="blue", width=2, joinstyle=tk.ROUND)
            canvas.bind("<Button-1>", lambda event: command())
            return canvas

    def move(self, axis, direction):
        print(f"Command: Move {axis} Axis {direction}")
        # TODO: Add hardware control logic here

    def stop(self, axis):
        print(f"Command: STOP {axis} motion")
        # TODO: Add hardware control logic here

    def toggle_mirror1(self):
        state = "ON" if self.var_mirror1.get() else "OFF"
        print(f"Command: Mirror 1 {state}")

    def toggle_mirror2(self):
        state = "ON" if self.var_mirror2.get() else "OFF"
        print(f"Command: Mirror 2 {state}")

    def connect_controller(self):
        if not HAS_PYTHONNET:
            print("Error: Pythonnet library not loaded. Cannot connect.")
            return
        print("Command: Connect to Aerotech Controller via .NET DLL")
        # TODO: Add specific Aerotech connection code here

    def load_aerotech_libraries(self):
        """Loads the Aerotech.Ensemble.dll using pythonnet."""
        if not HAS_PYTHONNET:
            print("Failed to load Aerotech libraries: 'pythonnet' could not be imported.")
            print("Verify that 'pip install pythonnet' is run.")
            print("On Linux, ensure Mono is installed (e.g., 'sudo apt install mono-complete').")
            return

        try:
            # Add current directory to path so pythonnet can find the local DLLs
            dll_path = os.path.dirname(os.path.abspath(__file__))
            if dll_path not in sys.path:
                sys.path.append(dll_path)
            
            clr.AddReference("Aerotech.Ensemble")
            import Aerotech.Ensemble
            self.aerotech = Aerotech.Ensemble
            print("Aerotech.Ensemble.dll loaded successfully.")
        except Exception as e:
            print(f"Failed to load Aerotech libraries: {e}")
            print("Ensure the DLLs are in the script directory.")

    def open_raster_window(self):
        window = tk.Toplevel(self.root)
        window.title("Raster Scan Configuration")
        window.geometry("600x550")
        
        # Control Panel (Left)
        frame_controls = tk.Frame(window)
        frame_controls.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        tk.Label(frame_controls, text="Raster Width (X) [µm]").pack(anchor="w")
        entry_width = tk.Entry(frame_controls)
        entry_width.pack(fill=tk.X, pady=(0, 10))
        entry_width.insert(0, "1000.0")
        
        tk.Label(frame_controls, text="Raster Height (Y) [µm]").pack(anchor="w")
        entry_height = tk.Entry(frame_controls)
        entry_height.pack(fill=tk.X, pady=(0, 10))
        entry_height.insert(0, "1000.0")
        
        tk.Label(frame_controls, text="Line Spacing [µm]").pack(anchor="w")
        entry_spacing = tk.Entry(frame_controls)
        entry_spacing.pack(fill=tk.X, pady=(0, 10))
        entry_spacing.insert(0, "100.0")
        
        btn_preview = tk.Button(frame_controls, text="Preview Path", 
                                command=lambda: self.draw_raster_preview(canvas, entry_width, entry_height, entry_spacing))
        btn_preview.pack(fill=tk.X, pady=(20, 10))

        btn_raster_laser = tk.Button(frame_controls, text="Raster w/ Laser", bg="#9a2222", fg="white",
                                     command=lambda: self.run_raster(entry_width, entry_height, entry_spacing, laser=True))
        btn_raster_laser.pack(fill=tk.X, pady=5)

        btn_raster_no_laser = tk.Button(frame_controls, text="Raster w/o Laser", bg="#2e7d32", fg="white",
                                        command=lambda: self.run_raster(entry_width, entry_height, entry_spacing, laser=False))
        btn_raster_no_laser.pack(fill=tk.X, pady=5)
        
        # Visualization (Right)
        canvas = tk.Canvas(window, bg="white", width=400, height=400)
        canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Initial draw
        window.after(100, lambda: self.draw_raster_preview(canvas, entry_width, entry_height, entry_spacing))

    def draw_raster_preview(self, canvas, ent_w, ent_h, ent_s):
        canvas.delete("all")
        try:
            w, h, s = float(ent_w.get()), float(ent_h.get()), float(ent_s.get())
        except ValueError: return

        if w <= 0 or h <= 0 or s <= 0: return

        # Generate path points
        points = []
        x, y, direction = 0.0, 0.0, 1
        points.append((x, y))
        
        while y < h:
            x = w if direction == 1 else 0.0
            points.append((x, y)) # Move across
            if y + s > h: break
            y += s
            points.append((x, y)) # Move down
            direction *= -1

        # Scale to canvas
        cw, ch = max(canvas.winfo_width(), 400), max(canvas.winfo_height(), 400)
        margin = 20
        scale = min((cw - 2*margin) / w, (ch - 2*margin) / h)
        
        flat_points = []
        for px, py in points:
            flat_points.extend([margin + px * scale, margin + py * scale])
            
        if len(flat_points) >= 4:
            canvas.create_line(flat_points, fill="red", width=2, arrow=tk.LAST)
            canvas.create_oval(flat_points[0]-4, flat_points[1]-4, flat_points[0]+4, flat_points[1]+4, fill="green")

    def run_raster(self, ent_w, ent_h, ent_s, laser):
        try:
            w = float(ent_w.get())
            h = float(ent_h.get())
            s = float(ent_s.get())
        except ValueError:
            print("Invalid raster parameters")
            return
        print(f"Raster requested: Width={w}, Height={h}, Spacing={s}, Laser={laser}")

    def create_laser_operation_window(self):
        self.laser_window = tk.Toplevel(self.root)
        self.laser_window.title("IPG Laser Operation")
        self.laser_window.geometry("600x800")
        self.laser_window.protocol("WM_DELETE_WINDOW", self.laser_window.withdraw)

        self.laser_port_var = tk.StringVar(value="/dev/ttyUSB0")
        self.laser_connection_mode_var = tk.StringVar(value="serial")
        self.laser_ip_var = tk.StringVar(value="192.168.1.100")
        self.laser_tcp_port_var = tk.StringVar(value="10001")
        self.laser_connection_var = tk.StringVar(value="Disconnected")
        self.laser_state_var = tk.StringVar(value="Unknown")
        self.laser_emission_var = tk.StringVar(value="Off")
        self.laser_output_var = tk.StringVar(value="-")
        self.laser_temp_var = tk.StringVar(value="-")
        self.laser_status_word_var = tk.StringVar(value="-")
        self.laser_setpoint_var = tk.StringVar(value="20.0")
        self.laser_filter_window_var = tk.StringVar(value="1.0")

        frame = tk.Frame(self.laser_window)
        frame.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)
        frame.grid_columnconfigure(1, weight=1)

        connection_frame = tk.LabelFrame(frame, text="RS-232 Connection")
        connection_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        connection_frame.grid_columnconfigure(1, weight=1)
        connection_frame.grid_columnconfigure(3, weight=1)

        tk.Label(connection_frame, text="Mode").grid(row=0, column=0, sticky="w", padx=8, pady=6)
        tk.Radiobutton(
            connection_frame,
            text="Serial",
            variable=self.laser_connection_mode_var,
            value="serial",
            command=self.update_laser_connection_mode_ui,
        ).grid(row=0, column=1, sticky="w", padx=8, pady=6)
        tk.Radiobutton(
            connection_frame,
            text="TCP/IP",
            variable=self.laser_connection_mode_var,
            value="tcp",
            command=self.update_laser_connection_mode_ui,
        ).grid(row=0, column=2, sticky="w", padx=8, pady=6)

        self.lbl_serial_port = tk.Label(connection_frame, text="Serial Port")
        self.lbl_serial_port.grid(row=1, column=0, sticky="w", padx=8, pady=6)
        self.ent_serial_port = tk.Entry(connection_frame, textvariable=self.laser_port_var)
        self.ent_serial_port.grid(row=1, column=1, columnspan=3, sticky="ew", padx=8, pady=6)

        self.lbl_ip = tk.Label(connection_frame, text="IP")
        self.lbl_ip.grid(row=2, column=0, sticky="w", padx=8, pady=6)
        self.ent_ip = tk.Entry(connection_frame, textvariable=self.laser_ip_var)
        self.ent_ip.grid(row=2, column=1, sticky="ew", padx=8, pady=6)
        self.lbl_tcp_port = tk.Label(connection_frame, text="TCP Port")
        self.lbl_tcp_port.grid(row=2, column=2, sticky="w", padx=8, pady=6)
        self.ent_tcp_port = tk.Entry(connection_frame, textvariable=self.laser_tcp_port_var)
        self.ent_tcp_port.grid(row=2, column=3, sticky="ew", padx=8, pady=6)

        self.btn_laser_connect = tk.Button(connection_frame, text="Connect", width=12, command=self.toggle_laser_connection)
        self.btn_laser_connect.grid(row=3, column=2, padx=8, pady=8, sticky="ew")
        tk.Label(connection_frame, textvariable=self.laser_connection_var, fg="blue").grid(row=3, column=3, padx=8, pady=8, sticky="w")
        self.update_laser_connection_mode_ui()

        status_frame = tk.LabelFrame(frame, text="Live Status")
        status_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        status_frame.grid_columnconfigure(1, weight=1)
        tk.Label(status_frame, text="State").grid(row=0, column=0, sticky="w", padx=8, pady=4)
        tk.Label(status_frame, textvariable=self.laser_state_var).grid(row=0, column=1, sticky="w", padx=8, pady=4)
        tk.Label(status_frame, text="Emission").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        tk.Label(status_frame, textvariable=self.laser_emission_var).grid(row=1, column=1, sticky="w", padx=8, pady=4)
        tk.Label(status_frame, text="Output Power").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        tk.Label(status_frame, textvariable=self.laser_output_var).grid(row=2, column=1, sticky="w", padx=8, pady=4)
        tk.Label(status_frame, text="Temperature").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        tk.Label(status_frame, textvariable=self.laser_temp_var).grid(row=3, column=1, sticky="w", padx=8, pady=4)
        tk.Label(status_frame, text="Status Word").grid(row=4, column=0, sticky="w", padx=8, pady=4)
        tk.Label(status_frame, textvariable=self.laser_status_word_var).grid(row=4, column=1, sticky="w", padx=8, pady=4)
        self.btn_refresh_status = tk.Button(status_frame, text="Refresh Now", command=self.refresh_laser_status)
        self.btn_refresh_status.grid(row=5, column=0, columnspan=2, sticky="ew", padx=8, pady=8)

        controls_frame = tk.LabelFrame(frame, text="Laser Controls")
        controls_frame.grid(row=1, column=1, sticky="nsew")
        controls_frame.grid_columnconfigure(1, weight=1)

        tk.Button(controls_frame, text="Start Emission", bg="#9a2222", fg="white", command=self.start_emission).grid(row=0, column=0, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Stop Emission", bg="#2e7d32", fg="white", command=self.stop_emission).grid(row=0, column=1, sticky="ew", padx=8, pady=6)

        tk.Label(controls_frame, text="Current Setpoint (%)").grid(row=1, column=0, sticky="w", padx=8, pady=6)
        tk.Entry(controls_frame, textvariable=self.laser_setpoint_var).grid(row=1, column=1, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Set Power (SDC)", command=self.set_laser_power).grid(row=2, column=0, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Read Setpoint", command=self.read_laser_setpoint).grid(row=2, column=1, sticky="ew", padx=8, pady=6)

        tk.Label(controls_frame, text="Filter Window (s)").grid(row=3, column=0, sticky="w", padx=8, pady=6)
        tk.Entry(controls_frame, textvariable=self.laser_filter_window_var).grid(row=3, column=1, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Set Filter (SFWS)", command=self.set_filter_window).grid(row=4, column=0, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Read Filter (RFWS)", command=self.read_filter_window).grid(row=4, column=1, sticky="ew", padx=8, pady=6)

        tk.Button(controls_frame, text="Enable Modulation", command=self.enable_modulation).grid(row=5, column=0, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Disable Modulation", command=self.disable_modulation).grid(row=5, column=1, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Enable Ext. Control", command=self.enable_external_control).grid(row=6, column=0, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Disable Ext. Control", command=self.disable_external_control).grid(row=6, column=1, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Aiming Beam ON", command=self.aiming_beam_on).grid(row=7, column=0, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Aiming Beam OFF", command=self.aiming_beam_off).grid(row=7, column=1, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Reset Errors (RERR)", command=self.reset_laser_errors).grid(row=8, column=0, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Read Firmware (RFV)", command=self.read_firmware).grid(row=8, column=1, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Read Diode Current", command=self.read_diode_current).grid(row=9, column=0, sticky="ew", padx=8, pady=6)
        tk.Button(controls_frame, text="Read Min Setpoint", command=self.read_min_setpoint).grid(row=9, column=1, sticky="ew", padx=8, pady=6)

        log_frame = tk.LabelFrame(frame, text="Command Log")
        log_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        frame.grid_rowconfigure(2, weight=1)
        log_frame.grid_rowconfigure(0, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)

        self.txt_laser_log = tk.Text(log_frame, height=10, wrap=tk.WORD)
        self.txt_laser_log.grid(row=0, column=0, sticky="nsew")
        scrollbar = tk.Scrollbar(log_frame, command=self.txt_laser_log.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.txt_laser_log.configure(yscrollcommand=scrollbar.set)

        if not HAS_PYSERIAL:
            self.log_laser("pyserial is not installed. Laser serial control disabled.")

        self.laser_window.withdraw()

    def log_laser(self, message):
        self.txt_laser_log.insert(tk.END, message + "\n")
        self.txt_laser_log.see(tk.END)

    def toggle_laser_connection(self):
        if self.laser.is_connected():
            self.laser.disconnect()
            self.laser_connection_var.set("Disconnected")
            self.btn_laser_connect.configure(text="Connect")
            self.log_laser("Disconnected from laser.")
            return

        try:
            mode = self.laser_connection_mode_var.get()
            if mode == "tcp":
                host = self.laser_ip_var.get().strip()
                port = int(self.laser_tcp_port_var.get())
                self.laser.connect_tcp(host, port)
                connected_target = f"{host}:{port} (TCP)"
            else:
                serial_port = self.laser_port_var.get().strip()
                self.laser.connect_serial(serial_port)
                connected_target = f"{serial_port} (Serial)"

            self.laser_connection_var.set("Connected")
            self.btn_laser_connect.configure(text="Disconnect")
            self.log_laser(f"Connected to {connected_target}.")
            self.refresh_laser_status()
        except ValueError:
            self.laser_connection_var.set("Connection error")
            self.log_laser("Connection failed: TCP port must be an integer.")
        except IPGLaserError as exc:
            self.laser_connection_var.set("Connection error")
            self.log_laser(f"Connect failed: {exc}")

    def update_laser_connection_mode_ui(self):
        mode = self.laser_connection_mode_var.get()
        serial_state = tk.NORMAL if mode == "serial" else tk.DISABLED
        tcp_state = tk.NORMAL if mode == "tcp" else tk.DISABLED

        self.lbl_serial_port.configure(state=serial_state)
        self.ent_serial_port.configure(state=serial_state)
        self.lbl_ip.configure(state=tcp_state)
        self.ent_ip.configure(state=tcp_state)
        self.lbl_tcp_port.configure(state=tcp_state)
        self.ent_tcp_port.configure(state=tcp_state)

    def run_laser_command(self, action, success_message):
        try:
            result = action()
            if isinstance(result, str) and result:
                self.log_laser(f"{success_message}: {result}")
            else:
                self.log_laser(success_message)
            self.refresh_laser_status()
        except IPGLaserError as exc:
            self.log_laser(f"Command failed: {exc}")

    def refresh_laser_status(self):
        if self._laser_status_job is not None:
            self.root.after_cancel(self._laser_status_job)
            self._laser_status_job = None

        if self.laser.is_connected():
            try:
                snapshot = self.laser.get_snapshot()
                output = snapshot["output_power"]
                if isinstance(output, float):
                    output_text = f"{output:.2f} W"
                else:
                    output_text = output

                flags = snapshot["flags"]
                self.laser_state_var.set(snapshot["state"])
                self.laser_emission_var.set("On" if flags["emission_on"] else "Off")
                self.laser_output_var.set(output_text)
                self.laser_temp_var.set(f"{snapshot['temperature_c']:.1f} C")
                self.laser_status_word_var.set(f"{snapshot['status_word']} ({snapshot['status_hex']})")
            except IPGLaserError as exc:
                self.log_laser(f"Status read failed: {exc}")
        else:
            self.laser_state_var.set("Disconnected")
            self.laser_emission_var.set("-")
            self.laser_output_var.set("-")
            self.laser_temp_var.set("-")
            self.laser_status_word_var.set("-")

        self._laser_status_job = self.root.after(1000, self.refresh_laser_status)

    def start_emission(self):
        self.run_laser_command(self.laser.start_emission, "Emission start command sent")

    def stop_emission(self):
        self.run_laser_command(self.laser.stop_emission, "Emission stop command sent")

    def set_laser_power(self):
        try:
            setpoint = float(self.laser_setpoint_var.get())
        except ValueError:
            self.log_laser("Invalid setpoint. Enter a numeric current setpoint in percent.")
            return

        def action():
            return self.laser.set_diode_current(setpoint)

        self.run_laser_command(action, "Setpoint updated (%)")

    def read_laser_setpoint(self):
        def action():
            value = self.laser.read_current_setpoint()
            self.laser_setpoint_var.set(f"{value:.2f}")
            return f"{value:.2f}%"

        self.run_laser_command(action, "Current setpoint")

    def set_filter_window(self):
        try:
            seconds = float(self.laser_filter_window_var.get())
        except ValueError:
            self.log_laser("Invalid filter window. Enter a number in seconds.")
            return

        def action():
            applied = self.laser.set_filter_window(seconds)
            self.laser_filter_window_var.set(f"{applied:.2f}")
            return f"{applied:.2f}s"

        self.run_laser_command(action, "Filter window set")

    def read_filter_window(self):
        def action():
            value = self.laser.read_filter_window()
            self.laser_filter_window_var.set(f"{value:.2f}")
            return f"{value:.2f}s"

        self.run_laser_command(action, "Filter window")

    def enable_modulation(self):
        self.run_laser_command(self.laser.enable_modulation, "Modulation enabled")

    def disable_modulation(self):
        self.run_laser_command(self.laser.disable_modulation, "Modulation disabled")

    def enable_external_control(self):
        self.run_laser_command(self.laser.enable_external_control, "External analog control enabled")

    def disable_external_control(self):
        self.run_laser_command(self.laser.disable_external_control, "External analog control disabled")

    def aiming_beam_on(self):
        self.run_laser_command(self.laser.aiming_beam_on, "Aiming beam enabled")

    def aiming_beam_off(self):
        self.run_laser_command(self.laser.aiming_beam_off, "Aiming beam disabled")

    def reset_laser_errors(self):
        self.run_laser_command(self.laser.reset_errors, "Reset errors command sent")

    def read_firmware(self):
        def action():
            return self.laser.read_firmware_version()

        self.run_laser_command(action, "Firmware")

    def read_diode_current(self):
        def action():
            return f"{self.laser.read_diode_current():.2f} A"

        self.run_laser_command(action, "Diode current")

    def read_min_setpoint(self):
        def action():
            value = self.laser.read_min_current_setpoint()
            return f"{value:.2f}%"

        self.run_laser_command(action, "Minimum setpoint")

    def on_close(self):
        if self._laser_status_job is not None:
            self.root.after_cancel(self._laser_status_job)
            self._laser_status_job = None
        self.laser.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = AxisController(root)
    root.mainloop()
