import tkinter as tk
from tkinter import ttk, messagebox

import math
import queue
import re
import threading
from collections import deque

from opentelemetry import metrics
import serial
import serial.tools.list_ports

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator, FuncFormatter


## @class PIDControllerGUI
#  @brief Tkinter-based GUI application for monitoring and controlling an STM32 PID controller.
#
#  This class creates a serial communication interface, parameter input fields,
#  real-time plots and response metric displays for a PID controller running on STM32.
class PIDControllerGUI:

    ## @brief Regular expression pattern used to parse key-value pairs from UART messages.
    #
    #  Supported examples:
    #  - SetPoint: 2.50
    #  - Voltage=2.48
    #  - PIDOutput: 75.2
    UART_VALUE_PATTERN = re.compile(
        r"([A-Za-z][A-Za-z0-9_ -]*)\s*[:=]\s*"
        r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)"
    )

    ## @brief Initializes the PID controller GUI.
    #
    #  This constructor initializes window settings, serial communication variables,
    #  plotting buffers, response metric variables and starts periodic GUI update tasks.
    #
    #  @param root Main Tkinter root window.
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 PID Controller")
        self.root.geometry("1050x800")
        self.root.resizable(False, False)

        ## @brief Serial port object used for UART communication.
        self.serial_port = None

        ## @brief Background thread used for reading UART data.
        self.serial_thread = None

        ## @brief Serial thread running state flag.
        self.running = False

        ## @brief Plot update enable flag.
        self.plot_enabled = False

        ## @brief Queue used to safely transfer UART data from serial thread to GUI thread.
        self.rx_queue = queue.Queue()

        ## @brief Maximum number of data points displayed on the graph.
        self.max_points = 200

        ## @brief Sample index data buffer.
        self.time_data = deque(maxlen=self.max_points)

        ## @brief Voltage data buffer.
        self.voltage_data = deque(maxlen=self.max_points)

        ## @brief Setpoint data buffer.
        self.setpoint_data = deque(maxlen=self.max_points)

        ## @brief PID output data buffer.
        self.pid_output_data = deque(maxlen=self.max_points)

        ## @brief Current sample counter.
        self.sample_index = 0

        ## @brief Latest setpoint value received from UART or entered by the user.
        self.current_setpoint = 0.0

        ## @brief Sampling period in seconds.
        #
        #  This value should match the STM32 UART transmission period.
        self.sample_time_s = 0.1  # 100 ms sample time

        ## @brief Tkinter string variable used to display rise time.
        self.rise_time_var = tk.StringVar(value="--")

        ## @brief Tkinter string variable used to display settling time.
        self.settling_time_var = tk.StringVar(value="--")

        ## @brief Tkinter string variable used to display overshoot.
        self.overshoot_var = tk.StringVar(value="--")

        ## @brief Tkinter string variable used to display steady-state error.
        self.steady_state_error_var = tk.StringVar(value="--")

        ## @brief Stored rise time value after it is calculated.
        self.latched_rise_time = None

        ## @brief Stored settling time value after it is calculated.
        self.latched_settling_time = None

        ## @brief Stored overshoot percentage value.
        self.latched_overshoot_percent = None

        ## @brief Stored overshoot voltage value.
        self.latched_overshoot_value = None

        ## @brief Latest calculated steady-state error value.
        self.latest_steady_state_error = None

        self.create_widgets()
        self.refresh_ports()
        self.update_plot()
        self.process_serial_queue()

    ## @brief Creates and places all GUI widgets.
    #
    #  This function creates connection controls, PID parameter inputs,
    #  response metric labels and Matplotlib graph components.
    def create_widgets(self):
        main_frame = tk.Frame(self.root, bg="#d7e8f6")
        main_frame.pack(fill=tk.BOTH, expand=True)

        left_frame = tk.Frame(main_frame, bg="#d7e8f6", width=310)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=15, pady=15)

        right_frame = tk.Frame(main_frame, bg="#d7e8f6")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=15, pady=15)

        connection_frame = tk.LabelFrame(
            left_frame,
            text="Connection",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        connection_frame.pack(fill=tk.X, pady=5)

        tk.Label(connection_frame, text="COM Port:", bg="#d7e8f6").grid(
            row=0, column=0, sticky="w"
        )

        self.port_combo = ttk.Combobox(connection_frame, width=16, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=4)

        self.refresh_button = tk.Button(
            connection_frame,
            text="⟳",
            width=3,
            command=self.refresh_ports
        )
        self.refresh_button.grid(row=0, column=2, padx=5)

        tk.Label(connection_frame, text="Baudrate:", bg="#d7e8f6").grid(
            row=1, column=0, sticky="w"
        )

        self.baud_combo = ttk.Combobox(
            connection_frame,
            values=[
                "9600",
                "19200",
                "38400",
                "57600",
                "115200",
                "230400",
                "460800",
                "921600"
            ],
            width=16,
            state="readonly"
        )
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=1, column=1, padx=5, pady=4)

        tk.Label(connection_frame, text="StopBits:", bg="#d7e8f6").grid(
            row=2, column=0, sticky="w"
        )

        self.stopbits_combo = ttk.Combobox(
            connection_frame,
            values=["One", "Two"],
            width=16,
            state="readonly"
        )
        self.stopbits_combo.set("One")
        self.stopbits_combo.grid(row=2, column=1, padx=5, pady=4)

        tk.Label(connection_frame, text="Parity:", bg="#d7e8f6").grid(
            row=3, column=0, sticky="w"
        )

        self.parity_combo = ttk.Combobox(
            connection_frame,
            values=["None", "Even", "Odd"],
            width=16,
            state="readonly"
        )
        self.parity_combo.set("None")
        self.parity_combo.grid(row=3, column=1, padx=5, pady=4)

        self.connect_button = tk.Button(
            connection_frame,
            text="Connect",
            width=12,
            command=self.connect_serial
        )
        self.connect_button.grid(row=4, column=0, pady=10)

        self.disconnect_button = tk.Button(
            connection_frame,
            text="Disconnect",
            width=12,
            command=self.disconnect_serial
        )
        self.disconnect_button.grid(row=5, column=0, pady=4)

        self.status_label = tk.Label(
            connection_frame,
            text="DISCONNECTED",
            bg="#f2c6c6",
            fg="black",
            width=18
        )
        self.status_label.grid(row=4, column=1, padx=5)

        parameter_frame = tk.LabelFrame(
            left_frame,
            text="Parameters",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        parameter_frame.pack(fill=tk.X, pady=15)

        self.setpoint_entry = self.create_parameter_row(
            parameter_frame,
            "Set Point:",
            0,
            self.send_setpoint
        )

        self.kp_entry = self.create_parameter_row(
            parameter_frame,
            "Kp:",
            1,
            self.send_kp
        )

        self.ki_entry = self.create_parameter_row(
            parameter_frame,
            "Ki:",
            2,
            self.send_ki
        )

        self.kd_entry = self.create_parameter_row(
            parameter_frame,
            "Kd:",
            3,
            self.send_kd
        )

        self.setpoint_entry.insert(0, "3.00")
        self.kp_entry.insert(0, "1.0")
        self.ki_entry.insert(0, "0.0")
        self.kd_entry.insert(0, "0.0")

        control_frame = tk.Frame(left_frame, bg="#d7e8f6")
        control_frame.pack(fill=tk.X, pady=20)

        metrics_frame = tk.LabelFrame(
            left_frame,
            text="Response Metrics",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        metrics_frame.pack(fill=tk.X, pady=10)

        tk.Label(metrics_frame, text="Rise Time:", bg="#d7e8f6").grid(
            row=0, column=0, sticky="w", pady=4
        )
        tk.Label(metrics_frame, textvariable=self.rise_time_var, bg="#d7e8f6").grid(
            row=0, column=1, sticky="w", padx=8
        )

        tk.Label(metrics_frame, text="Settling Time:", bg="#d7e8f6").grid(
            row=1, column=0, sticky="w", pady=4
        )
        tk.Label(metrics_frame, textvariable=self.settling_time_var, bg="#d7e8f6").grid(
            row=1, column=1, sticky="w", padx=8
        )

        tk.Label(metrics_frame, text="Overshoot:", bg="#d7e8f6").grid(
            row=2, column=0, sticky="w", pady=4
        )
        tk.Label(metrics_frame, textvariable=self.overshoot_var, bg="#d7e8f6").grid(
            row=2, column=1, sticky="w", padx=8
        )

        tk.Label(metrics_frame, text="Steady State Error:", bg="#d7e8f6").grid(
            row=3, column=0, sticky="w", pady=4
        )
        tk.Label(metrics_frame, textvariable=self.steady_state_error_var, bg="#d7e8f6").grid(
            row=3, column=1, sticky="w", padx=8
        )

        self.start_button = tk.Button(
            control_frame,
            text="START",
            width=12,
            command=self.start_pid
        )
        self.start_button.grid(row=0, column=0, padx=8)

        self.stop_button = tk.Button(
            control_frame,
            text="STOP",
            width=12,
            command=self.stop_pid
        )
        self.stop_button.grid(row=0, column=1, padx=8)

        self.clear_button = tk.Button(
            control_frame,
            text="CLEAR GRAPH",
            width=26,
            command=self.clear_graph
        )
        self.clear_button.grid(row=1, column=0, columnspan=2, pady=12)

        graph_frame = tk.LabelFrame(
            right_frame,
            text="Graph",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        graph_frame.pack(fill=tk.BOTH, expand=True)

        self.figure = Figure(figsize=(7.8, 4.8), dpi=100)
        self.ax = self.figure.add_subplot(111)

        self.ax.set_title("PID Response")
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Voltage / Set Point (V)")
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 3.5)
        self.ax.grid(True)

        self.ax.yaxis.set_major_locator(MaxNLocator(nbins=8))
        self.ax.yaxis.set_major_formatter(FuncFormatter(self.format_y_axis))

        self.ax_pid = self.ax.twinx()
        self.ax_pid.set_ylabel("PID Output (%)")
        self.ax_pid.set_ylim(0, 100)

        self.voltage_line, = self.ax.plot(
            [], [],
            label="Voltage",
            color="#1f77b4",
            linewidth=1.8
        )

        self.setpoint_line, = self.ax.plot(
            [], [],
            label="Set Point",
            color="#ff7f0e",
            linewidth=1.8
        )

        self.pid_output_line, = self.ax_pid.plot(
            [], [],
            label="PID Output",
            color="#2ca02c",
            linewidth=1.8
        )

        lines1, labels1 = self.ax.get_legend_handles_labels()
        lines2, labels2 = self.ax_pid.get_legend_handles_labels()

        self.ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

        self.canvas = FigureCanvasTkAgg(self.figure, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    ## @brief Dynamically sets the y-axis limits based on given values.
    #
    #  This function adjusts the selected axis limits by using the minimum and
    #  maximum values in the provided data list. A minimum span is applied to
    #  prevent excessive zooming when the data range is very small.
    #
    #  @param axis Matplotlib axis object whose y-limits will be updated.
    #  @param values Data values used for calculating the axis limits.
    #  @param min_span Minimum allowed y-axis span.
    #  @param margin_ratio Ratio used to add margin around the data range.
    def set_dynamic_ylim(self, axis, values, min_span, margin_ratio=0.15):
        y_min = min(values)
        y_max = max(values)

        center = (y_min + y_max) / 2.0
        span = y_max - y_min

        if span < min_span:
            y_min = center - (min_span / 2.0)
            y_max = center + (min_span / 2.0)
        else:
            margin = span * margin_ratio
            y_min -= margin
            y_max += margin

        axis.set_ylim(y_min, y_max)

    ## @brief Creates a PID parameter input row.
    #
    #  This function creates a label, entry field and SET button for a single
    #  PID parameter.
    #
    #  @param parent Parent Tkinter widget.
    #  @param label_text Text displayed next to the entry field.
    #  @param row Grid row index.
    #  @param command Callback function executed when the SET button is pressed.
    #  @return Created Tkinter Entry widget.
    def create_parameter_row(self, parent, label_text, row, command):
        tk.Label(parent, text=label_text, bg="#d7e8f6").grid(
            row=row,
            column=0,
            sticky="w",
            pady=7
        )

        entry = tk.Entry(parent, width=12)
        entry.grid(row=row, column=1, padx=8, pady=7)

        button = tk.Button(parent, text="SET", width=8, command=command)
        button.grid(row=row, column=2, padx=5, pady=7)

        return entry

    ## @brief Refreshes the available serial COM ports.
    #
    #  This function scans connected serial ports and updates the COM port
    #  selection combobox.
    def refresh_ports(self):
        previous_selection = self.port_combo.get()

        ports = serial.tools.list_ports.comports()
        port_names = [port.device for port in ports]

        self.port_combo["values"] = port_names

        if previous_selection in port_names:
            self.port_combo.set(previous_selection)
        elif port_names:
            self.port_combo.set(port_names[0])
        else:
            self.port_combo.set("")

    ## @brief Opens the selected serial port connection.
    #
    #  This function reads serial settings from the GUI, opens the selected
    #  COM port and starts the background serial reading thread.
    def connect_serial(self):
        if self.serial_port and self.serial_port.is_open:
            return

        port = self.port_combo.get()

        if not port:
            messagebox.showerror("Error", "No COM port selected.")
            return

        try:
            baudrate = int(self.baud_combo.get())

            stopbits = serial.STOPBITS_ONE
            if self.stopbits_combo.get() == "Two":
                stopbits = serial.STOPBITS_TWO

            parity = serial.PARITY_NONE
            if self.parity_combo.get() == "Even":
                parity = serial.PARITY_EVEN
            elif self.parity_combo.get() == "Odd":
                parity = serial.PARITY_ODD

            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=parity,
                stopbits=stopbits,
                timeout=0.1
            )

            self.running = True
            self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.serial_thread.start()

            self.status_label.config(text="CONNECTED", bg="#bfe6bf")

        except (serial.SerialException, ValueError) as e:
            self.serial_port = None
            self.status_label.config(text="FAILED", bg="#f2c6c6")
            messagebox.showerror("Serial Error", str(e))

    ## @brief Closes the active serial port connection.
    #
    #  This function stops serial reading, closes the serial port and updates
    #  the GUI connection status.
    def disconnect_serial(self):
        self.running = False
        self.plot_enabled = False

        if self.serial_port:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except serial.SerialException:
                pass

        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=0.3)

        self.serial_thread = None
        self.serial_port = None

        self.status_label.config(text="DISCONNECTED", bg="#f2c6c6")

    ## @brief Reads UART data in a background thread.
    #
    #  This function continuously reads lines from the serial port and places
    #  valid received lines into the receive queue.
    def read_serial(self):
        while self.running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    line = self.serial_port.readline().decode(
                        "utf-8",
                        errors="ignore"
                    ).strip()

                    if line:
                        self.rx_queue.put(line)

            except serial.SerialException:
                if self.running:
                    self.rx_queue.put("SERIAL_ERROR")
                break

    ## @brief Processes received UART lines from the queue.
    #
    #  This function runs periodically in the Tkinter main thread. It parses
    #  received UART messages, updates graph data buffers and refreshes response
    #  metrics when plotting is enabled.
    def process_serial_queue(self):
        while not self.rx_queue.empty():
            line = self.rx_queue.get()

            if line == "SERIAL_ERROR":
                self.disconnect_serial()
                messagebox.showerror("Serial Error", "Serial connection lost.")
                continue

            parsed = self.parse_uart_data(line)

            if parsed is not None and self.plot_enabled:
                voltage, setpoint, pid_output = parsed

                self.sample_index += 1

                self.time_data.append(self.sample_index)
                self.voltage_data.append(voltage)
                self.setpoint_data.append(setpoint)

                if pid_output is None:
                    self.pid_output_data.append(math.nan)
                else:
                    self.pid_output_data.append(pid_output)

                self.update_response_metrics()
        self.root.after(20, self.process_serial_queue)

    ## @brief Parses a UART data line received from STM32.
    #
    #  Supported UART receive formats:
    #
    #  SetPoint:3.00 Voltage:2.85 PIDOutput:1.50
    #  SetPoint: 3.00 Voltage: 2.85
    #  Setpoint:3.00, Voltage:2.85
    #  SETPOINT=3.00 VOLTAGE=2.85
    #
    #  @param line UART line string received from STM32.
    #  @return Tuple containing voltage, setpoint and PID output values.
    #  @retval None Returned if the UART line cannot be parsed.
    def parse_uart_data(self, line):
        """
        Supported UART receive formats:

        SetPoint:3.00 Voltage:2.85 PIDOutput:1.50
        SetPoint: 3.00 Voltage: 2.85
        Setpoint:3.00, Voltage:2.85
        SETPOINT=3.00 VOLTAGE=2.85
        """

        matches = self.UART_VALUE_PATTERN.findall(line)

        if not matches:
            print("Invalid UART data:", line)
            return None

        data = {}

        for key, value in matches:
            normalized_key = self.normalize_key(key)

            try:
                data[normalized_key] = float(value)
            except ValueError:
                print("Invalid numeric value:", line)
                return None

        setpoint = data.get("setpoint")
        voltage = data.get("voltage")
        pid_output = data.get("pidoutput")

        if setpoint is None or voltage is None:
            print("Missing SetPoint or Voltage:", line)
            return None

        self.current_setpoint = setpoint

        return voltage, setpoint, pid_output

    ## @brief Normalizes UART data keys.
    #
    #  This function converts a key string to lowercase and removes spaces,
    #  underscores and hyphens to simplify key matching.
    #
    #  @param key Raw key string received from UART data.
    #  @return Normalized key string.
    @staticmethod
    def normalize_key(key):
        return (
            key.strip()
            .lower()
            .replace(" ", "")
            .replace("_", "")
            .replace("-", "")
        )

    ## @brief Formats y-axis tick labels.
    #
    #  This function formats numeric axis values according to their magnitude
    #  to keep graph labels readable.
    #
    #  @param value Axis tick value.
    #  @param _ Unused formatter position argument.
    #  @return Formatted string representation of the axis value.
    @staticmethod
    def format_y_axis(value, _):
        abs_value = abs(value)

        if abs_value == 0:
            return "0"

        if abs_value < 0.01:
            return f"{value:.4f}".rstrip("0").rstrip(".")

        if abs_value < 1:
            return f"{value:.3f}".rstrip("0").rstrip(".")

        if abs_value < 10:
            return f"{value:.2f}".rstrip("0").rstrip(".")

        if abs_value < 100:
            return f"{value:.1f}".rstrip("0").rstrip(".")

        return f"{value:.0f}"

    ## @brief Resets all stored response metric values.
    #
    #  This function clears latched rise time, settling time, overshoot and
    #  steady-state error values and resets their GUI labels.
    def reset_metrics(self):

        self.latched_rise_time = None
        self.latched_settling_time = None
        self.latched_overshoot_percent = None
        self.latched_overshoot_value = None
        self.latest_steady_state_error = None

        self.rise_time_var.set("--")
        self.settling_time_var.set("--")
        self.overshoot_var.set("--")
        self.steady_state_error_var.set("--")

    ## @brief Updates response metric labels on the GUI.
    #
    #  This function displays the latest stored response metric values.
    #  If a metric has not been calculated yet, "--" is shown.
    def refresh_metric_labels(self):
        if self.latched_rise_time is None:
            self.rise_time_var.set("--")
        else:
            self.rise_time_var.set(f"{self.latched_rise_time:.2f} s")

        if self.latched_settling_time is None:
            self.settling_time_var.set("--")
        else:
            self.settling_time_var.set(f"{self.latched_settling_time:.2f} s")

        if self.latched_overshoot_percent is None:
            self.overshoot_var.set("--")
        else:
            self.overshoot_var.set(
                f"{self.latched_overshoot_percent:.2f} % "
                f"({self.latched_overshoot_value:.3f} V)"
            )

        if self.latest_steady_state_error is None:
            self.steady_state_error_var.set("--")
        else:
            self.steady_state_error_var.set(
                f"{self.latest_steady_state_error:.4f} V"
            )

    ## @brief Updates calculated response metrics.
    #
    #  This function calculates response metrics using the current graph data.
    #  Rise time, settling time and overshoot are latched after valid calculation,
    #  while steady-state error is updated continuously.
    def update_response_metrics(self):
        metrics = self.calculate_response_metrics()

        if metrics is None:
            self.refresh_metric_labels()
            return

        rise_time = metrics["rise_time"]
        settling_time = metrics["settling_time"]
        overshoot_percent = metrics["overshoot_percent"]
        overshoot_value = metrics["overshoot_value"]
        steady_state_error = metrics["steady_state_error"]
        has_overshoot = metrics["has_overshoot"]

        if self.latched_rise_time is None and rise_time is not None:
            self.latched_rise_time = rise_time

        if self.latched_settling_time is None and settling_time is not None:
            self.latched_settling_time = settling_time

            self.latched_overshoot_percent = overshoot_percent
            self.latched_overshoot_value = overshoot_value

        if self.latched_settling_time is None and has_overshoot:
            self.latched_overshoot_percent = overshoot_percent
            self.latched_overshoot_value = overshoot_value

        self.latest_steady_state_error = steady_state_error

        self.refresh_metric_labels()

    ## @brief Calculates PID response metrics from graph data.
    #
    #  This function calculates rise time, settling time, overshoot and
    #  steady-state error using voltage and setpoint data stored in buffers.
    #
    #  @return Dictionary containing calculated response metrics.
    #  @retval None Returned when there is not enough data for calculation.
    def calculate_response_metrics(self):
        voltage_values = list(self.voltage_data)
        setpoint_values = list(self.setpoint_data)

        if len(voltage_values) < 5 or len(setpoint_values) < 5:
            return None

        initial_value = voltage_values[0]
        final_setpoint = setpoint_values[-1]

        step_amplitude = final_setpoint - initial_value
        abs_step = abs(step_amplitude)

        min_valid_step = max(0.02 * abs(final_setpoint), 0.01)

        rise_time = None
        overshoot_percent = None
        overshoot_value = 0.0
        has_overshoot = False

        if abs_step >= min_valid_step:
            direction = 1.0 if step_amplitude > 0.0 else -1.0

            y_10 = initial_value + 0.10 * step_amplitude
            y_90 = initial_value + 0.90 * step_amplitude

            index_10 = None
            index_90 = None

            for i, value in enumerate(voltage_values):
                if index_10 is None:
                    if direction * (value - y_10) >= 0.0:
                        index_10 = i

                if index_10 is not None:
                    if direction * (value - y_90) >= 0.0:
                        index_90 = i
                        break

            if index_10 is not None and index_90 is not None:
                rise_time = (index_90 - index_10) * self.sample_time_s

            if direction > 0.0:
                peak_value = max(voltage_values)
                overshoot_value = max(0.0, peak_value - final_setpoint)
            else:
                peak_value = min(voltage_values)
                overshoot_value = max(0.0, final_setpoint - peak_value)

            overshoot_percent = (overshoot_value / abs_step) * 100.0

            overshoot_threshold = max(0.005, 0.002 * abs(final_setpoint))
            has_overshoot = overshoot_value > overshoot_threshold

        settling_tolerance = max(0.02 * abs(final_setpoint), 0.01)

        settling_index = None
        settling_required_samples = 20

        if len(voltage_values) >= settling_required_samples:
            for i in range(len(voltage_values) - settling_required_samples + 1):
                remaining_values = voltage_values[i:]

                is_settled = all(
                    abs(value - final_setpoint) <= settling_tolerance
                    for value in remaining_values
                )

                if is_settled:
                    settling_index = i
                    break

        if settling_index is None:
            settling_time = None
        else:
            settling_time = settling_index * self.sample_time_s

        steady_state_window = min(20, len(voltage_values))
        steady_state_average = (
            sum(voltage_values[-steady_state_window:]) / steady_state_window
        )

        steady_state_error = abs(final_setpoint - steady_state_average)

        return {
            "rise_time": rise_time,
            "settling_time": settling_time,
            "overshoot_percent": overshoot_percent,
            "overshoot_value": overshoot_value,
            "has_overshoot": has_overshoot,
            "steady_state_error": steady_state_error
        }

    ## @brief Updates the Matplotlib graph.
    #
    #  This function refreshes voltage, setpoint and PID output data lines,
    #  dynamically updates axis limits and schedules the next plot update.
    def update_plot(self):
        self.voltage_line.set_data(self.time_data, self.voltage_data)
        self.setpoint_line.set_data(self.time_data, self.setpoint_data)
        self.pid_output_line.set_data(self.time_data, self.pid_output_data)

        if len(self.time_data) >= 1:
            x_min = self.time_data[0]
            x_max = self.time_data[-1]

            if x_min == x_max:
                self.ax.set_xlim(x_min - 1, x_max + 1)
            else:
                self.ax.set_xlim(x_min, x_max)

            self.ax_pid.set_xlim(self.ax.get_xlim())

            left_values = list(self.voltage_data) + list(self.setpoint_data)
            left_values = [
                value for value in left_values
                if math.isfinite(value)
            ]

            right_values = [
                value for value in self.pid_output_data
                if math.isfinite(value)
            ]

            if left_values:
                self.set_dynamic_ylim(self.ax, left_values, min_span=0.5)

            if right_values:
                self.set_dynamic_ylim(self.ax_pid, right_values, min_span=10.0)
            else:
                self.ax_pid.set_ylim(0, 100)

            self.ax.yaxis.set_major_locator(MaxNLocator(nbins=8))
            self.ax_pid.yaxis.set_major_locator(MaxNLocator(nbins=8))

        self.canvas.draw_idle()
        self.root.after(50, self.update_plot)

    ## @brief Sends a command string over UART.
    #
    #  This function checks whether the serial port is connected and transmits
    #  the provided command string to STM32.
    #
    #  @param command Command string to be sent over UART.
    #  @return True if transmission succeeds, otherwise False.
    def send_uart_command(self, command):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Warning", "Serial port is not connected.")
            return False

        try:
            self.serial_port.write(command.encode("utf-8"))
            self.serial_port.flush()
            return True

        except serial.SerialException as e:
            messagebox.showerror("Serial Error", str(e))
            return False

    ## @brief Sends the setpoint value to STM32.
    #
    #  This function reads the setpoint entry value, sends it through UART
    #  and clears the graph if the command is transmitted successfully.
    def send_setpoint(self):
        value = self.get_float_from_entry(self.setpoint_entry, "Set Point")

        if value is None:
            return

        if self.send_uart_command(f"SETPOINT:{value}\r\n"):
            self.current_setpoint = value
            self.clear_graph()

    ## @brief Sends the proportional gain value to STM32.
    #
    #  This function reads the Kp entry value, sends it through UART
    #  and clears the graph if the command is transmitted successfully.
    def send_kp(self):
        value = self.get_float_from_entry(self.kp_entry, "Kp")

        if value is None:
            return

        if self.send_uart_command(f"KP:{value}\r\n"):
            self.clear_graph()

    ## @brief Sends the integral gain value to STM32.
    #
    #  This function reads the Ki entry value, sends it through UART
    #  and clears the graph if the command is transmitted successfully.
    def send_ki(self):
        value = self.get_float_from_entry(self.ki_entry, "Ki")

        if value is None:
            return

        if self.send_uart_command(f"KI:{value}\r\n"):
            self.clear_graph()

    ## @brief Sends the derivative gain value to STM32.
    #
    #  This function reads the Kd entry value, sends it through UART
    #  and clears the graph if the command is transmitted successfully.
    def send_kd(self):
        value = self.get_float_from_entry(self.kd_entry, "Kd")

        if value is None:
            return

        if self.send_uart_command(f"KD:{value}\r\n"):
            self.clear_graph()

    ## @brief Reads and validates a floating-point value from an entry widget.
    #
    #  This function converts the text inside a Tkinter Entry widget to float.
    #  If conversion fails, an error message is displayed.
    #
    #  @param entry Tkinter Entry widget containing the value.
    #  @param name User-readable parameter name used in error messages.
    #  @return Converted floating-point value.
    #  @retval None Returned if the entry value is invalid.
    def get_float_from_entry(self, entry, name):
        try:
            return float(entry.get())

        except ValueError:
            messagebox.showerror("Invalid Value", f"{name} must be a number.")
            return None

    ## @brief Starts PID operation from the GUI.
    #
    #  This function sends all PID parameters to STM32, sends the START command,
    #  clears the graph and enables plotting.
    def start_pid(self):
        if not self.send_all_parameters():
            return

        if self.send_uart_command("START\n"):
            self.clear_graph()
            self.plot_enabled = True

    ## @brief Stops PID operation from the GUI.
    #
    #  This function disables plotting and sends the STOP command to STM32.
    def stop_pid(self):
        self.plot_enabled = False
        self.send_uart_command("STOP\n")

    ## @brief Clears all graph data and response metrics.
    #
    #  This function clears voltage, setpoint, PID output and sample data buffers,
    #  resets graph axes and clears response metric labels.
    def clear_graph(self):
        self.time_data.clear()
        self.voltage_data.clear()
        self.setpoint_data.clear()
        self.pid_output_data.clear()

        self.sample_index = 0

        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 5)
        self.ax_pid.set_ylim(0, 100)

        self.reset_metrics()
        self.canvas.draw_idle()

    ## @brief Handles GUI close event.
    #
    #  This function disconnects the serial port and destroys the Tkinter root window.
    def on_close(self):
        self.disconnect_serial()
        self.root.destroy()

    ## @brief Sends all PID parameters to STM32.
    #
    #  This function reads setpoint, Kp, Ki and Kd values from the GUI,
    #  validates them and sends all parameter commands over UART.
    #
    #  @return True if all parameters are sent successfully, otherwise False.
    def send_all_parameters(self):
        setpoint = self.get_float_from_entry(self.setpoint_entry, "Set Point")
        kp = self.get_float_from_entry(self.kp_entry, "Kp")
        ki = self.get_float_from_entry(self.ki_entry, "Ki")
        kd = self.get_float_from_entry(self.kd_entry, "Kd")

        if setpoint is None or kp is None or ki is None or kd is None:
            return False

        commands = [
            f"SETPOINT:{setpoint}\r\n",
            f"KP:{kp}\r\n",
            f"KI:{ki}\r\n",
            f"KD:{kd}\r\n"
        ]

        for command in commands:
            if not self.send_uart_command(command):
                return False

        self.current_setpoint = setpoint
        return True


## @brief Application entry point.
#
#  This block creates the Tkinter root window, initializes the PID controller GUI
#  and starts the Tkinter event loop.
if __name__ == "__main__":
    root = tk.Tk()
    app = PIDControllerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()