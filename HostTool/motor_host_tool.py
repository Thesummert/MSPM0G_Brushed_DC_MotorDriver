#!/usr/bin/env python3
"""Simple motor driver host tool.

This tool talks to the firmware UART protocol implemented in
User/Protocol/Src/motor_uart_control.c.

Frame format:
    master_id(2) + cmd(1) + slave_id(2) + data_len(2) + data + crc16(2)

All multi-byte fields are big-endian, while CRC16 is appended as
low byte first, then high byte, matching the firmware.
"""

from __future__ import annotations

import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - runtime dependency message
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


MOTOR_CMD_SET = 0x01
MOTOR_CMD_DRIVE = 0x02
MOTOR_CMD_TRANSMIT = 0x03
MOTOR_CMD_FEEDBACK = 0x04
MOTOR_CMD_ACK = 0x10
MOTOR_CMD_FAIL = 0x11

CMD_NAMES = {
    MOTOR_CMD_SET: "SET",
    MOTOR_CMD_DRIVE: "DRIVE",
    MOTOR_CMD_TRANSMIT: "TRANSMIT",
    MOTOR_CMD_FEEDBACK: "FEEDBACK",
    MOTOR_CMD_ACK: "ACK",
    MOTOR_CMD_FAIL: "FAIL",
}

CRC16_INIT = 0xFFFF

CRC16_TABLE = (
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
)


def crc16_firmware(data: bytes) -> int:
    """Match the firmware CRC16 implementation."""

    crc = CRC16_INIT
    for byte in data:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]) & 0xFFFF
    return crc


def pack_frame(master_id: int, slave_id: int, cmd: int, payload: bytes = b"") -> bytes:
    header = struct.pack(
        ">HBHH",
        master_id & 0xFFFF,
        cmd & 0xFF,
        slave_id & 0xFFFF,
        len(payload) & 0xFFFF,
    )
    frame_wo_crc = header + payload
    crc = crc16_firmware(frame_wo_crc)
    return frame_wo_crc + struct.pack("<H", crc)


def build_storage_payload(values: dict[str, str]) -> bytes:
    """Pack the EEPROM storage layout used by the firmware."""

    body = struct.pack(
        "<HHffffffffBIB",
        int(values["slave_id"], 0),
        int(values["master_id"], 0),
        float(values["kp"]),
        float(values["ki"]),
        float(values["kd"]),
        float(values["maxout"]),
        float(values["integral_limit"]),
        float(values["deadband"]),
        float(values["lpf"]),
        float(values["radio"]),
        int(values["multiplier"], 0) & 0xFF,
        int(values["ppr"], 0) & 0xFFFFFFFF,
        int(values["freq"], 0) & 0xFF,
    )
    crc = crc16_firmware(body)
    return body + struct.pack("<H", crc)


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


class MotorHostTool(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Motor Host Tool")
        self.geometry("900x680")
        self.minsize(820, 600)

        self.ser = None
        self.rx_thread = None
        self.rx_running = False

        self._build_ui()
        self.refresh_ports()

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)

        conn = ttk.LabelFrame(root, text="Serial", padding=10)
        conn.pack(fill="x")

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.status_var = tk.StringVar(value="Disconnected")

        ttk.Label(conn, text="Port").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(conn, textvariable=self.port_var, width=24, state="readonly")
        self.port_box.grid(row=0, column=1, padx=6, pady=4, sticky="w")
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=6)

        ttk.Label(conn, text="Baud").grid(row=0, column=3, sticky="w")
        ttk.Entry(conn, textvariable=self.baud_var, width=12).grid(row=0, column=4, padx=6, sticky="w")
        ttk.Button(conn, text="Connect", command=self.toggle_connection).grid(row=0, column=5, padx=6)
        ttk.Label(conn, textvariable=self.status_var).grid(row=0, column=6, padx=10, sticky="w")

        params = ttk.LabelFrame(root, text="Motor Parameters", padding=10)
        params.pack(fill="x", pady=10)

        self.param_vars: dict[str, tk.StringVar] = {
            "master_id": tk.StringVar(value="0x200"),
            "slave_id": tk.StringVar(value="0x100"),
            "kp": tk.StringVar(value="0.0"),
            "ki": tk.StringVar(value="0.0"),
            "kd": tk.StringVar(value="0.0"),
            "maxout": tk.StringVar(value="0.0"),
            "integral_limit": tk.StringVar(value="0.0"),
            "deadband": tk.StringVar(value="0.0"),
            "lpf": tk.StringVar(value="0.0"),
            "radio": tk.StringVar(value="30.0"),
            "multiplier": tk.StringVar(value="4"),
            "ppr": tk.StringVar(value="100"),
            "freq": tk.StringVar(value="1"),
        }

        fields = [
            ("master_id", "Master ID"),
            ("slave_id", "Slave ID"),
            ("kp", "Kp"),
            ("ki", "Ki"),
            ("kd", "Kd"),
            ("maxout", "MaxOut"),
            ("integral_limit", "IntegralLimit"),
            ("deadband", "DeadBand"),
            ("lpf", "LPF"),
            ("radio", "Radio"),
            ("multiplier", "Multiplier"),
            ("ppr", "PPR"),
            ("freq", "Freq"),
        ]

        for idx, (key, label) in enumerate(fields):
            row, col = divmod(idx, 3)
            base = row * 2
            ttk.Label(params, text=label).grid(row=base, column=col * 2, sticky="w", padx=4, pady=3)
            ttk.Entry(params, textvariable=self.param_vars[key], width=18).grid(
                row=base, column=col * 2 + 1, sticky="w", padx=4, pady=3
            )

        buttons = ttk.Frame(root)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="Send SET", command=self.send_set_mode).pack(side="left")
        ttk.Button(buttons, text="Send Parameters", command=self.send_parameters).pack(side="left", padx=8)
        ttk.Button(buttons, text="Send Set + Params", command=self.send_set_and_parameters).pack(side="left")

        log_frame = ttk.LabelFrame(root, text="Log", padding=10)
        log_frame.pack(fill="both", expand=True, pady=10)
        self.log_text = tk.Text(log_frame, wrap="none", height=18)
        self.log_text.pack(fill="both", expand=True)
        self.log_text.configure(state="disabled")

    def refresh_ports(self) -> None:
        if list_ports is None:
            self.port_box["values"] = []
            self._log(f"pyserial is required: {SERIAL_IMPORT_ERROR}")
            return

        ports = [f"{p.device} - {p.description}" for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self) -> None:
        if self.ser and self.ser.is_open:
            self._close_serial()
            return

        if serial is None:
            messagebox.showerror("Missing dependency", f"pyserial is required:\n{SERIAL_IMPORT_ERROR}")
            return

        port = self.port_var.get().split(" - ", 1)[0].strip()
        if not port:
            messagebox.showwarning("No port", "Select a serial port first.")
            return

        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showwarning("Invalid baud", "Baud rate must be an integer.")
            return

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
        except Exception as exc:
            messagebox.showerror("Open failed", str(exc))
            return

        self.rx_running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.status_var.set(f"Connected: {port} @ {baud}")
        self._log(f"Opened {port} @ {baud}")

    def _close_serial(self) -> None:
        self.rx_running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.status_var.set("Disconnected")
        self._log("Closed serial port")

    def _rx_loop(self) -> None:
        while self.rx_running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    self.after(0, self._log, f"RX {hex_bytes(data)}")
            except Exception as exc:
                self.after(0, self._log, f"RX error: {exc}")
                break
            time.sleep(0.02)

    def _log(self, text: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{time.strftime('%H:%M:%S')}  {text}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _write_frame(self, frame: bytes, label: str) -> None:
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Not connected", "Open the serial port first.")
            return
        self.ser.write(frame)
        self.ser.flush()
        self._log(f"TX {label}: {hex_bytes(frame)}")

    def _parse_params(self) -> dict[str, str]:
        values = {key: var.get().strip() for key, var in self.param_vars.items()}
        int(values["master_id"], 0)
        int(values["slave_id"], 0)
        int(values["multiplier"], 0)
        int(values["ppr"], 0)
        int(values["freq"], 0)
        for key in ["kp", "ki", "kd", "maxout", "integral_limit", "deadband", "lpf", "radio"]:
            float(values[key])
        return values

    def send_set_mode(self) -> None:
        try:
            values = self._parse_params()
            frame = pack_frame(int(values["master_id"], 0), int(values["slave_id"], 0), MOTOR_CMD_SET)
        except ValueError as exc:
            messagebox.showerror("Invalid input", str(exc))
            return
        self._write_frame(frame, "SET")

    def send_parameters(self) -> None:
        try:
            values = self._parse_params()
            payload = build_storage_payload(values)
            frame = pack_frame(int(values["master_id"], 0), int(values["slave_id"], 0), MOTOR_CMD_TRANSMIT, payload)
        except ValueError as exc:
            messagebox.showerror("Invalid input", str(exc))
            return
        self._write_frame(frame, "TRANSMIT")

    def send_set_and_parameters(self) -> None:
        try:
            values = self._parse_params()
            master_id = int(values["master_id"], 0)
            slave_id = int(values["slave_id"], 0)
            set_frame = pack_frame(master_id, slave_id, MOTOR_CMD_SET)
            payload = build_storage_payload(values)
            transmit_frame = pack_frame(master_id, slave_id, MOTOR_CMD_TRANSMIT, payload)
        except ValueError as exc:
            messagebox.showerror("Invalid input", str(exc))
            return

        self._write_frame(set_frame, "SET")
        time.sleep(0.05)
        self._write_frame(transmit_frame, "TRANSMIT")


def main() -> int:
    app = MotorHostTool()
    app.protocol("WM_DELETE_WINDOW", lambda: (app._close_serial(), app.destroy()))
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
