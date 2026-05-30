#!/usr/bin/env python3
"""
flash_tool_gui.py  –  CSDR NOR Flash Tool  (tkinter GUI)

Dependencies (same as flash_tool.py):
  pip install pyserial pillow
"""

import os
import sys
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# Allow importing flash_tool from the same directory
sys.path.insert(0, os.path.dirname(__file__))
from flash_tool import (
    FlashTool, FlashProtoError,
    MAX_PAGE, SECTOR_SIZE, BLOCK64_SIZE,
    FLASH_ADDR_LOGO,
    CMD_READ, CMD_WRITE, CMD_SECTOR_ERASE, CMD_CHIP_ID, CMD_BLOCK64_ERASE,
    STATUS_OK,
)

try:
    import serial.tools.list_ports as list_ports
    import serial
except ImportError:
    messagebox.showerror("Missing dependency", "Run: pip install pyserial")
    sys.exit(1)

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ---------------------------------------------------------------------------
# GUIFlashTool — adds progress + log callbacks to FlashTool
# ---------------------------------------------------------------------------
class GUIFlashTool(FlashTool):
    def __init__(self, port: str, progress_cb=None, log_cb=None):
        super().__init__(port)
        self._prog = progress_cb or (lambda done, total: None)
        self._log  = log_cb  or (lambda msg: None)

    def erase_range(self, addr: int, length: int, verbose: bool = True):
        start = (addr // SECTOR_SIZE) * SECTOR_SIZE
        end   = ((addr + length - 1) // SECTOR_SIZE + 1) * SECTOR_SIZE
        pos   = start
        total = end - start
        done  = 0
        while pos < end:
            if pos % BLOCK64_SIZE == 0 and (end - pos) >= BLOCK64_SIZE:
                self._log(f"  Erase 64K block @ 0x{pos:06X}")
                self.block64_erase(pos)
                done += BLOCK64_SIZE
                pos  += BLOCK64_SIZE
            else:
                self._log(f"  Erase 4K sector @ 0x{pos:06X}")
                self.sector_erase(pos)
                done += SECTOR_SIZE
                pos  += SECTOR_SIZE
            self._prog(done, total)
        self._log(f"  Erase done ({total} bytes)")

    def write_binary(self, addr: int, data: bytes, verbose: bool = True):
        total  = len(data)
        offset = 0
        while offset < total:
            page_off = (addr + offset) % MAX_PAGE
            chunk    = min(MAX_PAGE - page_off, total - offset)
            self.write_page(addr + offset, data[offset:offset + chunk])
            offset += chunk
            self._prog(offset, total)
        self._log(f"  Write done ({total} bytes)")


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CSDR Flash Tool")
        self.resizable(True, True)
        self.minsize(720, 580)

        self._tool: GUIFlashTool | None = None
        self._busy  = False
        self._queue: queue.Queue = queue.Queue()

        self._build_ui()
        self._refresh_ports()
        self._poll_queue()

    # -----------------------------------------------------------------------
    # UI construction
    # -----------------------------------------------------------------------
    def _build_ui(self):
        # ── Top bar: port + chip-id ────────────────────────────────────────
        bar = ttk.Frame(self, padding=6)
        bar.pack(fill="x")

        ttk.Label(bar, text="Port:").pack(side="left")
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(bar, textvariable=self._port_var,
                                       width=14, state="readonly")
        self._port_cb.pack(side="left", padx=4)

        ttk.Button(bar, text="⟳ Refresh",
                   command=self._refresh_ports).pack(side="left")
        ttk.Button(bar, text="Chip ID",
                   command=self._do_chip_id).pack(side="left", padx=8)

        self._id_lbl = ttk.Label(bar, text="", foreground="#0077cc",
                                  font=("Consolas", 9))
        self._id_lbl.pack(side="left")

        ttk.Separator(self, orient="horizontal").pack(fill="x")

        # ── Notebook ────────────────────────────────────────────────────────
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=6, pady=4)

        tab_flash = ttk.Frame(nb, padding=6)
        tab_logo  = ttk.Frame(nb, padding=6)
        nb.add(tab_flash, text="  Flash Operations  ")
        nb.add(tab_logo,  text="  Logo  ")

        self._build_flash_tab(tab_flash)
        self._build_logo_tab(tab_logo)

        # ── Progress bar ────────────────────────────────────────────────────
        prog_frame = ttk.Frame(self, padding=(6, 2, 6, 2))
        prog_frame.pack(fill="x")

        self._prog_var = tk.DoubleVar(value=0)
        self._prog_bar = ttk.Progressbar(prog_frame, variable=self._prog_var,
                                          maximum=100, length=300)
        self._prog_bar.pack(side="left", fill="x", expand=True)
        self._prog_lbl = ttk.Label(prog_frame, text="", width=8, anchor="e",
                                    font=("Consolas", 9))
        self._prog_lbl.pack(side="left", padx=4)

        # ── Log ─────────────────────────────────────────────────────────────
        log_frame = ttk.LabelFrame(self, text="Log", padding=4)
        log_frame.pack(fill="both", expand=False, padx=6, pady=(0, 6))

        self._log = tk.Text(log_frame, height=8, font=("Consolas", 9),
                             bg="#1e1e1e", fg="#d4d4d4",
                             insertbackground="white", state="disabled",
                             wrap="word")
        scroll = ttk.Scrollbar(log_frame, command=self._log.yview)
        self._log.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self._log.pack(fill="both", expand=True)

        btn_row = ttk.Frame(self, padding=(6, 0, 6, 6))
        btn_row.pack(fill="x")
        ttk.Button(btn_row, text="Clear log",
                   command=self._clear_log).pack(side="right")

    def _build_flash_tab(self, parent):
        # Columns: 0=label  1=entry(flex)  2=label  3=entry(flex)  4=Browse  5=action
        # No rowspan anywhere — each button occupies exactly one cell.

        # ── READ ──────────────────────────────────────────────────────────
        rf = ttk.LabelFrame(parent, text="Read", padding=6)
        rf.pack(fill="x", pady=4)
        rf.columnconfigure(1, weight=1)
        rf.columnconfigure(3, weight=1)

        # Row 0 — address + length
        ttk.Label(rf, text="Address:").grid(row=0, column=0, sticky="w")
        self._r_addr = ttk.Entry(rf, width=12)
        self._r_addr.insert(0, "0x000000")
        self._r_addr.grid(row=0, column=1, sticky="ew", padx=4)

        ttk.Label(rf, text="Length:").grid(row=0, column=2, sticky="w", padx=(8, 0))
        self._r_len = ttk.Entry(rf, width=10)
        self._r_len.insert(0, "256")
        self._r_len.grid(row=0, column=3, sticky="ew", padx=4)

        # Row 1 — output file + Browse + Read (Browse & Read always adjacent)
        # File entry spans col 1-3 (same width as addr+length above).
        # Browse=col4, Read=col5 — both outside the weighted columns.
        ttk.Label(rf, text="Output:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self._r_file = ttk.Entry(rf)
        self._r_file.grid(row=1, column=1, columnspan=3, sticky="ew",
                          padx=4, pady=(4, 0))
        ttk.Button(rf, text="Browse…",
                   command=lambda: self._browse_save(self._r_file, "bin")
                   ).grid(row=1, column=4, padx=(2, 2), pady=(4, 0))
        ttk.Button(rf, text="Read →",
                   command=self._do_read).grid(row=1, column=5, padx=(0, 0),
                                               pady=(4, 0))

        # ── WRITE ─────────────────────────────────────────────────────────
        wf = ttk.LabelFrame(parent, text="Write  (auto-erase)", padding=6)
        wf.pack(fill="x", pady=4)
        wf.columnconfigure(1, weight=1)

        # Row 0 — address
        ttk.Label(wf, text="Address:").grid(row=0, column=0, sticky="w")
        self._w_addr = ttk.Entry(wf, width=12)
        self._w_addr.insert(0, "0x000000")
        self._w_addr.grid(row=0, column=1, sticky="ew", padx=4)

        # Row 1 — input file + Browse + Write (all on the same row)
        ttk.Label(wf, text="Input:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self._w_file = ttk.Entry(wf)
        self._w_file.grid(row=1, column=1, sticky="ew", padx=4, pady=(4, 0))
        ttk.Button(wf, text="Browse…",
                   command=lambda: self._browse_open(self._w_file)
                   ).grid(row=1, column=2, padx=2, pady=(4, 0))
        ttk.Button(wf, text="Write →",
                   command=self._do_write).grid(row=1, column=3, padx=(6, 0),
                                                pady=(4, 0))

        # ── ERASE ─────────────────────────────────────────────────────────
        ef = ttk.LabelFrame(parent, text="Erase", padding=6)
        ef.pack(fill="x", pady=4)
        ef.columnconfigure(1, weight=1)
        ef.columnconfigure(3, weight=1)

        # Row 0 — address + length + Erase (all on the same row)
        ttk.Label(ef, text="Address:").grid(row=0, column=0, sticky="w")
        self._e_addr = ttk.Entry(ef, width=12)
        self._e_addr.insert(0, "0x000000")
        self._e_addr.grid(row=0, column=1, sticky="ew", padx=4)

        ttk.Label(ef, text="Length:").grid(row=0, column=2, sticky="w", padx=(8, 0))
        self._e_len = ttk.Entry(ef, width=10)
        self._e_len.insert(0, "4096")
        self._e_len.grid(row=0, column=3, sticky="ew", padx=4)

        ttk.Button(ef, text="Erase →",
                   command=self._do_erase).grid(row=0, column=4, padx=(6, 0))

    def _build_logo_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

        # ── UPLOAD ────────────────────────────────────────────────────────
        uf = ttk.LabelFrame(parent, text="Upload logo", padding=8)
        uf.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        uf.columnconfigure(1, weight=1)

        ttk.Label(uf, text="Image:").grid(row=0, column=0, sticky="w")
        self._ul_file = ttk.Entry(uf)
        self._ul_file.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(uf, text="Browse…",
                   command=lambda: self._browse_image(self._ul_file)
                   ).grid(row=0, column=2)

        ttk.Label(uf, text="Width:").grid(row=1, column=0, sticky="w", pady=4)
        self._ul_w = ttk.Entry(uf, width=6)
        self._ul_w.insert(0, "480")
        self._ul_w.grid(row=1, column=1, sticky="w", padx=4)

        ttk.Label(uf, text="Height:").grid(row=2, column=0, sticky="w")
        self._ul_h = ttk.Entry(uf, width=6)
        self._ul_h.insert(0, "320")
        self._ul_h.grid(row=2, column=1, sticky="w", padx=4)

        self._ul_preview = ttk.Label(uf, text="(no preview)", anchor="center",
                                      relief="sunken", width=20)
        self._ul_preview.grid(row=3, column=0, columnspan=3,
                               sticky="nsew", pady=8)
        uf.rowconfigure(3, weight=1)
        self._ul_tk_img = None

        ttk.Button(uf, text="⬆  Upload Logo",
                   command=self._do_logo_upload).grid(row=4, column=0,
                                                       columnspan=3, pady=4)
        self._ul_file.bind("<FocusOut>", lambda e: self._update_upload_preview())
        self._ul_file.bind("<Return>",   lambda e: self._update_upload_preview())

        # ── DOWNLOAD ──────────────────────────────────────────────────────
        df = ttk.LabelFrame(parent, text="Download logo", padding=8)
        df.grid(row=0, column=1, sticky="nsew")
        df.columnconfigure(1, weight=1)

        ttk.Label(df, text="Output:").grid(row=0, column=0, sticky="w")
        self._dl_file = ttk.Entry(df)
        self._dl_file.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(df, text="Browse…",
                   command=lambda: self._browse_save(self._dl_file, "png")
                   ).grid(row=0, column=2)

        ttk.Label(df, text="Width:").grid(row=1, column=0, sticky="w", pady=4)
        self._dl_w = ttk.Entry(df, width=6)
        self._dl_w.insert(0, "480")
        self._dl_w.grid(row=1, column=1, sticky="w", padx=4)

        ttk.Label(df, text="Height:").grid(row=2, column=0, sticky="w")
        self._dl_h = ttk.Entry(df, width=6)
        self._dl_h.insert(0, "320")
        self._dl_h.grid(row=2, column=1, sticky="w", padx=4)

        self._dl_preview = ttk.Label(df, text="(no preview)", anchor="center",
                                      relief="sunken", width=20)
        self._dl_preview.grid(row=3, column=0, columnspan=3,
                               sticky="nsew", pady=8)
        df.rowconfigure(3, weight=1)
        self._dl_tk_img = None

        ttk.Button(df, text="⬇  Download Logo",
                   command=self._do_logo_download).grid(row=4, column=0,
                                                         columnspan=3, pady=4)

    # -----------------------------------------------------------------------
    # Helper: preview
    # -----------------------------------------------------------------------
    def _show_preview(self, label_widget, attr_name: str, img: "Image.Image"):
        """Resize and display a PIL image in a label widget."""
        if not HAS_PIL:
            return
        thumb = img.copy()
        thumb.thumbnail((200, 140), Image.LANCZOS)
        tk_img = ImageTk.PhotoImage(thumb)
        setattr(self, attr_name, tk_img)  # keep reference
        label_widget.configure(image=tk_img, text="")

    def _update_upload_preview(self):
        path = self._ul_file.get().strip()
        if not path or not os.path.isfile(path) or not HAS_PIL:
            return
        try:
            img = Image.open(path)
            self._show_preview(self._ul_preview, "_ul_tk_img", img)
        except Exception:
            pass

    # -----------------------------------------------------------------------
    # Helper: file dialogs
    # -----------------------------------------------------------------------
    def _browse_open(self, entry: ttk.Entry):
        path = filedialog.askopenfilename()
        if path:
            entry.delete(0, "end")
            entry.insert(0, path)

    def _browse_image(self, entry: ttk.Entry):
        path = filedialog.askopenfilename(
            filetypes=[("Images", "*.png *.jpg *.jpeg *.bmp *.gif"), ("All", "*.*")])
        if path:
            entry.delete(0, "end")
            entry.insert(0, path)
            self._update_upload_preview()

    def _browse_save(self, entry: ttk.Entry, ext: str):
        path = filedialog.asksaveasfilename(
            defaultextension=f".{ext}",
            filetypes=[(ext.upper(), f"*.{ext}"), ("All", "*.*")])
        if path:
            entry.delete(0, "end")
            entry.insert(0, path)

    # -----------------------------------------------------------------------
    # Port management
    # -----------------------------------------------------------------------
    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _get_tool(self) -> GUIFlashTool:
        port = self._port_var.get().strip()
        if not port:
            raise ValueError("No port selected")
        return GUIFlashTool(port,
                            progress_cb=self._on_progress,
                            log_cb=self._queue_log)

    # -----------------------------------------------------------------------
    # Logging
    # -----------------------------------------------------------------------
    def _queue_log(self, msg: str):
        self._queue.put(("log", msg))

    def _queue_done(self, msg: str = ""):
        self._queue.put(("done", msg))

    def _queue_error(self, msg: str):
        self._queue.put(("error", msg))

    def _queue_result(self, key: str, value):
        self._queue.put(("result", (key, value)))

    def _on_progress(self, done: int, total: int):
        pct = (done * 100 // total) if total else 100
        self._queue.put(("progress", pct))

    def _append_log(self, msg: str):
        self._log.configure(state="normal")
        self._log.insert("end", msg + "\n")
        self._log.see("end")
        self._log.configure(state="disabled")

    def _clear_log(self):
        self._log.configure(state="normal")
        self._log.delete("1.0", "end")
        self._log.configure(state="disabled")

    # -----------------------------------------------------------------------
    # Queue polling (runs on main thread via after())
    # -----------------------------------------------------------------------
    def _poll_queue(self):
        try:
            while True:
                msg_type, payload = self._queue.get_nowait()
                if msg_type == "log":
                    self._append_log(payload)
                elif msg_type == "progress":
                    self._prog_var.set(payload)
                    self._prog_lbl.configure(text=f"{payload}%")
                elif msg_type == "done":
                    self._busy = False
                    self._prog_var.set(100)
                    self._prog_lbl.configure(text="Done")
                    if payload:
                        self._append_log(payload)
                elif msg_type == "error":
                    self._busy = False
                    self._prog_var.set(0)
                    self._prog_lbl.configure(text="Error")
                    self._append_log(f"ERROR: {payload}")
                    messagebox.showerror("Error", payload)
                elif msg_type == "result":
                    key, value = payload
                    if key == "chip_id":
                        jedec = value
                        mfr   = (jedec >> 16) & 0xFF
                        dev   = jedec & 0xFFFF
                        txt   = f"JEDEC: 0x{jedec:06X}  MFR=0x{mfr:02X}"
                        self._id_lbl.configure(text=txt)
                        self._append_log(f"Chip ID: {txt}  DEV=0x{dev:04X}")
                    elif key == "dl_preview":
                        img = value
                        self._show_preview(self._dl_preview, "_dl_tk_img", img)
        except queue.Empty:
            pass
        self.after(50, self._poll_queue)

    # -----------------------------------------------------------------------
    # Background task runner
    # -----------------------------------------------------------------------
    def _run(self, fn):
        if self._busy:
            messagebox.showwarning("Busy", "Another operation is in progress.")
            return
        self._busy = True
        self._prog_var.set(0)
        self._prog_lbl.configure(text="0%")
        t = threading.Thread(target=fn, daemon=True)
        t.start()

    # -----------------------------------------------------------------------
    # Operations
    # -----------------------------------------------------------------------
    def _do_chip_id(self):
        def _task():
            try:
                tool = self._get_tool()
                self._queue_log(f"Reading chip ID from {self._port_var.get()}…")
                jedec = tool.chip_id()
                tool.close()
                self._queue_result("chip_id", jedec)
                self._queue_done()
            except Exception as e:
                self._queue_error(str(e))
        self._run(_task)

    def _do_read(self):
        try:
            addr   = int(self._r_addr.get(), 0)
            length = int(self._r_len.get(), 0)
            path   = self._r_file.get().strip()
        except ValueError as e:
            messagebox.showerror("Input error", str(e))
            return

        def _task():
            try:
                tool = self._get_tool()
                self._queue_log(f"Reading {length} bytes @ 0x{addr:06X}…")
                data = tool.read(addr, length)
                tool.close()
                if path:
                    with open(path, "wb") as f:
                        f.write(data)
                    self._queue_log(f"Saved to {path}")
                else:
                    lines = []
                    for i in range(0, len(data), 16):
                        chunk = data[i:i+16]
                        h = " ".join(f"{b:02X}" for b in chunk)
                        a = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in chunk)
                        lines.append(f"  {addr+i:06X}:  {h:<47}  {a}")
                    self._queue_log("\n".join(lines))
                self._queue_done("Read complete.")
            except Exception as e:
                self._queue_error(str(e))
        self._run(_task)

    def _do_write(self):
        try:
            addr = int(self._w_addr.get(), 0)
            path = self._w_file.get().strip()
            if not path:
                raise ValueError("No input file")
            with open(path, "rb") as f:
                data = f.read()
        except (ValueError, OSError) as e:
            messagebox.showerror("Input error", str(e))
            return

        def _task():
            try:
                tool = self._get_tool()
                self._queue_log(f"Erasing {len(data)} bytes @ 0x{addr:06X}…")
                tool.erase_range(addr, len(data))
                self._queue_log(f"Writing {len(data)} bytes…")
                tool.write_binary(addr, data)
                tool.close()
                self._queue_done("Write complete.")
            except Exception as e:
                self._queue_error(str(e))
        self._run(_task)

    def _do_erase(self):
        try:
            addr   = int(self._e_addr.get(), 0)
            length = int(self._e_len.get(), 0)
        except ValueError as e:
            messagebox.showerror("Input error", str(e))
            return

        def _task():
            try:
                tool = self._get_tool()
                self._queue_log(f"Erasing {length} bytes @ 0x{addr:06X}…")
                tool.erase_range(addr, length)
                tool.close()
                self._queue_done("Erase complete.")
            except Exception as e:
                self._queue_error(str(e))
        self._run(_task)

    def _do_logo_upload(self):
        path = self._ul_file.get().strip()
        try:
            w = int(self._ul_w.get())
            h = int(self._ul_h.get())
        except ValueError:
            messagebox.showerror("Input error", "Width/height must be integers")
            return
        if not path or not os.path.isfile(path):
            messagebox.showerror("Input error", "Select a valid image file")
            return
        if not HAS_PIL:
            messagebox.showerror("Missing dependency", "Run: pip install pillow")
            return

        def _task():
            try:
                tool = self._get_tool()
                total_bytes = w * h * 2
                self._queue_log(f"Converting {os.path.basename(path)} → {w}×{h} RGB565 ({total_bytes} B)…")
                rgb565 = FlashTool.image_to_rgb565(path, w, h)
                self._queue_log(f"Erasing logo area @ 0x{FLASH_ADDR_LOGO:06X}…")
                tool.erase_range(FLASH_ADDR_LOGO, total_bytes)
                self._queue_log("Writing logo…")
                tool.write_binary(FLASH_ADDR_LOGO, rgb565)
                tool.close()
                self._queue_done("Logo uploaded.")
            except Exception as e:
                self._queue_error(str(e))
        self._run(_task)

    def _do_logo_download(self):
        path = self._dl_file.get().strip()
        try:
            w = int(self._dl_w.get())
            h = int(self._dl_h.get())
        except ValueError:
            messagebox.showerror("Input error", "Width/height must be integers")
            return
        if not path:
            messagebox.showerror("Input error", "Set an output file path")
            return
        if not HAS_PIL:
            messagebox.showerror("Missing dependency", "Run: pip install pillow")
            return

        def _task():
            try:
                tool = self._get_tool()
                total_bytes = w * h * 2
                self._queue_log(f"Reading logo ({w}×{h}, {total_bytes} B) @ 0x{FLASH_ADDR_LOGO:06X}…")
                raw = tool.read(FLASH_ADDR_LOGO, total_bytes)
                tool.close()
                self._queue_log("Converting to image…")
                img = FlashTool.rgb565_to_image(raw, w, h)
                img.save(path)
                self._queue_log(f"Saved to {path}")
                self._queue_result("dl_preview", img)
                self._queue_done("Logo downloaded.")
            except Exception as e:
                self._queue_error(str(e))
        self._run(_task)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    app = App()
    app.mainloop()
