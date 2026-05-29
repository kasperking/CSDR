#!/usr/bin/env python3
"""
flash_tool.py  –  NOR flash programmer for CSDR over USB CDC binary protocol.

Usage examples:
  python flash_tool.py COM5 chip-id
  python flash_tool.py COM5 read  0x003000 256 logo_first256.bin
  python flash_tool.py COM5 write 0x003000 mydata.bin
  python flash_tool.py COM5 erase 0x003000 307200
  python flash_tool.py COM5 logo-upload  boot_logo.png
  python flash_tool.py COM5 logo-upload  boot_logo.png --width 480 --height 320
  python flash_tool.py COM5 logo-download logo_out.png
  python flash_tool.py COM5 logo-download logo_out.png --width 480 --height 320

Dependencies:
  pip install pyserial pillow
"""

import argparse
import struct
import sys
import time

import serial

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
MAGIC             = 0xFF
CMD_READ          = 0x01
CMD_WRITE         = 0x02
CMD_SECTOR_ERASE  = 0x03
CMD_CHIP_ID       = 0x04
CMD_BLOCK64_ERASE = 0x05

STATUS_OK         = 0x00
STATUS_ERR_FLASH  = 0x01
STATUS_ERR_ADDR   = 0x02
STATUS_ERR_LEN    = 0x03
STATUS_ERR_CMD    = 0x04

STATUS_NAMES = {
    STATUS_OK:        "OK",
    STATUS_ERR_FLASH: "ERR_FLASH",
    STATUS_ERR_ADDR:  "ERR_ADDR",
    STATUS_ERR_LEN:   "ERR_LEN",
    STATUS_ERR_CMD:   "ERR_CMD",
}

MAX_PAGE       = 256          # bytes per read or write
SECTOR_SIZE    = 4096         # bytes
BLOCK64_SIZE   = 65536        # bytes
FLASH_SIZE     = 16 * 1024 * 1024

# Flash layout (mirrors w25q128.h)
FLASH_ADDR_SETTINGS = 0x000000
FLASH_ADDR_BAND_CAL = 0x001000
FLASH_ADDR_LOGO     = 0x003000

# Timeouts
RESP_TIMEOUT_NORMAL = 2.0    # seconds — read / write / chip-id
RESP_TIMEOUT_ERASE  = 3.0    # seconds — sector erase (max 400 ms per chip spec)
RESP_TIMEOUT_B64    = 5.0    # seconds — 64 KB block erase (max 2 s per chip spec)


class FlashProtoError(Exception):
    pass


class FlashTool:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, baudrate=115200, timeout=RESP_TIMEOUT_NORMAL)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    # -----------------------------------------------------------------------
    # Low-level frame I/O
    # -----------------------------------------------------------------------
    def _send(self, cmd: int, addr: int, payload: bytes, timeout: float = RESP_TIMEOUT_NORMAL) -> tuple:
        """Send one request frame and return (status, response_data)."""
        dlen = len(payload)
        frame = bytes([
            MAGIC, cmd,
            (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF,
            (dlen >> 8) & 0xFF, dlen & 0xFF,
        ]) + payload

        self.ser.timeout = timeout
        self.ser.reset_input_buffer()
        self.ser.write(frame)

        # Response header: magic(1) status(1) rlen(2)
        hdr = self.ser.read(4)
        if len(hdr) < 4:
            raise FlashProtoError(f"Timeout waiting for response (got {len(hdr)}/4 bytes)")
        if hdr[0] != MAGIC:
            raise FlashProtoError(f"Bad magic in response: 0x{hdr[0]:02X}")

        status = hdr[1]
        rlen   = (hdr[2] << 8) | hdr[3]

        data = b""
        if rlen > 0:
            data = self.ser.read(rlen)
            if len(data) < rlen:
                raise FlashProtoError(f"Truncated response data ({len(data)}/{rlen})")

        if status != STATUS_OK:
            name = STATUS_NAMES.get(status, f"0x{status:02X}")
            raise FlashProtoError(f"Device returned {name}")

        return status, data

    # -----------------------------------------------------------------------
    # High-level operations
    # -----------------------------------------------------------------------
    def chip_id(self) -> int:
        _, data = self._send(CMD_CHIP_ID, 0, b"")
        return struct.unpack(">I", data)[0]

    def read(self, addr: int, length: int) -> bytes:
        """Read `length` bytes starting at `addr` (handles chunking)."""
        result = bytearray()
        while length > 0:
            chunk = min(length, MAX_PAGE)
            _, data = self._send(CMD_READ, addr,
                                 bytes([(chunk >> 8) & 0xFF, chunk & 0xFF]))
            if len(data) != chunk:
                raise FlashProtoError(f"Short read: expected {chunk}, got {len(data)}")
            result.extend(data)
            addr   += chunk
            length -= chunk
        return bytes(result)

    def sector_erase(self, addr: int):
        """Erase one 4 KB sector at `addr` (must be sector-aligned)."""
        if addr % SECTOR_SIZE:
            raise ValueError(f"Address 0x{addr:06X} not 4 KB aligned")
        self._send(CMD_SECTOR_ERASE, addr, b"", timeout=RESP_TIMEOUT_ERASE)

    def block64_erase(self, addr: int):
        """Erase one 64 KB block at `addr` (must be 64 KB aligned)."""
        if addr % BLOCK64_SIZE:
            raise ValueError(f"Address 0x{addr:06X} not 64 KB aligned")
        self._send(CMD_BLOCK64_ERASE, addr, b"", timeout=RESP_TIMEOUT_B64)

    def write_page(self, addr: int, data: bytes):
        """Write up to 256 bytes at `addr` (no auto-erase)."""
        if len(data) > MAX_PAGE:
            raise ValueError(f"Payload too large: {len(data)} > {MAX_PAGE}")
        self._send(CMD_WRITE, addr, data)

    def erase_range(self, addr: int, length: int, verbose: bool = True):
        """
        Erase all sectors covering [addr, addr+length).
        Uses 64 KB block erase where possible for speed.
        """
        start = (addr // SECTOR_SIZE) * SECTOR_SIZE
        end   = ((addr + length - 1) // SECTOR_SIZE + 1) * SECTOR_SIZE
        pos   = start
        total = end - start
        done  = 0

        while pos < end:
            if pos % BLOCK64_SIZE == 0 and (end - pos) >= BLOCK64_SIZE:
                if verbose:
                    print(f"  erase 64K block @ 0x{pos:06X}  "
                          f"({done*100//total:3d}%)", end="\r", flush=True)
                self.block64_erase(pos)
                done += BLOCK64_SIZE
                pos  += BLOCK64_SIZE
            else:
                if verbose:
                    print(f"  erase 4K sector @ 0x{pos:06X}  "
                          f"({done*100//total:3d}%)", end="\r", flush=True)
                self.sector_erase(pos)
                done += SECTOR_SIZE
                pos  += SECTOR_SIZE

        if verbose:
            print(f"  erase done — {done} bytes erased.          ")

    def write_binary(self, addr: int, data: bytes, verbose: bool = True):
        """Write binary blob at `addr` in page-sized chunks (no auto-erase)."""
        total = len(data)
        offset = 0
        while offset < total:
            # Align to page boundary
            page_off = (addr + offset) % MAX_PAGE
            chunk    = min(MAX_PAGE - page_off, total - offset)
            self.write_page(addr + offset, data[offset:offset + chunk])
            offset += chunk
            if verbose:
                print(f"  write @ 0x{addr+offset-chunk:06X}  "
                      f"({offset*100//total:3d}%)", end="\r", flush=True)
        if verbose:
            print(f"  write done — {total} bytes written.         ")

    # -----------------------------------------------------------------------
    # Logo helpers
    # -----------------------------------------------------------------------
    @staticmethod
    def image_to_rgb565(path: str, width: int, height: int) -> bytes:
        """Convert image file to raw RGB565 little-endian bytes (width×height)."""
        if not HAS_PIL:
            raise RuntimeError("Pillow is not installed — run: pip install pillow")
        img = Image.open(path).convert("RGB")
        if img.size != (width, height):
            img = img.resize((width, height), Image.LANCZOS)
        pixels = img.load()
        buf = bytearray(width * height * 2)
        idx = 0
        for y in range(height):
            for x in range(width):
                r, g, b = pixels[x, y]
                rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
                # Little-endian
                buf[idx]     = rgb565 & 0xFF
                buf[idx + 1] = (rgb565 >> 8) & 0xFF
                idx += 2
        return bytes(buf)

    @staticmethod
    def rgb565_to_image(data: bytes, width: int, height: int) -> "Image.Image":
        """Convert raw RGB565 little-endian bytes back to a PIL Image."""
        if not HAS_PIL:
            raise RuntimeError("Pillow is not installed — run: pip install pillow")
        img = Image.new("RGB", (width, height))
        pixels = img.load()
        idx = 0
        for y in range(height):
            for x in range(width):
                lo  = data[idx]
                hi  = data[idx + 1]
                idx += 2
                rgb565 = (hi << 8) | lo
                r = (rgb565 >> 8) & 0xF8
                g = (rgb565 >> 3) & 0xFC
                b = (rgb565 << 3) & 0xF8
                pixels[x, y] = (r, g, b)
        return img

    def logo_upload(self, image_path: str, width: int, height: int, verbose: bool = True):
        """Convert image and program it into the boot logo area."""
        total_bytes = width * height * 2
        if verbose:
            print(f"Converting {image_path} → {width}×{height} RGB565 "
                  f"({total_bytes} bytes)...")
        rgb565 = self.image_to_rgb565(image_path, width, height)

        if verbose:
            print(f"Erasing logo area @ 0x{FLASH_ADDR_LOGO:06X} "
                  f"({total_bytes} bytes)...")
        self.erase_range(FLASH_ADDR_LOGO, total_bytes, verbose=verbose)

        if verbose:
            print(f"Writing logo...")
        self.write_binary(FLASH_ADDR_LOGO, rgb565, verbose=verbose)
        if verbose:
            print("Logo upload complete.")

    def logo_download(self, output_path: str, width: int, height: int, verbose: bool = True):
        """Read the boot logo area and save as PNG."""
        total_bytes = width * height * 2
        if verbose:
            print(f"Reading logo area @ 0x{FLASH_ADDR_LOGO:06X} "
                  f"({total_bytes} bytes)...")
        raw = self.read(FLASH_ADDR_LOGO, total_bytes)
        if verbose:
            print("Converting to image...")
        img = self.rgb565_to_image(raw, width, height)
        img.save(output_path)
        if verbose:
            print(f"Logo saved to {output_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_addr(s: str) -> int:
    return int(s, 0)


def cmd_chip_id(tool: FlashTool, _args):
    jedec = tool.chip_id()
    mfr   = (jedec >> 16) & 0xFF
    dev   = jedec & 0xFFFF
    print(f"JEDEC ID: 0x{jedec:06X}  (MFR=0x{mfr:02X}, DEV=0x{dev:04X})")
    if mfr == 0xEF:
        print("Manufacturer: Winbond")
    elif mfr == 0xC8:
        print("Manufacturer: GigaDevice")
    elif mfr == 0x20:
        print("Manufacturer: Micron/ST")


def cmd_read(tool: FlashTool, args):
    addr = parse_addr(args.addr)
    length = int(args.length, 0)
    print(f"Reading {length} bytes @ 0x{addr:06X}...")
    data = tool.read(addr, length)
    if args.output:
        with open(args.output, "wb") as f:
            f.write(data)
        print(f"Saved to {args.output}")
    else:
        # Hex dump
        for i in range(0, len(data), 16):
            chunk = data[i:i+16]
            hex_part = " ".join(f"{b:02X}" for b in chunk)
            asc_part = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in chunk)
            print(f"  {addr+i:06X}:  {hex_part:<47}  {asc_part}")


def cmd_write(tool: FlashTool, args):
    addr = parse_addr(args.addr)
    with open(args.input, "rb") as f:
        data = f.read()
    print(f"Erasing {len(data)} bytes @ 0x{addr:06X}...")
    tool.erase_range(addr, len(data))
    print(f"Writing {len(data)} bytes...")
    tool.write_binary(addr, data)


def cmd_erase(tool: FlashTool, args):
    addr = parse_addr(args.addr)
    length = int(args.length, 0)
    print(f"Erasing {length} bytes @ 0x{addr:06X}...")
    tool.erase_range(addr, length)


def cmd_logo_upload(tool: FlashTool, args):
    tool.logo_upload(args.image, args.width, args.height)


def cmd_logo_download(tool: FlashTool, args):
    tool.logo_download(args.output, args.width, args.height)


def main():
    parser = argparse.ArgumentParser(
        description="CSDR NOR flash tool (binary protocol over USB CDC)")
    parser.add_argument("port", help="Serial port (e.g. COM5 or /dev/ttyACM0)")

    sub = parser.add_subparsers(dest="command", required=True)

    # chip-id
    sub.add_parser("chip-id", help="Read JEDEC chip ID")

    # read
    p_read = sub.add_parser("read", help="Read bytes from flash")
    p_read.add_argument("addr",   help="Start address (hex: 0x003000)")
    p_read.add_argument("length", help="Byte count (dec or hex)")
    p_read.add_argument("output", nargs="?", help="Output file (default: hex dump)")

    # write
    p_write = sub.add_parser("write", help="Write binary file to flash (auto-erase)")
    p_write.add_argument("addr",  help="Start address")
    p_write.add_argument("input", help="Input binary file")

    # erase
    p_erase = sub.add_parser("erase", help="Erase flash range")
    p_erase.add_argument("addr",   help="Start address")
    p_erase.add_argument("length", help="Byte count")

    # logo-upload
    p_lu = sub.add_parser("logo-upload", help="Upload boot logo image")
    p_lu.add_argument("image", help="Source image (PNG/JPG/BMP)")
    p_lu.add_argument("--width",  type=int, default=480, help="LCD width  (default 480)")
    p_lu.add_argument("--height", type=int, default=320, help="LCD height (default 320)")

    # logo-download
    p_ld = sub.add_parser("logo-download", help="Download boot logo as PNG")
    p_ld.add_argument("output", help="Output PNG file")
    p_ld.add_argument("--width",  type=int, default=480, help="LCD width  (default 480)")
    p_ld.add_argument("--height", type=int, default=320, help="LCD height (default 320)")

    args = parser.parse_args()

    dispatch = {
        "chip-id":       cmd_chip_id,
        "read":          cmd_read,
        "write":         cmd_write,
        "erase":         cmd_erase,
        "logo-upload":   cmd_logo_upload,
        "logo-download": cmd_logo_download,
    }

    try:
        tool = FlashTool(args.port)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        dispatch[args.command](tool, args)
    except FlashProtoError as e:
        print(f"Protocol error: {e}", file=sys.stderr)
        sys.exit(1)
    except (ValueError, RuntimeError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(1)
    finally:
        tool.close()


if __name__ == "__main__":
    main()
