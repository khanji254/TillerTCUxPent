#!/usr/bin/env python3
"""
flash_tcu.py - Factory NVS Provisioning & Flashing Tool for KSC TCU V2.1

Automatically manages sequential fleet naming (KTCU_0000, KTCU_0001, ...)
without requiring per-vehicle firmware compilation.

Features:
  - Generates NVS partition with device_name and optional provisioning keys.
  - Automatically queries chip MAC address via esptool.
  - Flashes NVS partition to 0x9000 (and optionally full firmware stack).
  - Logs all flashed units into tools/provisioned_fleet.csv.
  - Atomically increments next_id in tools/fleet_registry.json.

Usage:
  python3 tools/flash_tcu.py                     # Flash next sequential TCU
  python3 tools/flash_tcu.py --status            # Show fleet status
  python3 tools/flash_tcu.py --dry-run           # Preview next assignment
  python3 tools/flash_tcu.py --set-id 5          # Jump counter to 0005
  python3 tools/flash_tcu.py --name KTCU_TEST    # Flash custom name
  python3 tools/flash_tcu.py --full              # Flash firmware + NVS
"""

import argparse
import csv
import datetime
import glob
import importlib.util
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
REGISTRY_FILE = os.path.join(SCRIPT_DIR, "fleet_registry.json")
LOG_FILE = os.path.join(SCRIPT_DIR, "provisioned_fleet.csv")
NVS_PARTITION_OFFSET = "0x9000"
NVS_PARTITION_SIZE = 0x6000  # 24 KB


def load_registry():
    if not os.path.exists(REGISTRY_FILE):
        default = {"prefix": "KTCU_", "next_id": 0, "pad_width": 4}
        save_registry(default)
        return default
    with open(REGISTRY_FILE, "r") as f:
        return json.load(f)


def save_registry(data):
    with open(REGISTRY_FILE, "w") as f:
        json.dump(data, f, indent=2)


def get_next_device_name(registry):
    prefix = registry.get("prefix", "KTCU_")
    next_id = registry.get("next_id", 0)
    pad = registry.get("pad_width", 4)
    return f"{prefix}{next_id:0{pad}d}"


def generate_nvs_binary(device_name: str, prov_key="", prov_secret="", size=NVS_PARTITION_SIZE) -> bytes:
    """Generates a standard ESP-IDF NVS v2 partition binary natively."""
    bitmap = bytearray([0xFF] * 32)

    def set_slot(slot_idx, state=0b10):
        byte_idx = slot_idx // 4
        bit_offset = (slot_idx % 4) * 2
        bitmap[byte_idx] &= ~(0b11 << bit_offset)
        bitmap[byte_idx] |= (state << bit_offset)

    set_slot(0)  # Page header
    set_slot(1)  # Bitmap

    slots = []

    # Namespace entry: "tcu_creds" -> namespace index 1
    key_bytes = b"tcu_creds".ljust(16, b"\x00")
    data = struct.pack("<B7x", 1)
    crc = zlib.crc32(key_bytes + data) & 0xFFFFFFFF
    header = struct.pack("<BBBB I", 0, 0x01, 1, 0xFF, crc)
    slots.append(header + key_bytes + data)
    set_slot(2)

    curr_slot = 3

    def make_str_entry(ns_id, key_name, str_val):
        nonlocal curr_slot
        val_bytes = str_val.encode("utf-8") + b"\x00"
        val_len = len(val_bytes)
        data_slots_count = (val_len + 31) // 32
        total_span = 1 + data_slots_count

        # CRC16 (CCITT-FALSE)
        crc = 0xFFFF
        for byte in val_bytes:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        data_crc16 = crc

        k_bytes = key_name.encode("utf-8")[:15].ljust(16, b"\x00")
        entry_meta = struct.pack("<H H I", val_len, data_crc16, 0xFFFFFFFF)
        padded_data = val_bytes.ljust(data_slots_count * 32, b"\xff")

        entry_crc32 = zlib.crc32(k_bytes + entry_meta + padded_data) & 0xFFFFFFFF
        slot_header = struct.pack("<BBBB I", ns_id, 0x21, total_span, 0xFF, entry_crc32)

        out = [slot_header + k_bytes + entry_meta]
        set_slot(curr_slot)
        curr_slot += 1

        for i in range(data_slots_count):
            out.append(padded_data[i * 32 : (i + 1) * 32])
            set_slot(curr_slot)
            curr_slot += 1
        return out

    # Write device_name
    for s in make_str_entry(1, "device_name", device_name):
        slots.append(s)

    # Write optional provisioning key/secret
    if prov_key:
        for s in make_str_entry(1, "prov_key", prov_key):
            slots.append(s)
    if prov_secret:
        for s in make_str_entry(1, "prov_secret", prov_secret):
            slots.append(s)

    # Page Header (32 bytes)
    header_raw = struct.pack("<I I B 19s", 0xFFFFFFFE, 0, 0xFE, b"\xff" * 19)
    header_crc = zlib.crc32(header_raw[4:]) & 0xFFFFFFFF
    page_header = header_raw + struct.pack("<I", header_crc)

    slot_data = b"".join(slots)
    remaining = (126 * 32) - len(slot_data)
    slot_data += b"\xff" * remaining

    page0 = page_header + bytes(bitmap) + slot_data
    pages = [page0]

    for _ in range((size // 4096) - 1):
        pages.append(b"\xff" * 4096)

    return b"".join(pages)


def _python_has_module(python_bin, module_name):
    try:
        res = subprocess.run(
            [python_bin, "-c", f"import {module_name}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
        return res.returncode == 0
    except Exception:
        return False


def find_esptool_cmd(esptool_override=None):
    """Resolve an executable esptool command prefix across PATH, PlatformIO, and ESP-IDF."""
    if esptool_override:
        if not os.path.exists(esptool_override):
            return None
        if os.access(esptool_override, os.X_OK) and not esptool_override.endswith(".py"):
            return [esptool_override]
        return [sys.executable, esptool_override]

    pio_core = os.environ.get("PLATFORMIO_CORE_DIR", os.path.expanduser("~/.platformio"))
    pio_bins = [os.path.join(pio_core, "penv", "bin")]
    pio_cli = shutil.which("pio")
    if pio_cli:
        pio_bins.append(os.path.dirname(os.path.realpath(pio_cli)))

    # 1. Direct executables / wrapper scripts on PATH, PlatformIO penv, or ~/.local/bin
    direct_candidates = [
        shutil.which("esptool.py"),
        shutil.which("esptool"),
        os.path.expanduser("~/.local/bin/esptool.py"),
        os.path.expanduser("~/.local/bin/esptool"),
    ]
    for bin_dir in pio_bins:
        direct_candidates.extend([
            os.path.join(bin_dir, "esptool.py"),
            os.path.join(bin_dir, "esptool"),
        ])
    for path in direct_candidates:
        if path and os.path.exists(path):
            return [path]

    # 2. Find an available Python interpreter (prefer PlatformIO penv if present)
    python_candidates = []
    for bin_dir in pio_bins:
        for py_name in ("python", "python3"):
            py_path = os.path.join(bin_dir, py_name)
            if os.path.exists(py_path) and py_path not in python_candidates:
                python_candidates.append(py_path)
    if sys.executable not in python_candidates:
        python_candidates.append(sys.executable)

    # 3. PlatformIO's tool-esptoolpy package
    for pkg_dir in sorted(glob.glob(os.path.join(pio_core, "packages", "tool-esptoolpy*"))):
        esptool_py = os.path.join(pkg_dir, "esptool.py")
        if os.path.exists(esptool_py):
            return [python_candidates[0], esptool_py]

    # 4. Python -m esptool in current Python, PlatformIO penv, or ESP-IDF venv
    if importlib.util.find_spec("esptool") is not None:
        return [sys.executable, "-m", "esptool"]

    espressif_pythons = sorted(
        glob.glob(os.path.expanduser("~/.espressif/python_env/*/bin/python"))
    )
    for py_path in python_candidates + espressif_pythons:
        if py_path != sys.executable and _python_has_module(py_path, "esptool"):
            return [py_path, "-m", "esptool"]

    return None


def read_device_mac(esptool_cmd, port, baud):
    cmd = list(esptool_cmd)
    if port:
        cmd.extend(["--port", port])
    cmd.extend(["--baud", str(baud), "read_mac"])

    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        for line in res.stdout.splitlines():
            if "MAC:" in line:
                return line.split("MAC:")[-1].strip()
    except Exception:
        pass
    return "UNKNOWN"


def log_provision(device_name, mac, port, status):
    file_exists = os.path.exists(LOG_FILE)
    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["Timestamp", "Device Name", "MAC Address", "Port", "Status"])
        writer.writerow([
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            device_name,
            mac,
            port or "auto",
            status
        ])


def resolve_build_artifact(build_dir, *relative_candidates):
    """Return the first existing build artifact path, or the primary expected path."""
    for rel_path in relative_candidates:
        full_path = os.path.join(build_dir, rel_path)
        if os.path.exists(full_path):
            return full_path
    return os.path.join(build_dir, relative_candidates[0])


def main():
    parser = argparse.ArgumentParser(description="KSC TCU Factory NVS Provisioning & Flashing Tool")
    parser.add_argument("--status", action="store_true", help="Show current fleet registry status")
    parser.add_argument("--dry-run", action="store_true", help="Preview next device name without flashing")
    parser.add_argument("--set-id", type=int, help="Update next sequential ID")
    parser.add_argument("--set-prefix", type=str, help="Update device prefix (e.g. KTCU_)")
    parser.add_argument("--name", type=str, help="Override device name for this flash")
    parser.add_argument("--port", type=str, default=None, help="Serial port (auto-detects if omitted)")
    parser.add_argument("--baud", type=int, default=460800, help="Flashing baud rate (default: 460800)")
    parser.add_argument("--full", action="store_true", help="Flash complete firmware stack + NVS")
    parser.add_argument("--esptool", type=str, default=None, help="Path to esptool.py / esptool executable")
    parser.add_argument("--prov-key", type=str, default="", help="Optional ThingsBoard provisioning key for NVS")
    parser.add_argument("--prov-secret", type=str, default="", help="Optional ThingsBoard provisioning secret for NVS")

    args = parser.parse_args()
    registry = load_registry()

    if args.set_prefix:
        registry["prefix"] = args.set_prefix
        save_registry(registry)
        print(f"[*] Prefix updated to: {registry['prefix']}")

    if args.set_id is not None:
        registry["next_id"] = args.set_id
        save_registry(registry)
        print(f"[*] Next ID updated to: {registry['next_id']} ({get_next_device_name(registry)})")
        return

    if args.status:
        print("=== KSC TCU Fleet Registry Status ===")
        print(f"Prefix:      {registry.get('prefix', 'KTCU_')}")
        print(f"Next Name:   {get_next_device_name(registry)}")
        print(f"Registry:    {REGISTRY_FILE}")
        print(f"Log File:    {LOG_FILE}")
        if os.path.exists(LOG_FILE):
            with open(LOG_FILE, "r") as f:
                lines = f.readlines()
                count = max(0, len(lines) - 1)
                print(f"Total Flashed: {count}")
                if count > 0:
                    print(f"Last entry:  {lines[-1].strip()}")
        return

    device_name = args.name if args.name else get_next_device_name(registry)

    if args.dry_run:
        print(f"[DRY-RUN] Next device to be flashed: {device_name}")
        return

    print("==================================================")
    print(f"  Flashing TCU Device: {device_name}")
    print("==================================================")

    esptool_cmd = find_esptool_cmd(args.esptool)
    if not esptool_cmd:
        print("[!] Error: Could not locate esptool (checked PATH, ~/.platformio/packages/tool-esptoolpy, and Python environments).")
        print("    Install with 'pip install esptool' or pass '--esptool /path/to/esptool.py'.")
        sys.exit(1)

    print(f"[*] Using esptool: {' '.join(esptool_cmd)}")
    print(f"[*] Detecting device on port: {args.port or 'auto-detect'}...")
    mac = read_device_mac(esptool_cmd, args.port, args.baud)
    print(f"[*] Target ESP32 MAC: {mac}")

    # Generate NVS binary
    nvs_bin_data = generate_nvs_binary(device_name, args.prov_key, args.prov_secret)

    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as tmp_nvs:
        tmp_nvs.write(nvs_bin_data)
        tmp_nvs_path = tmp_nvs.name

    try:
        flash_cmd = list(esptool_cmd)
        if args.port:
            flash_cmd.extend(["--port", args.port])
        flash_cmd.extend(["--baud", str(args.baud), "write_flash"])

        if args.full:
            # Full firmware flashing (PlatformIO places artifacts directly in build_dir)
            build_dir = os.path.join(PROJECT_DIR, ".pio", "build", "esp32-s3-devkitc-1")
            bootloader = resolve_build_artifact(
                build_dir, "bootloader.bin", os.path.join("bootloader", "bootloader.bin")
            )
            partitions = resolve_build_artifact(
                build_dir, "partitions.bin", os.path.join("partition_table", "partition-table.bin")
            )
            app_bin = resolve_build_artifact(build_dir, "firmware.bin", "KSC_TCU.bin")

            for label, path in [
                ("Bootloader", bootloader),
                ("Partition table", partitions),
                ("Firmware", app_bin),
            ]:
                if not os.path.exists(path):
                    print(f"[!] Error: {label} binary not found at {path}. Build first with PlatformIO.")
                    sys.exit(1)

            flash_cmd.extend([
                "0x0", bootloader,
                "0x8000", partitions,
                NVS_PARTITION_OFFSET, tmp_nvs_path,
                "0x10000", app_bin,
            ])
            print(f"[*] Flashing FULL firmware + NVS partition ({device_name})...")
        else:
            flash_cmd.extend([NVS_PARTITION_OFFSET, tmp_nvs_path])
            print(f"[*] Flashing NVS partition to {NVS_PARTITION_OFFSET} ({device_name})...")

        subprocess.run(flash_cmd, check=True)
        print(f"[+] SUCCESS: {device_name} provisioned successfully!")

        # Log and increment registry if not a manual name override
        log_provision(device_name, mac, args.port, "SUCCESS")
        if not args.name:
            registry["next_id"] += 1
            save_registry(registry)
            print(f"[*] Next TCU will be: {get_next_device_name(registry)}")

    except Exception as e:
        print(f"[-] FLASH FAILED: {e}")
        log_provision(device_name, mac, args.port, f"FAILED: {e}")
        sys.exit(1)
    finally:
        if os.path.exists(tmp_nvs_path):
            os.remove(tmp_nvs_path)


if __name__ == "__main__":
    main()

