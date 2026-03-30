import os
import time
import shutil
from pathlib import Path

# --------------------------------------------------
# Places to search for freshly compiled UF2 files
# --------------------------------------------------
SEARCH_ROOTS = [
    Path.cwd(),
    Path.home() / "Arduino",
    Path.home() / ".var/app/cc.arduino.IDE2/cache",
    Path("/tmp"),
]

UF2_EXT = ".uf2"

# Linux mount locations to check for RPI-RP2
MOUNT_BASES = [
    Path("/run/media"),
    Path("/media"),
    Path("/mnt"),
]

def find_latest_uf2():
    print("Searching for latest UF2 file...")
    candidates = []

    for root in SEARCH_ROOTS:
        if not root.exists():
            continue

        for dirpath, _, filenames in os.walk(root):
            for name in filenames:
                if name.lower().endswith(UF2_EXT):
                    full = Path(dirpath) / name
                    try:
                        mtime = full.stat().st_mtime
                        candidates.append((mtime, full))
                    except OSError:
                        pass

    if not candidates:
        raise RuntimeError(
            "No UF2 files found.\n"
            "Compile first using Arduino IDE, preferably with:\n"
            "Sketch -> Export Compiled Binary"
        )

    candidates.sort(key=lambda x: x[0], reverse=True)
    newest = candidates[0][1]
    print(f"Found newest UF2: {newest}")
    return newest

def find_rpi_rp2_mount():
    for base in MOUNT_BASES:
        if not base.exists():
            continue

        # search a few levels down
        for dirpath, dirnames, _ in os.walk(base):
            for d in dirnames:
                if d == "RPI-RP2":
                    mount = Path(dirpath) / d
                    print(f"Found board at: {mount}")
                    return mount
    return None

def wait_for_board(timeout=20):
    print("Waiting for RP2040 board in BOOTSEL mode...")
    start = time.time()

    while time.time() - start < timeout:
        mount = find_rpi_rp2_mount()
        if mount:
            return mount
        time.sleep(1)

    raise RuntimeError(
        "RPI-RP2 drive not found.\n"
        "Hold BOOTSEL while plugging in the board, then run again."
    )

def upload_latest():
    uf2 = find_latest_uf2()
    board_mount = wait_for_board()

    dest = board_mount / uf2.name
    print(f"Copying:\n  {uf2}\n-> {dest}")
    shutil.copy2(uf2, dest)
    print("Upload complete.")

if __name__ == "__main__":
    try:
        upload_latest()
    except Exception as e:
        print(f"ERROR: {e}")
