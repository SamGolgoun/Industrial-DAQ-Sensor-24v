import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

TARGET_PCB = Path("stm32g4-canfd-sensor-node.kicad_pcb")
BACKUP_DIR = Path("backups")
SCRIPT_TO_RUN = "route_nrst_u2_pin7.py"
DRC_SCRIPT = "run_drc_check.py"


def main() -> None:
    if not TARGET_PCB.exists():
        raise FileNotFoundError(f"Target PCB not found: {TARGET_PCB}")

    BACKUP_DIR.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dest = BACKUP_DIR / f"{TARGET_PCB.stem}_backup_{stamp}.kicad_pcb"
    shutil.copy2(TARGET_PCB, dest)
    if not dest.exists() or dest.stat().st_size == 0:
        raise RuntimeError(f"Backup verification failed: {dest}")
    print(f"Backup created: {dest}")

    rc = subprocess.run([sys.executable, SCRIPT_TO_RUN], check=False).returncode
    if rc != 0:
        raise RuntimeError(f"{SCRIPT_TO_RUN} exited with code {rc}. Restore from {dest} if needed.")

    if Path(DRC_SCRIPT).exists():
        subprocess.run([sys.executable, DRC_SCRIPT], check=False)


if __name__ == "__main__":
    main()
