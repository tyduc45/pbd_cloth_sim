"""Build an isolated source snapshot and run real UE automation tests.

The working editor can remain open. No live project DLL, content, or map is modified.
Usage: python Scripts/test_pbd_collision.py [--engine D:/UE5/UE_5.8]
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--engine", type=Path, default=Path("D:/UE5/UE_5.8"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    validation = root / "Saved" / "CollisionValidation"
    validation.mkdir(parents=True, exist_ok=True)
    shutil.copytree(root / "Source", validation / "Source", dirs_exist_ok=True)
    project = validation / "pdd_cloth.uproject"
    shutil.copy2(root / "pdd_cloth.uproject", project)
    report = validation / "Report"
    build_log = validation / "Build.log"
    command = [str(args.engine / "Engine/Build/BatchFiles/Build.bat"),
               "pdd_clothEditor", "Win64", "Development", f"-Project={project}",
               "-WaitMutex", "-NoHotReloadFromIDE"]
    print(f"Building source snapshot; log: {build_log}", flush=True)
    with build_log.open("w", encoding="utf-8") as output:
        subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=True, timeout=900)
    # Remove only the previous summary file, so a failed run cannot reuse an old pass.
    summary_file = report / "index.json"
    summary_file.unlink(missing_ok=True)
    run_log = validation / "Automation.log"
    command = [str(args.engine / "Engine/Binaries/Win64/UnrealEditor-Cmd.exe"), str(project),
               "-unattended", "-nop4", "-NullRHI", "-nosplash", "-nosound",
               "-ExecCmds=Automation RunTests PBD.Collision", "-TestExit=Automation Test Queue Empty",
               f"-ReportExportPath={report}", f"-abslog={run_log}"]
    print(f"Running PBD.Collision; log: {run_log}", flush=True)
    with (validation / "Process.log").open("w", encoding="utf-8") as output:
        subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=True, timeout=300)
    summary = json.loads(summary_file.read_text(encoding="utf-8-sig"))
    if summary.get("failed", 0) or summary.get("succeeded", 0) != 4:
        raise RuntimeError(f"Expected four passing tests: {summary_file}")
    print(f"PASS: {summary['succeeded']} tests; report: {summary_file}", flush=True)


if __name__ == "__main__":
    main()
