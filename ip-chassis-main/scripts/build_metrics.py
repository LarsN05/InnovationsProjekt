#!/usr/bin/env python3
"""Build the firmware and collect build & code metrics.

Steps:
  1. Regenerates ip-motorshield/web-page.h from web-page.html (records the
     minification sizes).
  2. Compiles the firmware with arduino-cli (records flash and RAM usage).
  3. Counts C++ lines of code (excluding the generated web-page.h), split
     into: motorshield facade (everything under src/), web interface, and
     other application code.
  4. Writes a summary file (default: scripts/metrics-summary.txt, gitignored).

Run with the repo venv so the HTML minifier is available:
  .venv/bin/python scripts/build_metrics.py
"""
import argparse
import datetime
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SKETCH_DIR = REPO_ROOT / "ip-motorshield"
HTML_FILE = SKETCH_DIR / "web-page.html"
HEADER_FILE = SKETCH_DIR / "web-page.h"
FQBN = "esp32:esp32:esp32s3"
SOURCE_SUFFIXES = {".cpp", ".h", ".ino"}


def rebuild_web_page_header():
    result = subprocess.run(
        [sys.executable, str(REPO_ROOT / "scripts" / "html_to_header.py"), str(HTML_FILE), str(HEADER_FILE)],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        sys.exit(f"html_to_header.py failed:\n{result.stdout}{result.stderr}")

    match = re.search(r"HTML minified: (\d+) -> (\d+) bytes", result.stdout)
    if match is None:
        sys.exit(f"Could not parse minification metrics from:\n{result.stdout}{result.stderr}")
    return {"html_bytes": int(match.group(1)), "minified_bytes": int(match.group(2))}


def compile_firmware():
    result = subprocess.run(
        ["arduino-cli", "compile", "--fqbn", FQBN, str(SKETCH_DIR)],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        sys.exit(f"arduino-cli compile failed:\n{result.stdout}{result.stderr}")

    flash = re.search(r"Sketch uses (\d+) bytes \((\d+)%\).*?Maximum is (\d+)", result.stdout)
    ram = re.search(r"Global variables use (\d+) bytes \((\d+)%\).*?Maximum is (\d+)", result.stdout)
    if flash is None or ram is None:
        sys.exit(f"Could not parse size metrics from:\n{result.stdout}")
    return {
        "flash_bytes": int(flash.group(1)),
        "flash_percent": int(flash.group(2)),
        "flash_max_bytes": int(flash.group(3)),
        "ram_bytes": int(ram.group(1)),
        "ram_percent": int(ram.group(2)),
        "ram_max_bytes": int(ram.group(3)),
    }


def count_file_loc(path):
    """Counts code/comment/blank lines. A line with both code and a trailing
    comment counts as code (same convention as cloc)."""
    code = comment = blank = 0
    in_block_comment = False
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if in_block_comment:
            comment += 1
            if "*/" in stripped:
                in_block_comment = False
            continue
        if not stripped:
            blank += 1
        elif stripped.startswith("//"):
            comment += 1
        elif stripped.startswith("/*"):
            comment += 1
            if "*/" not in stripped[2:]:
                in_block_comment = True
        else:
            code += 1
    return {"files": 1, "code": code, "comment": comment, "blank": blank}


def categorize(path):
    relative = path.relative_to(SKETCH_DIR)
    if relative.parts[0] == "src":
        return "Motorshield facade (src/)"
    if relative.name.startswith("web-interface"):
        return "Web interface"
    return "Other application code"


def count_code_metrics():
    categories = {}
    for path in sorted(SKETCH_DIR.rglob("*")):
        if path.suffix not in SOURCE_SUFFIXES or path == HEADER_FILE:
            continue
        totals = categories.setdefault(categorize(path), {"files": 0, "code": 0, "comment": 0, "blank": 0})
        for key, value in count_file_loc(path).items():
            totals[key] += value
    return categories


def git_revision():
    result = subprocess.run(
        ["git", "-C", str(REPO_ROOT), "describe", "--always", "--dirty"],
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def write_summary(output_path, web, build, code):
    lines = []
    lines.append("IP Motorshield firmware metrics")
    lines.append(f"Generated: {datetime.datetime.now().isoformat(timespec='seconds')}")
    lines.append(f"Git revision: {git_revision()}")
    lines.append("")
    lines.append("Web page")
    lines.append(f"  web-page.html:        {web['html_bytes']} bytes")
    lines.append(f"  web-page.h (minified): {web['minified_bytes']} bytes "
                 f"({100 * web['minified_bytes'] / web['html_bytes']:.0f}% of original)")
    lines.append("")
    lines.append("Firmware build")
    lines.append(f"  Flash: {build['flash_bytes']} / {build['flash_max_bytes']} bytes ({build['flash_percent']}%)")
    lines.append(f"  RAM:   {build['ram_bytes']} / {build['ram_max_bytes']} bytes ({build['ram_percent']}%)")
    lines.append("")
    lines.append("C++ code lines (web-page.h excluded)")
    lines.append(f"  {'Category':<28} {'Files':>5} {'Code':>6} {'Comment':>8} {'Blank':>6}")
    total = {"files": 0, "code": 0, "comment": 0, "blank": 0}
    for name in ("Motorshield facade (src/)", "Web interface", "Other application code"):
        counts = code.get(name, {"files": 0, "code": 0, "comment": 0, "blank": 0})
        lines.append(f"  {name:<28} {counts['files']:>5} {counts['code']:>6} {counts['comment']:>8} {counts['blank']:>6}")
        for key in total:
            total[key] += counts[key]
    lines.append(f"  {'Total':<28} {total['files']:>5} {total['code']:>6} {total['comment']:>8} {total['blank']:>6}")
    lines.append("")

    output_path.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Build the firmware and write a metrics summary.")
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "scripts" / "metrics-summary.txt",
        help="summary output file (default: scripts/metrics-summary.txt)",
    )
    args = parser.parse_args()

    print("Rebuilding web-page.h ...")
    web = rebuild_web_page_header()
    print("Compiling firmware ...")
    build = compile_firmware()
    print("Counting code metrics ...")
    code = count_code_metrics()

    write_summary(args.output, web, build, code)
    print(f"Wrote {args.output}")
    print(args.output.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
