"""Generate embedded WLED UI gzip blobs from WLED v16.0.0 source files.

Downloads the real WLED v16.0.0 UI assets from GitHub (cached locally) and
embeds them as gzip PROGMEM arrays in wled_ui_data.cpp.

Served paths:
  /                → index.htm (text/html)
  /index.htm       → index.htm
  /index.js        → index.js  (application/javascript)
  /index.css       → index.css (text/css)
  /rangetouch.js   → rangetouch.js
  /iro.js          → iro.js
  /settings*       → custom "managed by ESPHome" stub page

Usage:
  python tools/gen_wled_ui.py            # download + generate
  python tools/gen_wled_ui.py --offline  # use cached files only
"""

from __future__ import annotations

import argparse
import gzip
import sys
import urllib.request
from pathlib import Path
from textwrap import dedent

WLED_TAG = "v16.0.0"
WLED_RAW = f"https://raw.githubusercontent.com/Aircoookie/WLED/{WLED_TAG}/wled00/data"

ROOT = Path(__file__).resolve().parents[1]
CACHE_DIR = ROOT / "tools" / "wled_ui_cache"
OUT = ROOT / "components" / "wled_bridge" / "wled_ui_data.cpp"
UI_ASSETS_DIR = ROOT / "components" / "wled_bridge" / "ui_assets"

# Files to download from WLED, mapped to (symbol_prefix, content_type)
WLED_FILES: list[tuple[str, str, str]] = [
    ("index.htm",      "WLED_INDEX",        "text/html"),
    ("index.js",       "WLED_INDEX_JS",     "application/javascript"),
    ("index.css",      "WLED_INDEX_CSS",    "text/css"),
    ("rangetouch.js",  "WLED_RANGETOUCH",   "application/javascript"),
    ("iro.js",         "WLED_IRO",          "application/javascript"),
]

# Custom settings redirect page served for /settings* paths
SETTINGS_HTML = dedent(
    """\
    <!DOCTYPE html>
    <html lang="en">
    <head>
      <meta charset="utf-8">
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>WLED Bridge — Settings</title>
      <style>
        body{font-family:system-ui,sans-serif;background:#0b0d10;color:#ecf2f8;
             display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0}
        .card{background:#15191f;border:1px solid #303946;border-radius:8px;
              padding:32px;max-width:480px;text-align:center}
        h1{font-size:20px;margin:0 0 12px}
        p{color:#9ba8b5;margin:0 0 20px;line-height:1.5}
        a{display:inline-block;background:#31c48d;color:#03100b;text-decoration:none;
          border-radius:6px;padding:10px 20px;font-weight:650}
      </style>
    </head>
    <body>
      <div class="card">
        <h1>Settings</h1>
        <p>Hardware, WiFi, LED pin, OTA, and security settings are managed by ESPHome YAML.
        To change these settings, edit your device&rsquo;s YAML configuration and recompile.</p>
        <a href="/">&#8592; Back to WLED Bridge</a>
      </div>
    </body>
    </html>
    """
).strip()


def fetch_file(filename: str, offline: bool) -> bytes:
    cache_path = CACHE_DIR / filename
    if cache_path.exists():
        data = cache_path.read_bytes()
        print(f"  {filename}: {len(data):,} bytes (cached)")
        return data
    if offline:
        print(f"ERROR: {filename} not in cache and --offline specified", file=sys.stderr)
        sys.exit(1)
    url = f"{WLED_RAW}/{filename}"
    print(f"  {filename}: downloading from {url} …", end=" ", flush=True)
    with urllib.request.urlopen(url, timeout=30) as resp:
        data = resp.read()
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    cache_path.write_bytes(data)
    print(f"{len(data):,} bytes")
    return data


def format_array(data: bytes) -> str:
    lines: list[str] = []
    for start in range(0, len(data), 19):
        chunk = data[start: start + 19]
        lines.append("    " + ", ".join(f"0x{b:02x}" for b in chunk) + ",")
    return "\n".join(lines)


def blob_section(symbol: str, data: bytes) -> str:
    compressed = gzip.compress(data, compresslevel=9, mtime=0)
    return (
        f"const uint8_t {symbol}_GZ[] = {{\n"
        f"{format_array(compressed)}\n"
        f"}};\n"
        f"const size_t {symbol}_GZ_SIZE = {len(compressed)};\n\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offline", action="store_true",
                        help="Use cached files only, do not download")
    args = parser.parse_args()

    print(f"Fetching WLED {WLED_TAG} UI assets …")
    sections: list[str] = []
    total_raw = 0
    total_gz = 0
    # raw_data: list of (filename, symbol, raw_bytes) — captured for .gz output below
    raw_data: list[tuple[str, str, bytes]] = []

    for filename, symbol, _ in WLED_FILES:
        raw = fetch_file(filename, args.offline)
        compressed = gzip.compress(raw, compresslevel=9, mtime=0)
        total_raw += len(raw)
        total_gz += len(compressed)
        sections.append(blob_section(symbol, raw))
        raw_data.append((filename, symbol, raw))
        print(f"    → {symbol}_GZ: {len(raw):,} raw → {len(compressed):,} gzip")

    # Settings redirect page
    settings_raw = SETTINGS_HTML.encode("utf-8")
    settings_gz = gzip.compress(settings_raw, compresslevel=9, mtime=0)
    total_raw += len(settings_raw)
    total_gz += len(settings_gz)
    sections.append(blob_section("WLED_SETTINGS", settings_raw))
    print(f"    → WLED_SETTINGS_GZ: {len(settings_raw):,} raw → {len(settings_gz):,} gzip")

    print(f"\nTotal: {total_raw:,} raw → {total_gz:,} gzip ({total_gz / total_raw * 100:.1f}%)")

    # Write individual .gz binary files for Python-side blob injection in __init__.py
    UI_ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    for filename, _symbol, raw in raw_data:
        gz_path = UI_ASSETS_DIR / (filename + ".gz")
        gz_data = gzip.compress(raw, compresslevel=9, mtime=0)
        gz_path.write_bytes(gz_data)
        print(f"  Written {gz_path.relative_to(ROOT)}")
    (UI_ASSETS_DIR / "settings.html.gz").write_bytes(settings_gz)
    print(f"  Written {(UI_ASSETS_DIR / 'settings.html.gz').relative_to(ROOT)}")

    source = (
        "// Auto-generated by tools/gen_wled_ui.py — do not edit manually.\n"
        f"// WLED source: {WLED_TAG}  (https://github.com/Aircoookie/WLED)\n"
        '#include "wled_ui_data.h"\n\n'
        "#ifdef USE_ESP32\n\n"
        "namespace esphome {\n"
        "namespace wled_bridge {\n\n"
        + "".join(sections)
        + "}  // namespace wled_bridge\n"
        "}  // namespace esphome\n\n"
        "#endif  // USE_ESP32\n"
    )
    OUT.write_text(source, encoding="utf-8")
    print(f"\nWritten → {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
