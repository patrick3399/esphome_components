"""Generate the embedded WLED bridge UI gzip blob."""

from __future__ import annotations

import gzip
from pathlib import Path
from textwrap import dedent


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "components" / "wled_bridge" / "wled_ui_data.cpp"


HTML = dedent(
    r"""
    <!doctype html>
    <html lang="en">
    <head>
      <meta charset="utf-8">
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>WLED Bridge</title>
      <style>
        :root {
          color-scheme: dark;
          --bg: #0b0d10;
          --panel: #15191f;
          --panel-2: #1c222b;
          --line: #303946;
          --text: #ecf2f8;
          --muted: #9ba8b5;
          --accent: #31c48d;
          --warn: #f6c445;
        }
        * { box-sizing: border-box; }
        body {
          margin: 0;
          background: var(--bg);
          color: var(--text);
          font: 14px/1.4 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
        }
        header {
          display: flex;
          align-items: center;
          gap: 12px;
          padding: 14px 18px;
          border-bottom: 1px solid var(--line);
          background: #0f1318;
          position: sticky;
          top: 0;
          z-index: 1;
        }
        h1 {
          font-size: 18px;
          font-weight: 650;
          margin: 0;
        }
        main {
          display: grid;
          grid-template-columns: minmax(260px, 360px) minmax(280px, 1fr);
          gap: 16px;
          padding: 16px;
          max-width: 980px;
          margin: 0 auto;
        }
        section {
          border: 1px solid var(--line);
          background: var(--panel);
          border-radius: 8px;
          padding: 14px;
        }
        section + section { margin-top: 16px; }
        h2 {
          font-size: 13px;
          color: var(--muted);
          letter-spacing: 0;
          text-transform: uppercase;
          margin: 0 0 12px;
        }
        label {
          display: grid;
          gap: 6px;
          margin: 12px 0;
          color: var(--muted);
        }
        .row {
          display: flex;
          align-items: center;
          gap: 10px;
        }
        .row > * { min-width: 0; }
        button, select, input[type="range"], input[type="number"], input[type="text"] {
          width: 100%;
        }
        button, select, input[type="number"], input[type="text"] {
          border: 1px solid var(--line);
          background: var(--panel-2);
          color: var(--text);
          border-radius: 6px;
          min-height: 38px;
          padding: 0 10px;
        }
        button {
          cursor: pointer;
          font-weight: 650;
        }
        button.primary {
          background: var(--accent);
          border-color: var(--accent);
          color: #03100b;
        }
        button.warn {
          background: transparent;
          color: var(--warn);
        }
        input[type="range"] { accent-color: var(--accent); }
        input[type="color"] {
          width: 44px;
          height: 38px;
          border: 1px solid var(--line);
          background: var(--panel-2);
          border-radius: 6px;
          padding: 2px;
        }
        .value {
          min-width: 34px;
          text-align: right;
          color: var(--text);
          font-variant-numeric: tabular-nums;
        }
        .status {
          color: var(--muted);
          font-size: 12px;
          min-height: 18px;
        }
        .stack { display: grid; gap: 10px; }
        .colors { display: grid; gap: 8px; }
        .preset-grid { display: grid; gap: 8px; }
        .color-row {
          display: grid;
          grid-template-columns: auto 1fr auto;
          align-items: center;
          gap: 10px;
        }
        .preset-row {
          display: grid;
          grid-template-columns: auto minmax(0, 1fr) auto auto auto auto;
          align-items: center;
          gap: 8px;
        }
        .preset-row strong { color: #f8fafc; }
        .hidden { display: none; }
        @media (max-width: 720px) {
          main { grid-template-columns: 1fr; padding: 12px; }
          header { padding: 12px; }
        }
      </style>
    </head>
    <body>
      <header>
        <h1 id="name">WLED Bridge</h1>
        <span class="status" id="status"></span>
      </header>
      <main>
        <div>
          <section>
            <h2>Power</h2>
            <div class="row">
              <button id="toggle" class="primary" type="button">On</button>
              <button id="refresh" type="button">Refresh</button>
            </div>
            <label>
              <span>Brightness</span>
              <div class="row">
                <input id="bri" type="range" min="0" max="255" value="128">
                <span class="value" id="briV">128</span>
              </div>
            </label>
            <label>
              <span>Transition</span>
              <div class="row">
                <input id="transition" type="range" min="0" max="10000" step="100" value="700">
                <span class="value" id="transitionV">700ms</span>
              </div>
            </label>
          </section>
          <section>
            <h2>Colors</h2>
            <div class="colors" id="colors"></div>
          </section>
          <section>
            <h2>Segment</h2>
            <label>
              <span>Start</span>
              <input id="segStart" type="number" min="0" step="1">
            </label>
            <label>
              <span>Stop</span>
              <input id="segStop" type="number" min="1" step="1">
            </label>
            <div class="stack">
              <label class="row">
                <input id="segRev" type="checkbox">
                <span>Reverse</span>
              </label>
              <label class="row">
                <input id="segMirror" type="checkbox">
                <span>Mirror</span>
              </label>
            </div>
          </section>
          <section>
            <h2>Presets</h2>
            <div class="preset-grid" id="presets"></div>
          </section>
        </div>
        <div>
          <section>
            <h2>Effect</h2>
            <label>
              <span>Mode</span>
              <select id="fx"></select>
            </label>
            <label>
              <span>Palette</span>
              <select id="pal"></select>
            </label>
            <div class="stack" id="sliders"></div>
            <div class="stack" id="checks"></div>
          </section>
          <section>
            <h2>Device</h2>
            <div class="status" id="device"></div>
          </section>
        </div>
      </main>
      <script>
        const $ = (id) => document.getElementById(id);
        const state = { info: {}, state: {}, eff: [], pal: [], presets: {} };
        let quiet = false;
        let postTimer = null;
        let liveVersion = -1;

        function rgbToHex(c) {
          const r = (c?.[0] ?? 0).toString(16).padStart(2, "0");
          const g = (c?.[1] ?? 0).toString(16).padStart(2, "0");
          const b = (c?.[2] ?? 0).toString(16).padStart(2, "0");
          return `#${r}${g}${b}`;
        }

        function hexToRgb(hex) {
          const n = parseInt(hex.slice(1), 16);
          return [(n >> 16) & 255, (n >> 8) & 255, n & 255, 0];
        }

        function effectName(meta) {
          return String(meta || "").split("@")[0];
        }

        function attr(text) {
          return String(text || "").replace(/[&"<>]/g, (ch) => ({
            "&": "&amp;",
            '"': "&quot;",
            "<": "&lt;",
            ">": "&gt;"
          }[ch]));
        }

        function metaLabels(meta) {
          const raw = String(meta || "");
          const after = raw.includes("@") ? raw.split("@")[1] : "";
          const [sliders = "", colors = "", palette = ""] = after.split(";");
          const names = sliders.split(",");
          const defaults = ["Speed", "Intensity", "Custom 1", "Custom 2", "Custom 3"];
          const visibleSliders = after ? names
            .slice(0, 5)
            .map((name, index) => ({ id: ["sx", "ix", "c1", "c2", "c3"][index], name, index }))
            .filter((item) => item.name || item.index < 2)
            .filter((item) => item.name !== "," && item.name !== ";") : [];
          const colorCount = colors
            ? Math.min(3, colors.split(",").filter((name) => name !== "").length)
            : (raw ? 1 : 0);
          return {
            sliders: visibleSliders.map((item) => ({
              id: item.id,
              label: item.name && item.name !== "!" ? item.name : defaults[item.index]
            })),
            colorCount,
            colors,
            palette
          };
        }

        function setStatus(text) {
          $("status").textContent = text || "";
        }

        async function api(path, options) {
          const res = await fetch(path, options);
          if (!res.ok) throw new Error(`${path} ${res.status}`);
          return await res.json();
        }

        async function refreshPresets() {
          state.presets = await api("/presets.json");
        }

        async function load() {
          try {
            setStatus("Loading");
            const [si, eff, pal] = await Promise.all([
              api("/json/si"),
              api("/json/eff"),
              api("/json/pal")
            ]);
            state.state = si.state || {};
            state.info = si.info || {};
            state.eff = eff || [];
            state.pal = pal || [];
            await refreshPresets();
            render();
            pollLive();
            setStatus("Ready");
          } catch (err) {
            setStatus(err.message);
          }
        }

        function renderOptions(select, values, formatter) {
          const current = select.value;
          select.innerHTML = "";
          values.forEach((value, index) => {
            const opt = document.createElement("option");
            opt.value = String(index);
            opt.textContent = formatter(value, index);
            select.appendChild(opt);
          });
          if (current) select.value = current;
        }

        function addSlider(parent, id, label, value) {
          const wrap = document.createElement("label");
          wrap.innerHTML =
            `<span>${label}</span><div class="row">` +
            `<input id="${id}" type="range" min="0" max="255" value="${value}">` +
            `<span class="value" id="${id}V">${value}</span></div>`;
          parent.appendChild(wrap);
          const input = $(id);
          input.addEventListener("input", () => {
            $(`${id}V`).textContent = input.value;
            schedulePost();
          });
        }

        function addCheck(parent, id, label, checked) {
          const row = document.createElement("label");
          row.className = "row";
          row.innerHTML = `<input id="${id}" type="checkbox" ${checked ? "checked" : ""}><span>${label}</span>`;
          parent.appendChild(row);
          $(id).addEventListener("change", schedulePost);
        }

        function render() {
          quiet = true;
          const s = state.state;
          const seg = (s.seg && s.seg[0]) || {};
          $("name").textContent = state.info.name || "WLED Bridge";
          $("toggle").textContent = s.on ? "On" : "Off";
          $("toggle").classList.toggle("primary", !!s.on);
          $("bri").value = s.bri ?? 128;
          $("briV").textContent = $("bri").value;
          const transitionMs = (s.tt ?? s.transition ?? 7) * 100;
          $("transition").value = transitionMs;
          $("transitionV").textContent = `${transitionMs}ms`;

          renderOptions($("fx"), state.eff, effectName);
          renderOptions($("pal"), state.pal, (name) => name);
          $("fx").value = String(seg.fx ?? 0);
          $("pal").value = String(seg.pal ?? 0);

          const labels = metaLabels(state.eff[seg.fx || 0]);
          const sliders = $("sliders");
          sliders.innerHTML = "";
          const sliderValues = { sx: seg.sx ?? 128, ix: seg.ix ?? 128, c1: seg.c1 ?? 128,
            c2: seg.c2 ?? 128, c3: seg.c3 ?? 16 };
          labels.sliders.forEach((slider) => {
            addSlider(sliders, slider.id, slider.label, sliderValues[slider.id]);
          });

          const checks = $("checks");
          checks.innerHTML = "";
          addCheck(checks, "o1", "Option 1", !!seg.o1);
          addCheck(checks, "o2", "Option 2", !!seg.o2);
          addCheck(checks, "o3", "Option 3", !!seg.o3);

          const colors = $("colors");
          colors.innerHTML = "";
          const cols = seg.col || [[255, 170, 0], [0, 0, 0], [0, 0, 0]];
          for (let i = 0; i < labels.colorCount; i++) {
            const row = document.createElement("div");
            row.className = "color-row";
            row.innerHTML =
              `<span>${i + 1}</span>` +
              `<input id="col${i}" type="color" value="${rgbToHex(cols[i])}">` +
              `<button id="colApply${i}" type="button">Set</button>`;
            colors.appendChild(row);
            $(`col${i}`).addEventListener("input", schedulePost);
            $(`colApply${i}`).addEventListener("click", postNow);
          }
          const ledCount = state.info.leds?.count ?? seg.stop ?? 1;
          $("segStart").max = Math.max(0, ledCount - 1);
          $("segStop").max = ledCount;
          $("segStart").value = seg.start ?? 0;
          $("segStop").value = seg.stop ?? ledCount;
          $("segRev").checked = !!seg.rev;
          $("segMirror").checked = !!seg.mi;

          const ledInfo = state.info.leds || {};
          $("device").textContent = [
            `${state.info.product || ""} ${state.info.ver || ""}`.trim(),
            `${ledInfo.count ?? 0} LEDs`,
            `${ledInfo.fps ?? 0} FPS`,
            `${ledInfo.pwr ?? 0}/${ledInfo.maxpwr ?? 0} mA`
          ].filter(Boolean).join(" / ");
          renderPresets(s);
          quiet = false;
        }

        function renderPresets(s) {
          const wrap = $("presets");
          const presets = s.presets || Array.from({ length: 16 }, () => false);
          wrap.innerHTML = "";
          presets.forEach((valid, index) => {
            const id = index + 1;
            const row = document.createElement("div");
            row.className = "preset-row";
            const preset = state.presets[String(id)] || {};
            const presetName = preset.n || `Preset ${id}`;
            row.innerHTML =
              `<strong>${id}</strong>` +
              `<input id="presetName${id}" type="text" maxlength="23" value="${attr(presetName)}">` +
              `<span>${valid ? (s.ps === id ? "Active" : "Saved") : "Empty"}</span>` +
              `<button id="presetLoad${id}" type="button" ${valid ? "" : "disabled"}>Load</button>` +
              `<button id="presetSave${id}" type="button">Save</button>`;
            wrap.appendChild(row);
            $(`presetLoad${id}`).addEventListener("click", () => postPreset("ps", id));
            $(`presetSave${id}`).addEventListener("click", () => postPreset("psave", id));
            if (valid) {
              const del = document.createElement("button");
              del.id = `presetDelete${id}`;
              del.type = "button";
              del.textContent = "Delete";
              row.appendChild(del);
              del.addEventListener("click", () => postPreset("pdel", id));
            }
          });
        }

        function numberValue(id, fallback) {
          const el = $(id);
          return el ? Number(el.value) : fallback;
        }

        function collectPayload() {
          const seg = (state.state.seg && state.state.seg[0]) || {};
          const currentCols = seg.col || [[255, 170, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
          const col = [0, 1, 2].map((i) => {
            const el = $(`col${i}`);
            return el ? hexToRgb(el.value) : currentCols[i];
          });
          return {
            on: $("toggle").classList.contains("primary"),
            bri: Number($("bri").value),
            tt: Math.round(Number($("transition").value) / 100),
            seg: [{
              id: 0,
              fx: Number($("fx").value),
              pal: Number($("pal").value),
              sx: numberValue("sx", seg.sx ?? 128),
              ix: numberValue("ix", seg.ix ?? 128),
              c1: numberValue("c1", seg.c1 ?? 128),
              c2: numberValue("c2", seg.c2 ?? 128),
              c3: numberValue("c3", seg.c3 ?? 16),
              o1: $("o1").checked,
              o2: $("o2").checked,
              o3: $("o3").checked,
              start: numberValue("segStart", seg.start ?? 0),
              stop: numberValue("segStop", seg.stop ?? state.info.leds?.count ?? 1),
              rev: $("segRev").checked,
              mi: $("segMirror").checked,
              col
            }]
          };
        }

        function schedulePost() {
          if (quiet) return;
          clearTimeout(postTimer);
          postTimer = setTimeout(postNow, 140);
        }

        async function postNow() {
          if (quiet) return;
          clearTimeout(postTimer);
          try {
            setStatus("Saving");
            const updated = await api("/json/state", {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify(collectPayload())
            });
            state.state = updated;
            liveVersion = -1;
            render();
            setStatus("Ready");
          } catch (err) {
            setStatus(err.message);
          }
        }

        async function postPreset(action, id) {
          clearTimeout(postTimer);
          try {
            setStatus("Saving");
            const payload = action === "psave" ? collectPayload() : {};
            payload[action] = id;
            if (action === "psave") payload.n = $(`presetName${id}`).value;
            const updated = await api("/json/state", {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify(payload)
            });
            state.state = updated;
            await refreshPresets();
            liveVersion = -1;
            render();
            setStatus("Ready");
          } catch (err) {
            setStatus(err.message);
          }
        }

        async function pollLive() {
          if (postTimer || quiet) return;
          try {
            const live = await api("/wled_events");
            if (live.version !== liveVersion) {
              liveVersion = live.version;
              state.state = live.state || state.state;
              render();
            }
          } catch (err) {
            setStatus(err.message);
          }
        }

        $("toggle").addEventListener("click", () => {
          $("toggle").classList.toggle("primary");
          $("toggle").textContent = $("toggle").classList.contains("primary") ? "On" : "Off";
          postNow();
        });
        $("refresh").addEventListener("click", load);
        $("bri").addEventListener("input", () => {
          $("briV").textContent = $("bri").value;
          schedulePost();
        });
        $("transition").addEventListener("input", () => {
          $("transitionV").textContent = `${$("transition").value}ms`;
          schedulePost();
        });
        $("fx").addEventListener("change", schedulePost);
        $("pal").addEventListener("change", schedulePost);
        $("segStart").addEventListener("change", schedulePost);
        $("segStop").addEventListener("change", schedulePost);
        $("segRev").addEventListener("change", schedulePost);
        $("segMirror").addEventListener("change", schedulePost);
        load();
        setInterval(pollLive, 1500);
      </script>
    </body>
    </html>
    """
).strip()


def format_array(data: bytes) -> str:
    lines: list[str] = []
    for start in range(0, len(data), 16):
        chunk = data[start : start + 16]
        lines.append("    " + ", ".join(f"0x{byte:02x}" for byte in chunk) + ",")
    return "\n".join(lines)


def main() -> None:
    compressed = gzip.compress(HTML.encode("utf-8"), compresslevel=9, mtime=0)
    source = (
        "// Auto-generated by tools/gen_wled_ui.py -- do not edit manually\n"
        '#include "wled_ui_data.h"\n\n'
        "namespace esphome {\n"
        "namespace wled_bridge {\n\n"
        "const uint8_t WLED_INDEX_GZ[] = {\n"
        f"{format_array(compressed)}\n"
        "};\n"
        f"const size_t WLED_INDEX_GZ_SIZE = {len(compressed)};\n\n"
        "}  // namespace wled_bridge\n"
        "}  // namespace esphome\n"
    )
    OUT.write_text(source, encoding="utf-8")


if __name__ == "__main__":
    main()
