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
          --bg: #0b0d10; --panel: #15191f; --panel-2: #1c222b;
          --line: #303946; --text: #ecf2f8; --muted: #9ba8b5;
          --accent: #31c48d; --warn: #f6c445;
        }
        * { box-sizing: border-box; }
        body { margin: 0; background: var(--bg); color: var(--text); font: 14px/1.4 system-ui,-apple-system,"Segoe UI",sans-serif; }
        header { display: flex; align-items: center; gap: 12px; padding: 14px 18px; border-bottom: 1px solid var(--line); background: #0f1318; position: sticky; top: 0; z-index: 1; }
        h1 { font-size: 18px; font-weight: 650; margin: 0; }
        main { display: grid; grid-template-columns: minmax(260px,360px) minmax(280px,1fr); gap: 16px; padding: 16px; max-width: 980px; margin: 0 auto; }
        section { border: 1px solid var(--line); background: var(--panel); border-radius: 8px; padding: 14px; }
        section + section { margin-top: 16px; }
        h2 { font-size: 13px; color: var(--muted); text-transform: uppercase; margin: 0 0 12px; display: flex; align-items: center; justify-content: space-between; }
        label { display: grid; gap: 6px; margin: 12px 0; color: var(--muted); }
        .row { display: flex; align-items: center; gap: 10px; }
        .row > * { min-width: 0; }
        button, select, input[type="range"], input[type="number"], input[type="text"] { width: 100%; }
        button, select, input[type="number"], input[type="text"] { border: 1px solid var(--line); background: var(--panel-2); color: var(--text); border-radius: 6px; min-height: 38px; padding: 0 10px; }
        button { cursor: pointer; font-weight: 650; }
        button.primary { background: var(--accent); border-color: var(--accent); color: #03100b; }
        button.warn { background: transparent; color: var(--warn); border-color: transparent; }
        button.sm { min-height: 26px; font-size: 12px; padding: 0 8px; width: auto; }
        input[type="range"] { accent-color: var(--accent); }
        input[type="color"] { width: 44px; height: 38px; border: 1px solid var(--line); background: var(--panel-2); border-radius: 6px; padding: 2px; }
        .value { min-width: 34px; text-align: right; color: var(--text); font-variant-numeric: tabular-nums; }
        .status { color: var(--muted); font-size: 12px; min-height: 18px; }
        .stack { display: grid; gap: 10px; }
        .colors { display: grid; gap: 8px; }
        .color-row { display: grid; grid-template-columns: auto 1fr auto; align-items: center; gap: 10px; }
        .preset-row { display: grid; grid-template-columns: auto minmax(0,1fr) auto auto auto auto; align-items: center; gap: 8px; }
        .preset-row strong { color: #f8fafc; }
        .seg-row { display: flex; align-items: center; gap: 8px; padding: 6px 8px; border-radius: 6px; cursor: pointer; border: 1px solid transparent; margin-bottom: 4px; }
        .seg-row:hover { background: var(--panel-2); }
        .seg-row.active { border-color: var(--accent); background: var(--panel-2); }
        .seg-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--accent); flex-shrink: 0; }
        .seg-dot.off { background: var(--line); }
        .seg-info { flex: 1; font-size: 13px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
        .seg-range { color: var(--muted); font-size: 11px; }
        .seg-editor { border-top: 1px solid var(--line); margin-top: 8px; padding-top: 10px; }
        canvas.peek { width: 100%; height: 32px; border-radius: 4px; display: block; image-rendering: pixelated; background: #000; }
        @media (max-width: 720px) { main { grid-template-columns: 1fr; padding: 12px; } header { padding: 12px; } }
      </style>
    </head>
    <body>
      <header>
        <h1 id="name">WLED Bridge</h1>
        <span class="status" id="status"></span>
      </header>
      <main>
        <div>
          <!-- Power -->
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
          <!-- Colors -->
          <section>
            <h2>Colors</h2>
            <div class="colors" id="colors"></div>
          </section>
          <!-- Segments -->
          <section>
            <h2>Segments <button class="sm" id="addSeg" type="button">+ Add</button></h2>
            <div id="segList"></div>
            <div class="seg-editor" id="segEditor" style="display:none">
              <label>
                <span>Start</span>
                <input id="segStart" type="number" min="0" step="1">
              </label>
              <label>
                <span>Stop</span>
                <input id="segStop" type="number" min="1" step="1">
              </label>
              <div class="stack">
                <label class="row"><input id="segOn" type="checkbox" checked><span>On</span></label>
                <label class="row"><input id="segRev" type="checkbox"><span>Reverse</span></label>
                <label class="row"><input id="segMirror" type="checkbox"><span>Mirror</span></label>
              </div>
            </div>
          </section>
          <!-- Presets -->
          <section>
            <h2>Presets</h2>
            <div id="presets"></div>
          </section>
          <!-- Timer -->
          <section>
            <h2>Timer</h2>
            <div class="row" style="margin-bottom:10px">
              <button id="nlToggle" type="button">Off</button>
            </div>
            <label>
              <span>Duration (minutes)</span>
              <div class="row">
                <input id="nlDur" type="number" min="1" max="255" value="1" style="max-width:80px">
              </div>
            </label>
            <label>
              <span>Mode</span>
              <select id="nlMode">
                <option value="0">Instant</option>
                <option value="1" selected>Fade</option>
                <option value="2">Fade + Color</option>
                <option value="3">Sunrise</option>
              </select>
            </label>
            <label>
              <span>Target Brightness</span>
              <div class="row">
                <input id="nlTbri" type="range" min="0" max="255" value="0">
                <span class="value" id="nlTbriV">0</span>
              </div>
            </label>
          </section>
        </div>
        <div>
          <!-- Effect -->
          <section>
            <h2>Effect</h2>
            <label><span>Mode</span><select id="fx"></select></label>
            <label><span>Palette</span><select id="pal"></select></label>
            <div class="stack" id="sliders"></div>
            <div class="stack" id="checks"></div>
          </section>
          <!-- Peek -->
          <section>
            <h2>Peek</h2>
            <canvas id="peek" class="peek" width="1" height="1"></canvas>
            <div class="status" id="peekInfo" style="margin-top:4px"></div>
          </section>
          <!-- Device -->
          <section>
            <h2>Device</h2>
            <div class="status" id="device"></div>
          </section>
        </div>
      </main>
      <script>
        const $ = id => document.getElementById(id);
        const st = { info: {}, state: {}, eff: [], fxdata: [], pal: [], presets: {} };
        let quiet = false, postTimer = null, liveVersion = -1;
        let activeSeg = 0, peekTimer = null;

        const rgbToHex = c => "#" + [c?.[0]??0, c?.[1]??0, c?.[2]??0]
          .map(v => v.toString(16).padStart(2,"0")).join("");
        const hexToRgb = hex => { const n = parseInt(hex.slice(1),16); return [(n>>16)&255,(n>>8)&255,n&255,0]; };
        const effectName = s => String(s||"").split("@")[0];
        const attr = s => String(s||"").replace(/[&"<>]/g, c => ({"&":"&amp;",'"':"&quot;","<":"&lt;",">":"&gt;"}[c]));

        // Parse WLED fxdata metadata string (part after '@' in effect descriptor).
        // fxdata format: "SpeedLabel,IntLabel,C1,C2,C3;Col0,Col1,Col2;PalFlag;Flags"
        // Empty slot = hidden; "!" = show with default label.
        function metaLabels(fxmeta) {
          const meta = String(fxmeta || "");
          const parts = meta.split(";");
          const sliderStr = parts[0] || "";
          const colorStr  = parts[1] || "";
          const names = sliderStr ? sliderStr.split(",") : [];
          const defaults = ["Speed","Intensity","Custom 1","Custom 2","Custom 3"];
          const ids = ["sx","ix","c1","c2","c3"];
          const visibleSliders = [];
          for (let i = 0; i < 5; i++) {
            const name = i < names.length ? names[i] : "";
            // Speed (0) and Intensity (1) always visible; Custom (2-4) only if explicitly named.
            if (i < 2) {
              visibleSliders.push({ id: ids[i], label: (name && name !== "!") ? name : defaults[i] });
            } else if (name && name !== "!") {
              visibleSliders.push({ id: ids[i], label: name });
            }
          }
          const sliders = visibleSliders;
          const colorParts = colorStr ? colorStr.split(",").filter(n => n !== "") : [];
          const colorCount = colorParts.length > 0 ? Math.min(3, colorParts.length) : 1;
          return { sliders, colorCount };
        }

        const setStatus = t => $("status").textContent = t || "";

        async function api(path, opts) {
          const r = await fetch(path, opts);
          if (!r.ok) throw new Error(path + " " + r.status);
          return r.json();
        }

        async function refreshPresets() { st.presets = await api("/presets.json"); }

        async function load() {
          try {
            setStatus("Loading");
            const [si, eff, fxdata, pal] = await Promise.all([
              api("/json/si"), api("/json/eff"), api("/json/fxdata"), api("/json/pal")
            ]);
            st.state = si.state || {};
            st.info  = si.info  || {};
            st.eff    = eff    || [];
            st.fxdata = fxdata || [];
            st.pal    = pal    || [];
            const n = (st.state.seg || []).length;
            if (activeSeg >= n) activeSeg = 0;
            await refreshPresets();
            render();
            pollLive();
            schedulePeek();
            setStatus("Ready");
          } catch(e) { setStatus(e.message); }
        }

        const currentSeg = () => { const s = st.state.seg || []; return s[activeSeg] || s[0] || {}; };

        function renderOptions(sel, vals, fmt) {
          const cur = sel.value;
          sel.innerHTML = "";
          vals.forEach((v,i) => {
            const o = document.createElement("option");
            o.value = String(i); o.textContent = fmt(v,i);
            sel.appendChild(o);
          });
          if (cur) sel.value = cur;
        }

        function addSlider(parent, id, label, value) {
          const w = document.createElement("label");
          w.innerHTML = `<span>${label}</span><div class="row">` +
            `<input id="${id}" type="range" min="0" max="255" value="${value}">` +
            `<span class="value" id="${id}V">${value}</span></div>`;
          parent.appendChild(w);
          $(id).addEventListener("input", () => { $(`${id}V`).textContent = $(id).value; schedulePost(); });
        }

        function addCheck(parent, id, label, checked) {
          const r = document.createElement("label");
          r.className = "row";
          r.innerHTML = `<input id="${id}" type="checkbox"${checked?" checked":""}><span>${label}</span>`;
          parent.appendChild(r);
          $(id).addEventListener("change", schedulePost);
        }

        function buildEffectSliders(fxIdx, segData) {
          const labels = metaLabels(st.fxdata[fxIdx] || "");
          const sv = { sx:segData.sx??128, ix:segData.ix??128, c1:segData.c1??128, c2:segData.c2??128, c3:segData.c3??16 };
          const sliders = $("sliders");
          sliders.innerHTML = "";
          labels.sliders.forEach(s => addSlider(sliders, s.id, s.label, sv[s.id]));
          const checks = $("checks");
          checks.innerHTML = "";
          addCheck(checks, "o1", "Option 1", !!segData.o1);
          addCheck(checks, "o2", "Option 2", !!segData.o2);
          addCheck(checks, "o3", "Option 3", !!segData.o3);
          return labels.colorCount;
        }

        function buildColorPickers(colorCount, segData) {
          const colors = $("colors");
          colors.innerHTML = "";
          const cols = segData.col || [[255,170,0],[0,0,0],[0,0,0]];
          for (let i = 0; i < colorCount; i++) {
            const row = document.createElement("div");
            row.className = "color-row";
            row.innerHTML = `<span>${i+1}</span>` +
              `<input id="col${i}" type="color" value="${rgbToHex(cols[i])}">` +
              `<button id="colApply${i}" type="button">Set</button>`;
            colors.appendChild(row);
            $(`col${i}`).addEventListener("input", schedulePost);
            $(`colApply${i}`).addEventListener("click", postNow);
          }
        }

        function renderSegList() {
          const segs = st.state.seg || [];
          const list = $("segList");
          list.innerHTML = "";
          segs.forEach((seg, i) => {
            const row = document.createElement("div");
            row.className = "seg-row" + (i === activeSeg ? " active" : "");
            const fn = effectName(st.eff[seg.fx] || "");
            const ledCount = st.info.leds?.count ?? 0;
            row.innerHTML =
              `<span class="seg-dot${seg.on===false?" off":""}"></span>` +
              `<span class="seg-info">Seg ${i} ` +
              `<span class="seg-range">${seg.start??0}–${seg.stop??ledCount}</span>` +
              `${fn ? " · "+fn : ""}</span>` +
              (i > 0 ? `<button class="sm warn" id="dSeg${i}" type="button">×</button>` : "");
            row.addEventListener("click", e => {
              if (e.target.closest(`#dSeg${i}`)) return;
              activeSeg = i;
              quiet = true;
              renderSegList();
              renderSegEditor();
              renderEffectAndColor();
              quiet = false;
            });
            list.appendChild(row);
            if (i > 0) $(`dSeg${i}`).addEventListener("click", () => doDeleteSeg(i));
          });
          $("segEditor").style.display = segs.length > 0 ? "" : "none";
        }

        function renderSegEditor() {
          const seg = currentSeg();
          const ledCount = st.info.leds?.count ?? 1;
          $("segStart").max = Math.max(0, ledCount - 1);
          $("segStop").max  = ledCount;
          $("segStart").value = seg.start ?? 0;
          $("segStop").value  = seg.stop  ?? ledCount;
          $("segOn").checked     = seg.on !== false;
          $("segRev").checked    = !!seg.rev;
          $("segMirror").checked = !!seg.mi;
        }

        function renderEffectAndColor() {
          const seg = currentSeg();
          renderOptions($("fx"),  st.eff, effectName);
          renderOptions($("pal"), st.pal, n => n);
          $("fx").value  = String(seg.fx  ?? 0);
          $("pal").value = String(seg.pal ?? 0);
          const count = buildEffectSliders(seg.fx ?? 0, seg);
          buildColorPickers(count, seg);
        }

        function renderNightlight() {
          const nl = st.state.nl || {};
          $("nlToggle").textContent = nl.on ? "Active" : "Off";
          $("nlToggle").classList.toggle("primary", !!nl.on);
          if (nl.dur  !== undefined) $("nlDur").value  = nl.dur;
          if (nl.mode !== undefined) $("nlMode").value = String(nl.mode);
          if (nl.tbri !== undefined) { $("nlTbri").value = nl.tbri; $("nlTbriV").textContent = nl.tbri; }
        }

        function renderPresets(s) {
          const wrap = $("presets");
          const valid = s.presets || Array.from({ length: 16 }, () => false);
          wrap.innerHTML = "";
          valid.forEach((v, idx) => {
            const id = idx + 1;
            const row = document.createElement("div");
            row.className = "preset-row";
            const presetName = (st.presets[String(id)] || {}).n || `Preset ${id}`;
            row.innerHTML = `<strong>${id}</strong>` +
              `<input id="presetName${id}" type="text" maxlength="23" value="${attr(presetName)}">` +
              `<span>${v?(s.ps===id?"Active":"Saved"):"Empty"}</span>` +
              `<button id="presetLoad${id}" type="button"${v?"":" disabled"}>Load</button>` +
              `<button id="presetSave${id}" type="button">Save</button>`;
            wrap.appendChild(row);
            $(`presetLoad${id}`).addEventListener("click", () => postPreset("ps",    id));
            $(`presetSave${id}`).addEventListener("click", () => postPreset("psave", id));
            if (v) {
              const d = document.createElement("button");
              d.id = `presetDelete${id}`; d.type = "button"; d.textContent = "Delete";
              row.appendChild(d);
              d.addEventListener("click", () => postPreset("pdel", id));
            }
          });
        }

        function render() {
          quiet = true;
          const s = st.state;
          $("name").textContent = st.info.name || "WLED Bridge";
          $("toggle").textContent = s.on ? "On" : "Off";
          $("toggle").classList.toggle("primary", !!s.on);
          $("bri").value = s.bri ?? 128; $("briV").textContent = $("bri").value;
          const tMs = (s.tt ?? s.transition ?? 7) * 100;
          $("transition").value = tMs; $("transitionV").textContent = tMs + "ms";
          const n = (s.seg||[]).length;
          if (activeSeg >= n) activeSeg = 0;
          renderSegList();
          renderSegEditor();
          renderEffectAndColor();
          renderNightlight();
          const li = st.info.leds || {};
          $("device").textContent = [
            `${st.info.product||""} ${st.info.ver||""}`.trim(),
            `${li.count??0} LEDs`, `${li.fps??0} FPS`, `${li.pwr??0}/${li.maxpwr??0} mA`
          ].filter(Boolean).join(" / ");
          renderPresets(s);
          quiet = false;
        }

        function numberValue(id, fallback) { const e = $(id); return e ? Number(e.value) : fallback; }

        function collectPayload() {
          const seg = currentSeg();
          const cols = seg.col || [[255,170,0,0],[0,0,0,0],[0,0,0,0]];
          const col = [0,1,2].map(i => { const e=$(`col${i}`); return e ? hexToRgb(e.value) : cols[i]; });
          const ledCount = st.info.leds?.count ?? 1;
          return {
            on: $("toggle").classList.contains("primary"),
            bri: Number($("bri").value),
            tt: Math.round(Number($("transition").value) / 100),
            seg: [{
              id:  activeSeg,
              fx:  Number($("fx").value),
              pal: Number($("pal").value),
              sx:  numberValue("sx",  seg.sx ??128),
              ix:  numberValue("ix",  seg.ix ??128),
              c1:  numberValue("c1",  seg.c1 ??128),
              c2:  numberValue("c2",  seg.c2 ??128),
              c3:  numberValue("c3",  seg.c3 ??16),
              o1:  !!($("o1")?.checked),
              o2:  !!($("o2")?.checked),
              o3:  !!($("o3")?.checked),
              start: numberValue("segStart", seg.start ?? 0),
              stop:  numberValue("segStop",  seg.stop  ?? ledCount),
              on:  $("segOn")?.checked !== false,
              rev: !!$("segRev")?.checked,
              mi:  !!$("segMirror")?.checked,
              col
            }]
          };
        }

        const schedulePost = () => { if (quiet) return; clearTimeout(postTimer); postTimer = setTimeout(postNow, 140); };

        async function postNow() {
          if (quiet) return;
          clearTimeout(postTimer);
          try {
            setStatus("Saving");
            const updated = await api("/json/state", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(collectPayload()) });
            st.state = updated; liveVersion = -1;
            render(); updatePeek(); setStatus("Ready");
          } catch(e) { setStatus(e.message); }
        }

        async function postPreset(action, id) {
          clearTimeout(postTimer);
          try {
            setStatus("Saving");
            const payload = action === "psave" ? collectPayload() : {};
            payload[action] = id;
            if (action === "psave") payload.n = $(`presetName${id}`).value;
            const updated = await api("/json/state", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(payload) });
            st.state = updated;
            await refreshPresets();
            liveVersion = -1; render(); setStatus("Ready");
          } catch(e) { setStatus(e.message); }
        }

        async function doAddSeg() {
          const ledCount = st.info.leds?.count ?? 60;
          const newId = (st.state.seg || []).length;
          try {
            setStatus("Adding segment");
            const updated = await api("/json/state", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({seg:[{id:newId, start:0, stop:Math.min(60,ledCount)}]}) });
            st.state = updated; activeSeg = Math.min(newId, (updated.seg||[]).length - 1);
            render(); setStatus("Ready");
          } catch(e) { setStatus(e.message); }
        }

        async function doDeleteSeg(id) {
          try {
            setStatus("Removing segment");
            const updated = await api("/json/state", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({seg:[{id, start:0, stop:0}]}) });
            st.state = updated;
            if (activeSeg >= id) activeSeg = Math.max(0, id - 1);
            render(); setStatus("Ready");
          } catch(e) { setStatus(e.message); }
        }

        async function toggleNightlight() {
          const nl = st.state.nl || {};
          const on = !nl.on;
          try {
            setStatus("Saving");
            const updated = await api("/json/state", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({nl:{on, dur:Number($("nlDur").value)||1, mode:Number($("nlMode").value), tbri:Number($("nlTbri").value)}}) });
            st.state = updated; renderNightlight(); setStatus("Ready");
          } catch(e) { setStatus(e.message); }
        }

        async function updatePeek() {
          try {
            const live = await api("/json/live");
            const leds = live.leds || [];
            const canvas = $("peek");
            canvas.width = Math.max(1, leds.length); canvas.height = 1;
            const ctx = canvas.getContext("2d");
            leds.forEach((hex, i) => { ctx.fillStyle = "#"+hex; ctx.fillRect(i,0,1,1); });
            const n = live.n || 1;
            $("peekInfo").textContent = leds.length + " LEDs shown" + (n > 1 ? " (1:"+n+" sampling)" : "");
          } catch(e) { $("peekInfo").textContent = e.message; }
        }

        function schedulePeek() {
          clearTimeout(peekTimer);
          peekTimer = setTimeout(async () => { await updatePeek(); schedulePeek(); }, 500);
        }

        async function pollLive() {
          if (postTimer || quiet) return;
          try {
            const live = await api("/wled_events");
            if (live.version !== liveVersion) {
              liveVersion = live.version;
              st.state = live.state || st.state;
              const n = (st.state.seg||[]).length;
              if (activeSeg >= n) activeSeg = 0;
              render();
            }
          } catch(e) { setStatus(e.message); }
        }

        // Wire up static controls
        $("toggle").addEventListener("click", () => {
          $("toggle").classList.toggle("primary");
          $("toggle").textContent = $("toggle").classList.contains("primary") ? "On" : "Off";
          postNow();
        });
        $("refresh").addEventListener("click", load);
        $("bri").addEventListener("input", () => { $("briV").textContent = $("bri").value; schedulePost(); });
        $("transition").addEventListener("input", () => { $("transitionV").textContent = $("transition").value+"ms"; schedulePost(); });
        $("fx").addEventListener("change", () => {
          if (!quiet) {
            const fxIdx = Number($("fx").value);
            const seg = currentSeg();
            const count = buildEffectSliders(fxIdx, seg);
            buildColorPickers(count, seg);
          }
          schedulePost();
        });
        $("pal").addEventListener("change", schedulePost);
        $("segStart").addEventListener("change",  schedulePost);
        $("segStop").addEventListener("change",   schedulePost);
        $("segOn").addEventListener("change",     schedulePost);
        $("segRev").addEventListener("change",    schedulePost);
        $("segMirror").addEventListener("change", schedulePost);
        $("addSeg").addEventListener("click", doAddSeg);
        $("nlToggle").addEventListener("click", toggleNightlight);
        $("nlTbri").addEventListener("input", () => { $("nlTbriV").textContent = $("nlTbri").value; });

        load();
        setInterval(pollLive, 1500);
      </script>
    </body>
    </html>
    """
).strip()


def format_array(data: bytes) -> str:
    lines: list[str] = []
    for start in range(0, len(data), 19):
        chunk = data[start : start + 19]
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
    print(f"Written {len(compressed)} bytes compressed ({len(HTML)} uncompressed) → {OUT}")


if __name__ == "__main__":
    main()
