# Manufacturing

How to reproduce Group 7's 3D-printed parts on a **Bambu Lab P2S** using **Bambu Studio** default settings with **tree supports**.

## 1. Purpose & scope

This doc covers only the operator workflow for printing the `.3mf` files already in `hardware/`. It does **not** cover design rationale, version history, materials selection, or assembly — those live in [`../hardware/README.md`](../hardware/README.md).

Out of scope: custom slicer tuning, non-Bambu printers, filament other than Bambu PLA Basic, and post-print assembly (rack-pinion alignment, washer count, screw torque).

## 2. Required equipment

| Item | Spec / Notes |
|---|---|
| 3D Printer | Bambu Lab P2S (base or Combo) — 256 × 256 × 256 mm build volume, 0.4 mm hotend, textured PEI plate |
| Slicer | Bambu Studio, latest stable |
| Filament | Bambu PLA Basic (bundled preset). If using another PLA, re-select the filament profile but keep the process profile defaults |
| Plate | Textured PEI (shipped with the P2S) |
| Tools | Flush cutters, flexible scraper for PEI, small flathead for tree-branch removal |

Printer reference: <https://bambulab.com/en-us/p2s>
Slicer download: <https://bambulab.com/en/download/studio>

## 3. Parts list

All paths are relative to the repo root. **Current = part of the v2.2.0 deployed assembly. Legacy = earlier iteration kept for reference; do not print unless rebuilding a historical revision.**

### Chassis (current)

| File | Path | Subassembly | CAD source |
|---|---|---|---|
| `launcher mount v2.3mf` | `hardware/chassis/mounts/launcher mount v2.3mf` | Launcher-to-waffle-plate bracket (v2) | `launcher mount v2.SLDPRT` (same dir) |
| `RPI camera mount.3mf` | `hardware/chassis/3mf/RPI camera mount.3mf` | RPi camera mount (top deck) | — (no SLDPRT in chassis tree; v1 variant at `launcher/v1/launcher assembly/RPI camera mount.SLDPRT`) |
| `mount to plate bottom.3mf` | `hardware/chassis/3mf/mount to plate bottom.3mf` | Bottom mounting plate interface | — |
| `wire holder (end).3mf` | `hardware/chassis/3mf/wire holder (end).3mf` | Cable-management end-cap | — |

### Chassis (legacy, do not print for the current build)

| File | Path | Superseded by |
|---|---|---|
| `launcher mount.3mf` | `hardware/chassis/3mf/launcher mount.3mf` | `launcher mount v2.3mf` (see above) |

### Launcher (current — v2.2.0 deployed)

| File | Path | Subassembly | CAD source |
|---|---|---|---|
| `Plunger face.3mf` | `hardware/launcher/3mf/Plunger face.3mf` | Plunger face (ball-contact side; M3 bolted to rack) — v2.1.0+ split | — |
| `Plunger rack.3mf` | `hardware/launcher/3mf/Plunger rack.3mf` | Rack half of rack-and-pinion plunger — v2.1.0+ split | — |
| `barrel2storage.3mf` | `hardware/launcher/3mf/barrel2storage.3mf` | Barrel-to-storage-tube adapter | `launcher/v1/launcher assembly/barrel2storage.SLDPRT` |
| `spur gear for servo.3mf` | `hardware/launcher/3mf/spur gear for servo.3mf` | MG90 servo pinion (no clutch bearing; use for v2.0.0 / testing builds) (unconfirmed whether superseded by bearing variant in the deployed stack) | `launcher/v1/launcher assembly/spur gear for servo.SLDPRT` |
| `spur gear for servo with bearing.3mf` | `hardware/launcher/components/spur gear for servo with bearing.3mf` | Pinion with internal clutch-bearing pocket — **v2.2.0 deployed variant** | `components/spur gear for servo with bearing.SLDPRT` |

Additional launcher CAD (SLDPRT only, no `.3mf` in repo): `Barrel guide`, `Carousell`, `Clamp1/2`, `Motor Clamp`, `Part 1-14`, `Part 2-12`, `Part 3`, `Rotor` — all under `hardware/launcher/components/`. Their print status is **unconfirmed**; export STL/3MF from SolidWorks if you need them.

### Launcher (legacy v1.0.0)

Under `hardware/launcher/v1/launcher assembly/`. Kept for historical reference. Print **only** to reproduce the v1.0.0 side-mount configuration documented in [`../hardware/README.md#version-100---first-design`](../hardware/README.md#version-100---first-design).

| File | Path |
|---|---|
| `Plunger.3mf` | `hardware/launcher/v1/launcher assembly/Plunger.3mf` |
| `Tube.3mf` | `hardware/launcher/v1/launcher assembly/Tube.3mf` |
| `launcher assembly.3mf` | `hardware/launcher/v1/launcher assembly/launcher assembly.3mf` |

Versioned archives (`Launcher assembly 1.0.0.zip`, `Launcher assembly 1.1.0.zip`, `mounts 1.0.0.zip`) contain CAD snapshots for those releases and are not needed for printing.

## 4. Bambu Studio setup (one-time)

1. Install **Bambu Studio** from <https://bambulab.com/en/download/studio>. Create or sign in to a Bambu Lab account on first launch.
2. Put the P2S and your PC on the **same Wi-Fi SSID** (no guest network / client isolation). Run the on-printer setup wizard.
3. In Bambu Studio: **Device** tab → click **+** → Studio discovers the P2S → click it to bind to your account.
4. (Optional) On the printer: **Settings → WLAN → LAN Only Mode** on. A lock icon appears next to the device in Studio.
5. Confirm the default presets are available: printer **Bambu Lab P2S 0.4 nozzle**, filament **Bambu PLA Basic**, process **0.20 mm Standard @BBL P2S**.

## 5. Per-part print workflow

For every `.3mf` file in §3:

1. **Open** the `.3mf` in Bambu Studio (File → Open Project, or drag-and-drop). Accept the prompt to load the project's printer, filament, and process presets.
2. **Verify presets** in the top-right preset bar:
   - Printer: **Bambu Lab P2S 0.4 nozzle**
   - Filament: **Bambu PLA Basic**
   - Process: a **Standard** preset (typically 0.20 mm)
   - Plate: **Textured PEI Plate**
3. **Do NOT change** layer height, wall loops, infill density/pattern, speed, temperature, or plate type. "Default settings" here means *the bundled Bambu Studio P2S + PLA Basic preset, unmodified*.
4. **Enable tree supports**:
   - Left sidebar → **Support** tab.
   - Tick **Enable support**.
   - Set **Type** to **`tree(auto)`**.
   - Leave the sub-style on its default — Bambu Studio auto-selects **Tree Hybrid** when the process preset enables supporting material, otherwise **Tree Organic**. Do not override.
   - (Menu wording has been observed as both `tree(auto)` and `Tree (auto)` across Bambu Studio versions; either option is the intended one.)
5. **Slice** (top-right **Slice plate**). Inspect the preview:
   - First layer should fully cover the bed-contact face.
   - Tree branches should not pass through the part.
   - Estimated time / material should look reasonable for the geometry.
6. **Send to printer**: click **Print** → select the bound P2S. On first print per plate, enable **Bed Leveling** and **Flow Calibration**. Click **Send**.
7. **Alternative offline path**: **Export → Export plate sliced file (.gcode.3mf)** to microSD, insert into printer, print from the touchscreen.
8. **Observe the first layer** via the printer camera in Studio's **Device** tab. Abort if extrusion isn't clean.
9. **Remove the part** after cooldown: flex the PEI plate to pop it, then peel tree branches by hand. Use flush cutters at stubborn contact points — never pry with metal tools against PEI.

## 6. Part-specific notes

See [`../hardware/README.md`](../hardware/README.md) for why each part exists in its current form. Printing notes below are orientation and post-processing hints; none of them require overriding defaults.

| Part | Notes |
|---|---|
| `launcher mount v2.3mf` | Print flat against the plate as saved in the `.3mf`. v2 is the deployed variant; see [Version 2.0.0 — servo integrated into launcher](../hardware/README.md#version-200---servo-integrated-into-launcher). |
| `RPI camera mount.3mf` | Check lens-hole orientation before printing. No post-processing required beyond tree-support removal. |
| `mount to plate bottom.3mf` | Interfaces with the TurtleBot3 waffle plate. Verify mounting holes are at M3 clearance after print; drill/ream if needed. |
| `wire holder (end).3mf` | Cosmetic / strain-relief. Default infill is adequate. |
| `Plunger face.3mf` | v2.1.0+ split from the rack. Mates to `Plunger rack.3mf` with an M3 self-forming thread — do not over-tighten. See [Version 2.1.0 — bearing supports for rack and pinion](../hardware/README.md#version-210---bearing-supports-for-rack-and-pinion). |
| `Plunger rack.3mf` | Rack teeth are on the critical surface. If tree supports scar the teeth in the slicer preview, re-slice with **Normal (auto)** supports for this part only. Washers may be needed during assembly — see [Rack and pinion engagement](../hardware/README.md#rack-and-pinion-engagement). |
| `barrel2storage.3mf` | Barrel-side interfaces with the storage tube — verify bore diameter after print; ream if under-sized. |
| `spur gear for servo.3mf` | Non-bearing pinion. Use only for v2.0.0 / bench testing — the deployed v2.2.0 stack uses the bearing variant below. |
| `spur gear for servo with bearing.3mf` | **Deployed v2.2.0 pinion.** Internal pocket for INA HF 1012 clutch bearing + IKO LRT 61010 inner ring (press-fit after print). See [Clutch bearing reset](../hardware/README.md#clutch-bearing-reset). Sand the bore lightly if press-fit is too tight. |
| `Plunger.3mf` (v1) | Legacy single-piece plunger (pre-split). Do not print for the current build. |
| `Tube.3mf` (v1) | Legacy tube. Do not print for the current build. |
| `launcher assembly.3mf` (v1) | Legacy multi-part plate. Do not print for the current build. |
| `launcher mount.3mf` (legacy) | Superseded by `launcher mount v2.3mf`. Do not print for the current build. |

## 7. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| First layer won't stick, part pops off | Plate type in slicer must match physical plate (**Textured PEI**). Clean plate with warm water + dish soap (no acetone, no moisturizer soaps). Run **Bed Leveling** from the printer utility menu. Try +5 °C bed. |
| Tree supports won't detach / tear the part | Increase **top z-distance** by one layer for that part only, or re-slice with **Normal (auto)** supports. Never pry with metal tools against PEI. |
| Rough underside on an overhang | Known tree-support artefact. Re-slice that part with **Normal (auto)** supports. |
| Nozzle clog / under-extrusion / clicking extruder | Run **Nozzle Cleaning** from the printer utility menu, then **Flow Dynamics Calibration** before retrying. |
| Model looks rotated in the `.3mf` | Do not re-orient — the author saved it that way. Re-orienting forces support regeneration and may change the critical surface. Confirm with the team before rotating. |
| Slicer warns about AMS slots on a base P2S | Base P2S has no AMS port. Ignore the warning and feed from the external spool holder. On a P2S Combo, match each filament slot by colour before sending. |

## 8. References

- [`../hardware/README.md`](../hardware/README.md) — Daniel's mechanical design doc: system description, v1.0.0 → v2.2.0 history, clutch-bearing mechanism, materials, limitations.
- Bambu Lab P2S product page — <https://bambulab.com/en-us/p2s>
- Bambu Lab P2S specifications — <https://bambulab.com/en/p2s/specs>
- Bambu Studio download — <https://bambulab.com/en/download/studio>
- Bambu Studio Quick Start — <https://wiki.bambulab.com/en/software/bambu-studio/studio-quick-start>
- Bambu Studio Support settings (tree types) — <https://wiki.bambulab.com/en/software/bambu-studio/support>
- First layer troubleshooting — <https://wiki.bambulab.com/en/knowledge-sharing/first-layer-not-sticking>
- Textured PEI plate care — <https://wiki.bambulab.com/en/general/textured-PEI-plate-not-working-as-expected>
