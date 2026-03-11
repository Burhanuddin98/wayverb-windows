"""
fix_materials.py — Scientifically correct wayverb materials.json patch.

Fixes:
  1. Scattering for first 41 materials (all 0.10 flat → frequency-dependent)
  2. Mineral wool 100mm absorption (currently worse than 50mm at high freq)
  3. Bass trap absorption (should be high at LF, not HF)
  4. Diffuser QRD deep/shallow scattering (0.04 → 0.90+)
  5. Binary amplitude diffuser scattering (0.03 → 0.80+)
  6. Bookshelf scattering (0.11 → 0.50-0.75)

Reference: Beranek 1992, Kuttruff 2000, ISO 354, Harris 1991.
Bands (16 x 1/3-oct): 125,160,200,250,315,400,500,630,800,1k,1.25k,1.6k,2k,2.5k,3.15k,4k Hz
"""

import json, math, copy

SRC  = r"C:\RoomGUI\wayverb\wayverb-0.0.1\wayverb-0.0.1\bin\materials.json"
DEST = SRC  # overwrite in-place (back up first if nervous)

BANDS = [125, 160, 200, 250, 315, 400, 500, 630, 800,
         1000, 1250, 1600, 2000, 2500, 3150, 4000]
N = len(BANDS)

LOG_LO = math.log(BANDS[0])
LOG_HI = math.log(BANDS[-1])

def lerp_scat(lo, hi):
    """Scattering rising from lo@125Hz to hi@4kHz (log-freq linear)."""
    out = []
    for f in BANDS:
        t = (math.log(f) - LOG_LO) / (LOG_HI - LOG_LO)
        out.append(round(lo + t * (hi - lo), 4))
    return out

def interp_abs(oct_freqs, oct_vals):
    """
    Log-frequency interpolate from sparse octave-band reference values
    to the 16 1/3-octave bands.  oct_freqs must bracket [125,4000].
    """
    out = []
    for f in BANDS:
        lf = math.log(f)
        # find surrounding pair
        i = 0
        while i < len(oct_freqs) - 2 and math.log(oct_freqs[i+1]) < lf:
            i += 1
        l0 = math.log(oct_freqs[i]);   l1 = math.log(oct_freqs[i+1])
        v0 = oct_vals[i];              v1 = oct_vals[i+1]
        if l1 == l0:
            out.append(round(v0, 4))
        else:
            t = (lf - l0) / (l1 - l0)
            out.append(round(max(0.0, min(1.0, v0 + t * (v1 - v0))), 4))
    return out

# ─── SCATTERING CORRECTIONS ───────────────────────────────────────────────────
# Keys must match material names exactly as they appear in the JSON.

SCAT_PATCHES = {
    # Hard surfaces — very smooth → low scattering
    "hard surface - average":               lerp_scat(0.04, 0.10),
    "hard surface - rendered brickwork":    lerp_scat(0.05, 0.12),
    "hard surface - rough concrete":        lerp_scat(0.06, 0.18),
    "hard surface - smooth unpainted concrete": lerp_scat(0.03, 0.08),
    "hard surface - rough lime wash":       lerp_scat(0.08, 0.22),
    "hard surface - smooth brickwork, flush pointing, painted": lerp_scat(0.03, 0.07),
    "hard surface - smooth brickwork, 10mm deep pointing, rough": lerp_scat(0.10, 0.25),
    "hard surface - brick wall, rough finish": lerp_scat(0.08, 0.20),
    "hard surface - ceramic tiles, smooth finish": lerp_scat(0.02, 0.04),
    "hard surface - limestone":             lerp_scat(0.04, 0.12),
    "hard surface - reverb chamber":        lerp_scat(0.02, 0.04),
    "hard surface - concrete floor":        lerp_scat(0.03, 0.08),
    "hard surface - marble floor":          lerp_scat(0.02, 0.03),
    # Lightweight panels — some surface irregularity
    "lightweight - wool-backed plasterboard": lerp_scat(0.10, 0.25),
    "lightweight - wooden lining":          lerp_scat(0.12, 0.22),
    # Glazing — smooth and specular
    "glazing - single 3mm glass pane":      lerp_scat(0.02, 0.04),
    "glazing - normal glass window":        lerp_scat(0.03, 0.05),
    "glazing - lead glazing":               lerp_scat(0.04, 0.08),
    "glazing - double glazing, large gap":  lerp_scat(0.03, 0.05),
    "glazing - double glazing, small gap":  lerp_scat(0.03, 0.05),
    "glazing - double glazing, leaded":     lerp_scat(0.04, 0.08),
    # Wood — moderate scattering
    "wood - 16mm, on planks":               lerp_scat(0.10, 0.20),
    "wood - thin plywood":                  lerp_scat(0.10, 0.20),
    "wood - 16mm, on studs":                lerp_scat(0.10, 0.20),
    "wood - audience floor":                lerp_scat(0.10, 0.18),
    "wood - stage floor":                   lerp_scat(0.10, 0.18),
    "wood - solid door":                    lerp_scat(0.08, 0.15),
    # Floors — hard floors low, carpets high
    "floor - standard reflective":          lerp_scat(0.03, 0.08),
    "floor - cotton carpet":                lerp_scat(0.30, 0.70),
    "floor - thick carpet":                 lerp_scat(0.35, 0.75),
    "floor - thin carpet":                  lerp_scat(0.25, 0.60),
    "floor - open foam-backed carpet":      lerp_scat(0.30, 0.70),
    "floor - closed foam-backed carpet":    lerp_scat(0.30, 0.70),
    "floor - tufted felt-backed carpet":    lerp_scat(0.35, 0.75),
    "floor - needle felt":                  lerp_scat(0.25, 0.60),
    "floor - soft carpet":                  lerp_scat(0.30, 0.70),
    "floor - hairy carpet":                 lerp_scat(0.35, 0.75),
    "floor - rubber carpet":                lerp_scat(0.15, 0.35),
    # Curtains / drapes — fabric → moderate-high scattering
    "curtains - heavy cotton":              lerp_scat(0.35, 0.65),
    "curtains - light":                     lerp_scat(0.25, 0.55),
    "curtains - studio":                    lerp_scat(0.40, 0.70),
    # Diffusers — scattering should be VERY HIGH (that's the whole point)
    "Diffuser QRD (deep)":                  lerp_scat(0.85, 0.99),
    "Diffuser QRD (shallow)":               lerp_scat(0.70, 0.95),
    "Binary amplitude diffuser":            lerp_scat(0.75, 0.95),
    # Bookshelf — irregular surfaces → moderate-high scattering
    "Bookshelf (diffusive)":                lerp_scat(0.50, 0.78),
}

# ─── ABSORPTION CORRECTIONS ───────────────────────────────────────────────────
# Format: octave-band reference freqs, values → will be interpolated

ABS_PATCHES = {
    # Mineral wool 100mm must be >= 50mm at ALL frequencies.
    # Real measurements (ISO 354, 80 kg/m³ Rockwool):
    #   50mm:  125=0.14, 250=0.45, 500=0.80, 1k=0.97, 2k=0.99, 4k=0.99
    #   100mm: 125=0.35, 250=0.80, 500=0.98, 1k=0.99, 2k=0.99, 4k=0.99
    "Mineral wool 100mm": interp_abs(
        [125,  250,  500,  1000, 2000, 4000],
        [0.35, 0.80, 0.98, 0.99, 0.99, 0.99]
    ),
    # Mineral wool 50mm — current data (0.10→0.95) is actually plausible
    # but let's align to the reference above:
    "Mineral wool 50mm": interp_abs(
        [125,  250,  500,  1000, 2000, 4000],
        [0.14, 0.45, 0.80, 0.97, 0.99, 0.99]
    ),
    # Bass trap (porous corner) — should be HIGH at low freq.
    # Corner placement gives ~6 dB pressure gain → very high LF absorption.
    # Typical floor-to-ceiling 200mm thick corner fill (Everest 2001):
    #   125=0.65, 250=0.90, 500=0.95, 1k=0.95, 2k=0.90, 4k=0.85
    "Bass trap (porous corner)": interp_abs(
        [125,  250,  500,  1000, 2000, 4000],
        [0.65, 0.90, 0.95, 0.95, 0.90, 0.85]
    ),
    # Panel absorber (100Hz) — cavity/membrane tuned to 100Hz.
    # Should peak ~100 Hz, roll off fast above.
    # Beranek Table 6.1 style panel: peak α≈0.50 @100Hz, drops to 0.05 @1kHz
    "Panel absorber (100Hz)": interp_abs(
        [80,   100,  125,  160,  200,  250,  315,  400,  630,  1000, 2000, 4000],
        [0.38, 0.50, 0.45, 0.30, 0.18, 0.12, 0.08, 0.06, 0.05, 0.04, 0.04, 0.04]
    ),
    # Panel absorber (160Hz)
    "Panel absorber (160Hz)": interp_abs(
        [80,   100,  125,  160,  200,  250,  315,  400,  630,  1000, 2000, 4000],
        [0.22, 0.35, 0.45, 0.55, 0.42, 0.25, 0.14, 0.09, 0.06, 0.05, 0.04, 0.04]
    ),
    # Panel absorber (250Hz)
    "Panel absorber (250Hz)": interp_abs(
        [125,  160,  200,  250,  315,  400,  500,  630,  1000, 2000, 4000],
        [0.15, 0.25, 0.38, 0.52, 0.42, 0.28, 0.16, 0.10, 0.07, 0.05, 0.04]
    ),
    # Panel absorber (400Hz)
    "Panel absorber (400Hz)": interp_abs(
        [125,  160,  200,  250,  315,  400,  500,  630,  1000, 2000, 4000],
        [0.08, 0.12, 0.18, 0.30, 0.42, 0.55, 0.42, 0.28, 0.12, 0.07, 0.05]
    ),
    # Helmholtz trap — keep existing shape but make peak sharper and correct.
    # Current peaks around 400Hz but is named 'tuned'. Tune to ~200Hz.
    # Helmholtz: sharp peak, fast drop on both sides (Kuttruff p.252)
    "Helmholtz trap (tuned)": interp_abs(
        [125,  160,  200,  250,  315,  400,  500,  630,  1000, 2000, 4000],
        [0.12, 0.25, 0.55, 0.85, 0.55, 0.25, 0.12, 0.07, 0.05, 0.04, 0.04]
    ),
    # Acoustic foam 25mm — should rise monotonically (thinner = less LF abs)
    # Reference (Everest Table 11.1, 25mm melamine/polyurethane):
    #   125=0.05, 250=0.12, 500=0.35, 1k=0.72, 2k=0.92, 4k=0.97
    "Acoustic foam 25mm": interp_abs(
        [125,  250,  500,  1000, 2000, 4000],
        [0.05, 0.12, 0.35, 0.72, 0.92, 0.97]
    ),
    # Acoustic foam 50mm — more LF than 25mm
    # Reference: 125=0.12, 250=0.35, 500=0.72, 1k=0.94, 2k=0.98, 4k=0.98
    "Acoustic foam 50mm": interp_abs(
        [125,  250,  500,  1000, 2000, 4000],
        [0.12, 0.35, 0.72, 0.94, 0.98, 0.98]
    ),
    # Fiberglass 25mm — monotonic rise (Knudsen & Harris)
    "Fiberglass 25mm": interp_abs(
        [125,  250,  500,  1000, 2000, 4000],
        [0.08, 0.25, 0.65, 0.90, 0.97, 0.99]
    ),
    # Fiberglass 50mm — more LF than 25mm
    "Fiberglass 50mm": interp_abs(
        [125,  250,  500,  1000, 2000, 4000],
        [0.25, 0.65, 0.92, 0.99, 0.99, 0.99]
    ),
}

# ─── Apply patches ─────────────────────────────────────────────────────────────

with open(SRC, 'r') as f:
    materials = json.load(f)

patched_abs = 0
patched_scat = 0
names_seen = set()

for mat in materials:
    name = mat['name']
    names_seen.add(name)
    if name in ABS_PATCHES:
        mat['absorption'] = ABS_PATCHES[name]
        patched_abs += 1
    if name in SCAT_PATCHES:
        mat['scattering'] = SCAT_PATCHES[name]
        patched_scat += 1

# Sanity check: warn if any patch key didn't match
for k in list(ABS_PATCHES) + list(SCAT_PATCHES):
    if k not in names_seen:
        print(f"WARNING: patch key not found in JSON: '{k}'")

with open(DEST, 'w') as f:
    json.dump(materials, f, indent=2)

print(f"Done. Patched {patched_abs} absorption entries, {patched_scat} scattering entries.")
print(f"Total materials: {len(materials)}")

# Quick validation: check Mineral wool thickness ordering
mw50  = next(m for m in materials if m['name']=='Mineral wool 50mm')['absorption']
mw100 = next(m for m in materials if m['name']=='Mineral wool 100mm')['absorption']
for i, (a,b) in enumerate(zip(mw50, mw100)):
    if b < a:
        print(f"  [FAIL] Mineral wool 100mm < 50mm at band {BANDS[i]} Hz ({b:.3f} < {a:.3f})")
print("  Mineral wool 100mm >= 50mm at all bands: OK" if all(b>=a for a,b in zip(mw50,mw100)) else "  MINERAL WOOL ORDERING STILL WRONG")

# Check bass trap peaks at low freq
bt = next(m for m in materials if m['name']=='Bass trap (porous corner)')['absorption']
if bt[0] > bt[-1]:
    print("  Bass trap: HIGH at LF, LOW at HF — correct")
else:
    print("  [FAIL] Bass trap still peaks at HF")

# Check diffuser scattering
qrd = next(m for m in materials if m['name']=='Diffuser QRD (deep)')['scattering']
print(f"  QRD deep scattering range: {min(qrd):.3f} – {max(qrd):.3f} (expect 0.85–0.99)")
