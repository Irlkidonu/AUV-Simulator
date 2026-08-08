# Optical propagation and channel-availability model

**Status:** DRAFT — authored 28 July 2026, revised with literature values the same day.
Binding at the 7 August design freeze.
**Companion:** `../experiments/PROTOCOL.md`, `MODE_MANAGER_SPEC.md`,
`COMPARATOR_SPEC.md`, `EVALUATION_METRICS_SPEC.md`.

---

## 0. Plain summary

One physics model serves all three optical devices on the vehicle — the plain camera,
the camera with off-axis lighting, and the lidar. They are not three separate
simulations; they are three ways of reading the same model with different lamp and
receiver geometry.

The model's job is to answer one question at every instant: **for each optical device,
given the current water and the current altitude, does the vehicle get a usable position
fix or not?** Everything else in Paper 2 — the mode manager's predictions, the decision
to descend, the choice between camera and lidar — depends on that answer.

Two ideas carry the whole document:

1. **Attenuation length** is a distance unit that already accounts for how murky the
   water is. Each device works out to a certain number of attenuation lengths, and that
   number is a property of the device, published in the literature.
2. **Altitude is the vehicle's only lever on the water.** It cannot make the water
   clearer, but flying lower shortens the light path, and viability falls off
   exponentially with path length. That is why "descend to see" is a real navigation
   action and not a gimmick.

---

## 1. Why this document exists

The invalidated earlier work in this workspace prescribed optical degradation directly:
the visual position fix received noise `σ = 0.015·e^{3.9t}` and a **hand-chosen bias
`[0.35, 0.15, 0]·t`** applied straight to the measurement. That bias magnitude was later
found to drive almost the entire headline improvement, and it was never calibrated
against anything physical.

**In Paper 2 no optical error term is prescribed.** Noise, bias, dropout, and
availability all *emerge* from propagation geometry and water optical properties. The
free parameters are physical quantities with published ranges — attenuation coefficient,
backscatter fraction, lamp baseline, altitude — not error terms chosen to produce a
result. A reviewer may dispute a water level; they cannot dispute a bias that was fitted
to make the method win, because no such term exists.

---

## 2. The common currency: attenuation lengths

**In plain terms.** One attenuation length is the distance over which light falls to
about 37 % (1/e) of its starting brightness. It is not a fixed number of metres — it
depends on the water. In clear water one attenuation length may be several metres; in
murky water, tens of centimetres.

This unit is useful because it separates *how good the device is* from *how bad the
water is*. "This camera works to two attenuation lengths" is a statement about the
camera alone. Converting to metres happens afterwards, for whatever water you are in.

Formally, for a light path of geometric length `L` through water with beam attenuation
coefficient `c`:

```
τ = c · L          optical depth, in attenuation lengths
```

Every optical channel has a published maximum usable `τ` (§6). Because `τ` is linear in
both `c` and `L`, and viability is exponential in `τ`, the vehicle has exactly two ways
to change its optical situation — and it only controls one of them.

---

## 3. Water optical properties

### 3.1 The quantities

| Symbol | Quantity | Units |
|---|---|---|
| `a` | absorption coefficient | m⁻¹ |
| `b` | total scattering coefficient | m⁻¹ |
| `c = a + b` | **beam attenuation coefficient** — the imaging quantity | m⁻¹ |
| `b_b` | backscattering coefficient | m⁻¹ |
| `β(θ)` | volume scattering function | m⁻¹ sr⁻¹ |
| `B = b_b / b` | backscatter fraction | — |

Marine particulate scattering is strongly forward-peaked, so `B` is small. The model uses
a single-term analytic `β(θ)` normalised to `b` and `b_b` rather than a tabulated phase
function — adequate at this level and far cheaper.

### 3.2 Correction inherited from the earlier work

> The invalidated model used `μ_horiz = 0.201·t` m⁻¹ as a horizontal attenuation
> coefficient. A value near 0.2 m⁻¹ is a **diffuse downwelling** coefficient (`K_d`) —
> it describes how ambient daylight fades as you descend. Published `K_d` for clear
> oceanic Jerlov types runs 0.035–0.14 m⁻¹ [R6].
>
> The quantity that governs *imaging through a water path* is the **beam attenuation
> coefficient** `c = a + b`, which is substantially larger — published figures put clear
> seawater near 0.6 m⁻¹, highly turbid water above 1.5 m⁻¹, and polluted water in the
> 2.9–15.7 m⁻¹ range [R5].
>
> The old model therefore applied a daylight-fading coefficient to a camera-visibility
> problem. At its most turbid setting it was still optically *clearer than clear water*,
> which is why the simulated camera never genuinely struggled and the study had to add a
> hand-tuned bias to produce any effect at all. **Paper 2 uses `c`, and the manuscript
> states this distinction explicitly.**

### 3.3 Setting the water levels without the paywalled tables

The two authoritative per-Jerlov-type IOP tables — Solonenko & Mobley [R1] and Wei et
al. [R7] — are both behind paywalls, and the Ocean Optics Web Book only points at them.
**This is not a blocker, for a specific reason:**

> The load-bearing parameters in this specification are the per-channel `τ_max` limits
> (§6), and those are **openly published** [R2][R3]. The beam attenuation `c` only
> converts attenuation lengths into metres. It sets the scale of the scenario, not the
> structure of the decision.

Paper 2 therefore **does not claim to simulate a named Jerlov water type.** It declares
water levels directly in `c`, chosen to span the published plausible band and to place
the vehicle at the decision boundaries the experiment needs to exercise. This is a
design-of-experiment choice, declared as such, and it is more honest than asserting a
water-type correspondence the model has not earned.

**Declared water levels** (design levels, spanning the [R5] band):

| Level | `c` (m⁻¹) | Position in the published band |
|---|---|---|
| `W0` clear | 0.20 | below the clear-seawater figure |
| `W1` moderate | 0.60 | at the reported clear-seawater value |
| `W2` degraded | 1.20 | between clear and "highly turbid" |
| `W3` turbid | 2.00 | above the >1.5 highly-turbid threshold, below polluted |

The scenario turbidity index `t ∈ [0,1]` interpolates `c` across `W0`–`W3`.

> **`t` and `c` are hidden state.** They are generated by the scenario, consumed by this
> model, and written to the evaluator truth record. They are never published to the
> estimator, mode manager, or controller — protocol rule `N2`, test `T3`. The earlier
> system computed optical quality as `q = 1 − t`, inverting a value the experiment
> itself had set; that is exactly what this rule forbids.

### 3.4 Spectral handling

Two bands only, to stay cheap: a broadband white channel for the camera, and a narrow
532 nm green channel for the lidar. Green attenuates least in coastal water, which is why
laser systems use it, so the lidar receives a lower `c` than the camera's band-averaged
value. This is one of the two physical reasons the two optical channels fail at different
times; §6.3 gives the other.

---

## 4. Geometry — where altitude enters

For a downward-looking sensor at altitude `h` above the seabed with boresight tilt `φ`:

```
R = h / cos(φ)                                   one-way slant range to the seabed
L = R_source→target + R_target→receiver ≈ 2R     two-way path under vehicle illumination
τ = c · L ≈ 2 c h
```

**The two-way path is why altitude is such a strong lever.** Light travels down to the
seabed and back, so the optical depth is roughly `2ch` — halving altitude halves `τ`, and
viability improves as `exp(−2cΔh)`.

This single relation is the physical justification for the altitude action `A3` in
`MODE_MANAGER_SPEC.md`, and it is why altitude control is protected from the scope cuts
in `PROTOCOL.md` §11.

---

## 5. Radiometric model

**Signal** — light reflected off the seabed patch and returned to the receiver:

```
E_signal ∝ ρ · E_source · exp(−c · L) / R²
```

**Backscatter** — light scattered straight back to the receiver from particles in the
**common volume**, the region where the lamp cone and the receiver field of view overlap:

```
E_bs ∝ ∫_{r_min}^{r_max} β(θ(r)) · E_source(r) · exp(−2 c r) / r² dr
```

The `1/r²` weighting makes the **near field dominant**: most backscatter comes from
particles right in front of the vehicle, brightly lit by its own lamps. Capturing this
correctly is what makes off-axis lighting work in the model, because off-axis lighting
attacks exactly this term.

**Contrast and detectability:**

```
C   = (E_signal − E_bs) / (E_signal + E_bs)
SNR = E_signal / sqrt(E_signal + E_bs + E_read²)
```

Whether an optical position fix exists is a function of `C` and `SNR` — never of `t`
directly.

---

## 6. The three optical channels

### 6.1 Camera with coaxial lighting

Lamps sit next to the camera, so the common volume begins essentially at the housing
(`r_min → 0`) and the full near-field `1/r²` backscatter term is integrated. Backscatter
is maximal. **Published usable range: 1–2 attenuation lengths** [R3].

### 6.2 Camera with off-axis lighting

Separating the lamp from the camera by a baseline `d` means the lamp cone and the field
of view do not intersect until

```
r_min ≈ d / (tan θ_cam + tan θ_light)
```

Everything closer than `r_min` contributes **no backscatter at all** — and that excluded
region is precisely where the `1/r²` weighting was largest. So contrast rises while the
signal term is untouched. **Published usable range: ~3 attenuation lengths** [R3].

**This technique is standard published practice, not an invention of this project.** The
literature states directly that acceptable imaging at around three attenuation lengths is
obtained by spatially separating the light source from the camera [R3]. Paper 2's claim
is not that off-axis lighting works — it is that **deciding automatically when to use it,
as part of a navigation policy, improves mission outcomes.** That is a far safer claim.

**Where the optical bias comes from — now a consequence, not a choice.** An off-axis
source illuminates the scene asymmetrically, so the residual backscatter veil has a
spatial centroid displaced from the optical axis. That displaced veil shifts the apparent
centroid of tracked features, appearing in the derived position fix as a
**direction-dependent bias whose magnitude follows from `d`, `θ`, `h` and `c`** — not
from a tuned vector. Off-axis lighting therefore trades a large symmetric contrast gain
for a smaller geometric bias. The manager's decision to enable it is a genuine trade.

### 6.3 Lidar / laser line scan

A collimated 532 nm beam with a narrow receiver acceptance angle has an instantaneous
common volume orders of magnitude smaller than a floodlit camera's, and range gating
rejects returns outside the expected time-of-flight window. Both effects suppress `E_bs`
directly. **Published usable range: 5–6 attenuation lengths for continuous-wave laser
line scan [R2]; up to 7 for range-gated pulsed systems [R3].**

Its costs are real and must be modelled, or the manager will simply always choose it:

- narrow instantaneous footprint → scanning → **low effective fix rate**;
- precision degrades with range → **needs low altitude**, conflicting with survey swath;
- **power draw**, entering the mission-cost budget;
- it is still optical — at high enough `c` it dies too, just later.

### 6.4 The published ladder, and why it matters

| Configuration | Usable range | Source |
|---|---|---|
| Camera, lamp adjacent (coaxial) | **1–2** attenuation lengths | [R3] |
| Camera, lamp **spatially separated** | **~3** attenuation lengths | [R3] |
| Laser line scan (continuous wave) | **5–6** attenuation lengths | [R2] |
| Range-gated pulsed laser | **up to 7** attenuation lengths | [R3] |

A published gap of 1–2 vs. ~3 vs. 5–6 means the three configurations **genuinely fail at
different water and altitude states**. The multi-modal decision the manager makes is not
manufactured by parameter choice — it follows from measured device performance. This is
the evidence behind validation test V4.

---

## 7. Design check — the decision grid

With the declared altitudes of §9.3 (`h_nominal = 3.0 m`, `h_low = 1.0 m`, so `L ≈ 6 m`
and `2 m`) and the `τ_max` levels of §9.1, the model produces:

| Water | `τ` at 3.0 m | `τ` at 1.0 m | Coaxial (1.5) | Off-axis (3.0) | Lidar (5.5) |
|---|---|---|---|---|---|
| `W0` c=0.20 | 1.2 | 0.4 | ✅ both | ✅ both | ✅ both |
| `W1` c=0.60 | 3.6 | 1.2 | ❌ high, ✅ low | ❌ high, ✅ low | ✅ both |
| `W2` c=1.20 | 7.2 | 2.4 | ❌ both | ❌ high, ✅ low | ❌ high, ✅ low |
| `W3` c=2.00 | 12.0 | 4.0 | ❌ both | ❌ both | ❌ high, **✅ low only** |

This grid is a **design-time sanity check, not a result.** It shows the parameter set
produces the structure the study needs:

- **`W1` is the altitude case.** Descending alone resurrects the camera. Test V5.
- **`W2` is the off-axis case.** At low altitude, separated lighting works where coaxial
  does not — the lighting decision has consequences.
- **`W3` is the lidar case.** Only the lidar survives, and only at low altitude. The
  envelopes do not nest. Test V4.

Because the lidar's rate and power costs are modelled (§6.3), "always fly low with the
lidar" is not a free winning strategy — it sacrifices survey swath and endurance, which
the mission-cost budget prices.

---

## 8. Model outputs and the manager interface

### 8.1 Per-channel outputs, per tick

| Output | Meaning |
|---|---|
| `avail_k ∈ {0,1}` | whether a position fix is produced this tick |
| `R_k` | measurement covariance, derived from `SNR` and geometry |
| `bias_k` | geometric bias vector, derived per §6.2 |
| `rate_k` | effective fix rate |
| `quality_k` | image-derived quality score — **the only optical quantity visible to the manager** |

`quality_k` is computed from rendered or synthesised image content (feature count,
contrast statistics), never from `t` or `c`.

### 8.2 The availability model

The mode manager needs a *predictive* model to evaluate candidate configurations before
committing to them (`MODE_MANAGER_SPEC.md` §2, step 2):

```
P(usable fix | quality q, altitude h, configuration k)
```

- Fitted on **development seeds only**, from logged `(q, h, k) → avail` outcomes.
- Scored for **calibration**, not just accuracy — Brier score or reliability diagram, the
  same discipline Paper 1 applies to its visual-event calibration.
- Frozen 7 August; never refitted after held-out inspection.
- Deliberately trained on the *observable* `q`, not on `c` or `t`, so its predictions
  degrade honestly when the water state is ambiguous.

This model is the formal content of "optical feedback": the vehicle predicts what it
would be able to see under each configuration it could adopt, and acts on that
prediction.

---

## 9. Parameter table

No entry is blank. Every value is either a published figure with a source, or a declared
design choice with the reasoning shown.

### 9.1 Channel limits — from literature

| Symbol | Value | Published range | Source |
|---|---|---|---|
| `τ_max,coax` | **1.5** AL | 1–2 | [R3] |
| `τ_max,offaxis` | **3.0** AL | ~3 | [R3] |
| `τ_max,lidar` | **5.5** AL | 5–6 CW LLS; up to 7 range-gated | [R2][R3] |

### 9.2 Water — declared levels within the published band

| Symbol | Value | Basis |
|---|---|---|
| `c` at `W0…W3` | **0.20 / 0.60 / 1.20 / 2.00** m⁻¹ | Spans the [R5] band; see §3.3 |
| `B = b_b/b` | **0.0183** primary | Petzold's conventionally used value [R4] |
| `B` alternative | **0.013** | Field geometric mean across diverse waters [R4] — **used as the sensitivity alternative** |

### 9.3 Geometry — declared design choices

| Symbol | Value | Reasoning |
|---|---|---|
| `h_nominal` | **3.0 m** | Consistent with close-range AUV photographic survey practice, which requires low altitude for resolution and image overlap [R8] |
| `h_low` | **1.0 m** | Chosen so the §7 grid places `W1` on the altitude decision boundary — this is what makes test V5 meaningful |
| `θ_cam` | **30°** half-angle | Existing 60° FOV camera model in the workspace |
| `θ_light` | **30°** half-angle | Plausible lamp cone, matched to FOV |
| `d` | **0.35 m** | Gives `r_min = d/(tan30° + tan30°) = 0.35/1.155 ≈ 0.30 m`, excluding the brightest near-field backscatter while staying within BlueROV2-class frame width |
| `ρ` | **0.20** | Declared design value. Low sensitivity — it scales signal uniformly and does not decide the backscatter-limited availability boundary. **No literature value located**; sweep if it proves influential |

**Everything in §9.2 and §9.3 is an explicit design level, not an empirical claim,** and
the manuscript states this. Levels are checked for plausibility and detectability on
development runs, then locked. Any correction after freeze creates a new campaign
identifier rather than editing a frozen one.

### 9.4 Mandatory sensitivity sweeps

Per `PROTOCOL.md` §7, each of these gets a one-dimensional sweep **including its null
value**, with the claim stated conditionally over the range where it holds:

- **`c` at `W3`** — the degradation ceiling.
- **`d` → 0** — recovers the coaxial case and **removes off-axis lighting entirely**.
  The null test for the optical-feedback contribution.
- **`τ_max,lidar` → `τ_max,offaxis`** — makes the optical channels nest, removing the
  multi-modal decision.
- **`B`: 0.0183 → 0.013** — the two published backscatter-ratio values.

The middle two are the honest versions of *does the mechanism actually matter*. If the
method still wins with `d = 0` and nested envelopes, the win is not coming from optical
feedback, and the paper must say so.

---

## 10. Validation tests

Deterministic, cheap, and part of the freeze record.

- **V1 — monotonicity.** Contrast decreases monotonically in `c` and in `h`. Availability
  is non-increasing in `τ`.
- **V2 — limits.** As `c → 0`, backscatter → 0 and the channel is signal-limited. As
  `c → ∞`, all optical channels report unavailable — never a spuriously confident fix.
- **V3 — off-axis is not a free win.** Increasing the source–camera separation `d`
  strictly increases contrast, because less of the illuminated volume is shared with the
  camera's field of view and common-volume backscatter falls. It must also cost
  something: a laterally displaced source shifts the apparent position of a shadowed or
  sloped target, which enters navigation as a **geometric bias**.

  The bias is **not** monotone in `d`, and an earlier draft of this test wrongly asserted
  that it was. It is zero at `d = 0` — a coaxial source casts no visible shadow offset —
  rises as the separation grows, and falls again once the source is far enough off-axis
  that the illuminated and imaged volumes barely overlap: there is then little common
  volume left whose displacement could be misread. The test therefore asserts the
  physically defensible statement, which is that the bias is **strictly positive at the
  operating separation** `d = 0.35 m` and vanishes as `d → 0`, not that it increases
  without bound. A model in which off-axis lighting is a free win fails either form of
  the test; only the second form is also true.
- **V4 — envelope non-nesting.** There is a non-empty region of the `(c, h)` plane where
  the lidar is available and the camera is not, **and** a region where the camera's rate
  advantage makes it preferable. §7 predicts both; the test confirms them in code.
- **V5 — altitude lever.** At `W1`, reducing `h` from 3.0 m to 1.0 m restores camera
  availability. If it does not, the altitude action carries no contribution and the
  levels are wrong.
- **V6 — determinism.** Identical seed and configuration reproduce identical channel
  outputs bit-for-bit.
- **V7 — no hidden-state leakage.** A probe asserts that no consumer outside the
  evaluator can reach `t`, `c`, or `τ`.

**V4 and V5 are gating.** They run on development data before the 4 August cut gate. If
either fails, the affected claim is dropped rather than rescued by retuning the physics.

---

## 11. Deliberate exclusions

Stated as limitations in the manuscript, not hidden:

- no full radiative-transfer or Monte Carlo photon transport — single-scatter
  approximation with an analytic phase function;
- no wavelength-resolved spectra beyond the two bands of §3.4;
- no ambient-daylight variation with depth or time of day;
- no bubble, plankton-layer, or sediment-plume structure — turbidity is horizontally
  uniform within a scenario phase;
- no refractive housing model (Paper 1's `REFRACTIVE_CAMERA_SPEC.md` scope);
- **no claimed correspondence to a named Jerlov water type** (§3.3). Water levels are
  declared in `c` within a published band. If per-type IOP tables are obtained later, the
  mapping is a straightforward relabelling that changes no result;
- parameters are literature-plausible design levels, **not field-calibrated**, and no
  claim of quantitative field agreement is made anywhere in the paper.

---

## 12. References

Consulted 28 July 2026. To be added to the Paper 2 bibliography and cited at point of use.

**Openly accessible — these carry the load-bearing parameters:**

- **[R2]** *Performance considerations for continuous-wave and pulsed laser line scan
  (LLS) imaging systems.* Journal of the European Optical Society (2010) **5**, 10020.
  [ADS](https://ui.adsabs.harvard.edu/abs/2010JEOS....5E0020S/abstract) ·
  [PDF](https://www.researchgate.net/publication/236931190_Performance_considerations_for_continuous-wave_and_pulsed_laser_line_scan_LLS_imaging_systems).
  Source for `τ_max,lidar` (5–6 AL).
- **[R3]** *Extended Range Underwater Optical Imaging Architecture.*
  [PDF](https://www.researchgate.net/publication/224287116_Extended_Range_Underwater_Optical_Imaging_Architecture).
  Source for the 1–2 AL coaxial camera figure, the ~3 AL source-separation figure, and
  the up-to-7 AL range-gated figure. **The key citation for §6.2** — it states directly
  that spatial separation of source and camera is the accepted means of reaching ~3
  attenuation lengths.
- **[R4]** Boss, E. et al. *Particulate backscattering ratio at LEO 15 and its use to
  study particle composition and distribution.*
  [PDF](https://misclab.umeoce.maine.edu/documents/BossetalJGR2004.pdf) · Twardowski et
  al., *Spectral variability of the particulate backscattering ratio*,
  [Optics Express 15(11) 7019](https://opg.optica.org/oe/fulltext.cfm?uri=oe-15-11-7019).
  Source for `B = 0.0183` (Petzold) and `B = 0.013` (field geometric mean).
- **[R6]** Williamson, C. et al. (2023). *Depth profiles of Jerlov water types.*
  Limnology & Oceanography Letters.
  [Open PDF](https://aslopubs.onlinelibrary.wiley.com/doi/pdf/10.1002/lol2.10338).
  Source for the oceanic `K_d` ranges in §3.2.
- **[R8]** *On the modification of the SPARUS II AUV for close range imaging survey
  platform.* [arXiv:2111.08971](https://arxiv.org/abs/2111.08971). Context for
  close-range AUV imaging survey altitude and strobe lighting practice.

**Paywalled — cited for completeness, not required by this specification (§3.3):**

- **[R1]** Solonenko, M. G. & Mobley, C. D. (2015). *Inherent optical properties of
  Jerlov water types.* Applied Optics **54**(17), 5392–5401.
  [DOI 10.1364/AO.54.005392](https://doi.org/10.1364/AO.54.005392).
- **[R5]** *Beam attenuation coefficient for different water turbidities.* Applied Optics
  **63**(24), 6482 (2024).
  [Abstract](https://opg.optica.org/ao/abstract.cfm?uri=ao-63-24-6482). The magnitudes
  quoted in §3.2 and §3.3 come from the abstract and secondary reporting.
- **[R7]** Wei, et al. (2022). *Measured IOPs of Jerlov water types.* Applied Optics
  **61**, 9951. [ADS](https://ui.adsabs.harvard.edu/abs/2022ApOpt..61.9951W/abstract).
  Derives measured `a` and `b` for six Jerlov types from the World-wide Ocean Optics
  Database.

**Optional improvement if access is obtained later:** entering per-type `c` from [R1] or
[R7] would let the paper name its water types instead of declaring `c` levels. This
changes no result and no claim — only a label. It is not on the critical path.
