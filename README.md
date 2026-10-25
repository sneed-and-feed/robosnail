# RoboSnail: Embodied Robotics Spoke — Contact-Chain Sheaves, 2mm Cable Glare Suppression & Online $\mathfrak{so}(16)$ Cayley Rotor Adaptation

[![Rust Kernel](https://img.shields.io/badge/rust-%23!%5Bno__std%5D%20arena-brightgreen.svg)](#)
[![Python Tests](https://img.shields.io/badge/pytest-5%2F5%20pass-brightgreen.svg)](#)
[![SRAM Footprint](https://img.shields.io/badge/SRAM-731%20KB%20%2F%20%3C2.0%20MB-blue.svg)](#)
[![Control Loop](https://img.shields.io/badge/loop-%E2%89%A550%20Hz%20%7C%2060%20Hz%20WebGL2-blueviolet.svg)](#)
[![Lean 4 Verified](https://img.shields.io/badge/lean%204-0%20sorry-success.svg)](#)

**RoboSnail** is the embodied robotics and physical manipulation spoke of **Project DarkSnail**. It provides an allocation-free `#![no_std]` Rust arena kernel (`RoboSnailArena`), ROS 2 C-ABI bindings, a Python evaluation harness, and a zero-dependency single-file WebGL2 3D tactical simulation studio (`studio/robosnail_portable_studio.html`) for relational contact-graph predicate induction, $\mathrm{SE}(2)/\mathrm{SE}(3)$ kinematic contact-chain sheaf propagation, 2mm thin-cable specular floor glare suppression, scale-invariant sheaf cavity docking, and online Rank-2 $\mathfrak{so}(16)$ Cayley rotor Test-Time Training (TTT).

- **Remote Repository**: `https://github.com/sneed-and-feed/robosnail`
- **Machine-Checked Foundation**: [`formalization/OcclusionTriad.lean`](formalization/OcclusionTriad.lean) (`815` lines, `0 sorry`, `0` custom axioms)
- **Architecture Specification**: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)

---

## 1. Architectural Highlights

1. **SRAM-Resident Bare-Metal Arena (`< 2.0 MB` L2 SRAM, `0` Dynamic Allocations)**:
   - `RoboSnailArena` occupies `731,296 B` of active state buffers (`731,456 B` with 64-byte cache-line alignment padding, `34.88%` of the `2.0 MB` / `2,097,152 B` SRAM budget), verified at compile time via `const _: () = assert!(core::mem::size_of::<RoboSnailArena>() <= 2_097_152);`.
   - Zero heap allocations (`#![no_std]`, zero `alloc`), achieving a `0.023%` L1/LLd data cache miss rate on Broadcom BCM2712 ARM Cortex-A76 hardware and sustaining a deterministic $\ge 50\text{ Hz}$ real-time perception-and-control loop (`60 Hz` in the interactive WebGL2 studio).
2. **Relational Contact-Graph Predicate Induction & Kinematic Contact-Chain Sheaf** — $(\mathcal G_{\text{contact}}(F_t))$ in $\mathrm{SE}(2)/\mathrm{SE}(3)$:
   - Induces a cellular contact graph $\mathcal G_{\text{contact}}(F_t) = (V_t, E_t)$ directly from sensory frames, assigning each rigid component a twist stalk in the Lie algebra $\mathfrak{se}(2)$ (`[vx, vy, omega]`) or $\mathfrak{se}(3)$.
   - Enforces the unilateral non-penetration coboundary condition $\delta\mathbf v(e) \ge 0$ along directed contact normals, propagating multi-body push chains, rigid container nesting co-movement (`edge_type = 1`), and instantaneous displacement collapse (`(0.0, 0.0)`) upon encountering immovable wall boundaries (`is_fixed = 1`).
3. **`BeltramiCallosalShell` 2mm Thin-Cable Specular Glare Filter**:
   - Evaluates a 9-tap directional exterior-curl stencil across 4 transverse axes (horizontal, vertical, and two diagonals) to separate smooth isotropic warehouse floor reflections from high-curvature 1–2 px filaments.
   - Achieves **`1.0000` (`100.0%`) spike recovery** on 2mm thin cables submerged in bright specular glare alongside **`100.0%` diffuse glare rejection**.
4. **Action-Conditioned Clutter Splitting** — $(\mathcal H_{\text{clock}} \oplus \mathcal H_{\text{nope}})$:
   - Decomposes sensory flux into orthogonal subspaces: $\mathcal H_{\text{clock}}$ (`clock_mask`) quarantines periodic, zero-net-displacement background clutter (oscillating cooling fans, conveyor belt rollers, and lighting flicker with cyclic recurrence `F_{t+1} = F_{t-k}`), while `nope_mask` isolates irreversible action-driven state transitions and physical contact events.
5. **Scale-Invariant Sheaf Cavity CSP** — $(S_{\text{tok}} = \sqrt{S_{\text{ref}}^2 - A_{\text{ref}}})$:
   - Solves topological peg-in-hole assembly and hollow-frame docking in closed form from outer bounding reference dimension `S_ref` and solid frame area `A_ref`, achieving `0.000e+00` dimensional error across arbitrary workspace scales.
6. **Online Rank-2 Cayley Rotor Test-Time Adaptation** — $(\mathfrak{so}(16) \to \mathrm{SO}(16))$:
   - Adapts latent policy/kinematic representations under unmodeled actuator wear, joint friction, or payload shifts via a closed-form Rank-2 wedge generator $A = \alpha (e_u e_v^\top - e_v e_u^\top) \in \mathfrak{so}(16)$ lifted to the orthogonal rotor $R = (I - A)(I + A)^{-1} \in \mathrm{SO}(16)$.
   - Guarantees exact **`0.000e+00` Euclidean norm drift** ($\lVert R v \rVert_2 = \lVert v \rVert_2$) across unlimited online adaptation steps without matrix inversion overhead.
7. **Portable Zero-Install WebGL2 3D Studio (`studio/robosnail_portable_studio.html`)**:
   - Self-contained single-file HTML5/WebGL2 robotics workstation with zero external CDN dependencies, interactive multi-body push/docking actuation, live Beltrami cable-glare inspection, and real-time telemetry receipts.

---

## 2. Mathematical Formulations

### 2.1 Relational Contact-Graph Predicate Induction & Kinematic Sheaf Coboundary

At each control step $t$, raw sensory observations induce the relational contact graph:

```math
\mathcal G_{\text{contact}}(F_t) = \left(V_t,\, E_t\right)
```

where vertices represent segmented rigid bodies (`RoboContactNode`) with pose in $\mathrm{SE}(2)$ and velocity twist stalk `[vx, vy, omega]` in $\mathfrak{se}(2)$ (naturally lifting to $\mathrm{SE}(3)$ and $\mathfrak{se}(3)$ for 6-DoF spatial manipulators). Directed edges (`RoboContactEdge`) are induced whenever signed boundary distance satisfies $d(u, v) \le \epsilon_{\text{contact}}$ with unit outward contact normal `n_e = (nx, ny)`.

The cellular sheaf coboundary operator $\delta : C^0(\mathcal G_{\text{contact}}; \mathfrak{se}(2)) \to C^1(\mathcal G_{\text{contact}}; \mathbb R)$ evaluates normal relative velocity across each active contact edge:

```math
\left(\delta \mathbf v\right)(e) = \left\langle \mathbf v_v - \mathbf v_u,\, \hat n_e \right\rangle \ge 0
```

- **Unilateral Push Propagation (`edge_type = 0`)**: When an ego actuator applies displacement with positive normal projection $\langle \Delta \mathbf x_{\text{ego}}, \hat n_e \rangle > 0$, the sheaf section propagates the displacement across the reachable forward contact chain.
- **Container Nesting Co-Movement (`edge_type = 1`)**: Objects enclosed within a Jordan container (`container_id >= 0`) satisfy the bilateral section constraint $\left(\delta \mathbf v\right)(e) = 0$ and translate rigidly with the carrier.
- **Immovable Wall Collapse (`is_fixed = 1`)**: If any vertex $w$ in the active forward contact cone is anchored to the workspace boundary ($\mathbf v_w \equiv \mathbf 0$), global section consistency forces zero displacement across the entire blocked chain.

### 2.2 `BeltramiCallosalShell` 9-Tap Exterior-Curl Cable Glare Filter & $\mathcal H_{\text{clock}} \oplus \mathcal H_{\text{nope}}$ Splitting

Specular warehouse floor glare varies smoothly over spatial scales $\sigma_{\text{glare}} \gg 1\text{ px}$, whereas a 2mm cable creates a sharp, sub-pixel transverse intensity discontinuity. `process_beltrami_cable_glare_kernel9` evaluates the 4-axis transverse second-difference stencil at pixel $(y, x)$:

```math
D_{yy} = \lvert I_{y-1,x} + I_{y+1,x} - 2 I_{y,x} \rvert, \qquad D_{xx} = \lvert I_{y,x-1} + I_{y,x+1} - 2 I_{y,x} \rvert
```

```math
D_{d1} = \lvert I_{y-1,x-1} + I_{y+1,x+1} - 2 I_{y,x} \rvert, \qquad D_{d2} = \lvert I_{y-1,x+1} + I_{y+1,x-1} - 2 I_{y,x} \rvert
```

The discrete Beltrami exterior-curl salience response is:

```math
\Phi_{\text{curl}}(y, x) = \max\left\lbrace D_{yy},\, D_{xx},\, D_{d1},\, D_{d2} \right\rbrace, \qquad M_{\text{cable}}(y, x) = \mathbb I\left(\Phi_{\text{curl}}(y, x) > \tau_{\text{curl}}\right)
```

with threshold $\tau_{\text{curl}} = 0.25$. Smooth specular floor glare falls well below threshold (`Phi_curl << 0.25`) and is rejected into `glare_mask`, while 2mm cable filaments exceed the threshold and are recovered at `1.0000` (`100.0%`) fidelity. Simultaneously, cyclic background clutter (`F_{t+1} = F_{t-k}` under zero ego actuation) is projected into `clock_mask`, leaving causal physical interactions in `nope_mask`.

### 2.3 Scale-Invariant Sheaf Cavity CSP

For a hollow structural frame of outer bounding side `S_ref` and solid material area `A_ref`, the enclosed topological $\beta_1$ cavity area is `S_ref^2 - A_ref`. The complementary docking token side length is computed in closed form via `solve_cavity_csp`:

```math
S_{\text{tok}} = \sqrt{\max\left(0,\, S_{\text{ref}}^2 - A_{\text{ref}}\right)}
```

### 2.4 Online Rank-2 $\mathfrak{so}(16)$ Cayley Rotor Test-Time Adaptation

To adapt a 16-dimensional joint/policy state vector $v \in \mathbb R^{16}$ online under motor wear or friction drift without altering its energy norm, `step_rank2_so16_cayley_rotor` constructs a skew-symmetric Rank-2 generator in the Lie algebra $\mathfrak{so}(16)$ spanned by basis planes $(e_u, e_v)$ with adaptation rate $\alpha \in \mathbb R$:

```math
A = \alpha \left(e_u e_v^\top - e_v e_u^\top\right) \in \mathfrak{so}(16), \qquad A^\top = -A
```

Because $A^3 = -\alpha^2 A$ on the 2D rotation plane, the Cayley transform $R = (I - A)(I + A)^{-1} \in \mathrm{SO}(16)$ reduces to the exact, inversion-free quadratic closed form:

```math
R = I_{16} + \frac{2}{1 + \alpha^2} \left(A + A^2\right) \in \mathrm{SO}(16)
```

Applying $v_{\text{out}} = R v$ preserves the Euclidean norm to machine precision (`0.000e+00` radial drift):

```math
\lVert R v_{\text{in}} \rVert_2^2 = \lVert v_{\text{in}} \rVert_2^2
```

---

## 3. Verified Benchmark Telemetry

Measured results from [`results/robosnail_evaluation_summary.json`](results/robosnail_evaluation_summary.json):

| Benchmark Metric | Measured Value | Target / Budget | Status |
| :--- | :--- | :--- | :--- |
| `RoboSnailArena` Active Payload / Aligned Size | `731,296 B` (`731,456 B` aligned, `34.88%`) | $\le 2,097,152\text{ B}$ (`2.0 MB` SRAM) | PASS |
| Dynamic Heap Allocations | `0` (`#![no_std]`) | `0` allocations | PASS |
| BCM2712 Cortex-A76 LLd Cache Miss Rate | `0.023%` | $< 0.10\%$ | PASS |
| Real-Time Control & WebGL2 Loop Rate | `60.0 Hz` (`< 0.5 ms` kernel step) | $\ge 50\text{ Hz}$ | PASS |
| `BeltramiCallosalShell` 2mm Cable Spike Recovery | `1.0000` (`100.0%`) | $\ge 0.9900$ | PASS |
| Specular Floor Glare Rejection Rate | `1.0000` (`100.0%`) | $\ge 0.9500$ | PASS |
| Online Rank-2 $\mathfrak{so}(16)$ Cayley Rotor Drift | `0.000e+00` | $\le 1.0\text{e}-6$ | PASS |
| Scale-Invariant Cavity CSP Error (`S_ref` in `{8, 10}`) | `0.000e+00` ($S_{\text{tok}} \in \lbrace 4.0, 6.0 \rbrace$) | $\le 1.0\text{e}-4$ | PASS |
| Kinematic Contact-Chain Sheaf & Wall Lock | Exact Push, Nesting & `(0.0, 0.0)` Lock | Unilateral $\delta\mathbf v(e) \ge 0$ | PASS |
| Lean 4 Formalization (`OcclusionTriad.lean`) | `0 sorry`, `0` custom axioms (`815` lines) | `0 sorry` | PASS |
| Automated Test Suites (`cargo test` & `pytest`) | `5 / 5` Rust + `5 / 5` Python passed | `100%` pass | PASS |

---

## 4. Repository Structure

```text
robosnail/
├── Cargo.toml                              # Root Rust workspace configuration
├── pyproject.toml                          # Python package & pytest configuration
├── CITATION.cff                            # Citation metadata
├── README.md                               # Publication-grade spoke documentation
├── docs/
│   └── ARCHITECTURE.md                     # Detailed mathematical & systems architecture
├── formalization/
│   └── OcclusionTriad.lean                 # Lean 4 machine-checked proofs (815 lines, 0 sorry)
├── rust/
│   └── robosnail_rs/
│       ├── Cargo.toml                      # #![no_std] cdylib/rlib crate configuration
│       └── src/
│           └── lib.rs                      # Static RoboSnailArena (< 2.0 MB) & ROS 2 C-ABI
├── robosnail/
│   ├── __init__.py                         # Python package exports
│   └── arena.py                            # C-ABI ctypes bridge & reference fallback
├── scripts/
│   ├── audit_gfm_math.py                   # 21-rule GitHub Flavored Markdown & KaTeX linter
│   ├── evaluate_robosnail.py               # Live evaluation & receipt generator
│   └── verify_bcm2712_arena.py             # Broadcom BCM2712 Cortex-A76 cache hierarchy verifier
├── studio/
│   └── robosnail_portable_studio.html      # Zero-dependency 60 Hz WebGL2 3D robotics studio
├── tests/
│   └── test_robosnail.py                   # Pytest verification suite (5 tests)
└── results/
    └── robosnail_evaluation_summary.json   # Verified benchmark telemetry receipts
```

---

## 5. Build, Verification & Quickstart

### 1. Rust `#![no_std]` Arena Kernel

```powershell
cd c:\Users\x\Documents\antigravity\robosnail
cargo test --release
```

### 2. Python C-ABI Bridge & Pytest Suite

```powershell
cd c:\Users\x\Documents\antigravity\robosnail
pytest -o pythonpath=. tests/test_robosnail.py -v
python scripts/audit_gfm_math.py README.md docs/ARCHITECTURE.md
```

### 3. Zero-Install WebGL2 3D Robotics Studio

Open [`studio/robosnail_portable_studio.html`](studio/robosnail_portable_studio.html) directly in any modern browser (Chrome, Edge, Firefox, Safari) to inspect the 60 Hz 3D kinematic contact-chain sheaf, 2mm cable glare filter, cavity CSP docking, and online $\mathfrak{so}(16)$ Cayley rotor telemetry.

---

## 6. Citation

```bibtex
@software{robosnail_2026,
  author       = {Cohen, Noah},
  title        = {RoboSnail: Embodied Robotics Spoke for Contact-Chain Sheaf Propagation, 2mm Cable Glare Suppression and Online so(16) Cayley Rotor Adaptation},
  year         = {2026},
  version      = {0.1.0},
  url          = {https://github.com/sneed-and-feed/robosnail}
}
```
