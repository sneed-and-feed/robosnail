# RoboSnail: Embodied Robotics Spoke

Allocation-free `#![no_std]` Rust arena kernel, ROS 2 C-ABI bindings, and WebGL2 portable 3D simulation studio for contact-chain sheaf propagation, 2mm thin cable specular glare suppression, and online $\mathfrak{so}(16)$ Cayley rotor test-time adaptation.

Part of the **DarkSnail Multi-Project Expansion**.

Remote repository: `https://github.com/sneed-and-feed/robosnail`

## Mathematical Architecture & Verification

1. **SRAM-Resident Bare-Metal Arena (`< 2.0 MB`)**:
   - `RoboSnailArena` occupies 731,456 bytes (34.88% of 2.0 MB budget).
   - Compile-time assertion verified: `const _: () = assert!(core::mem::size_of::<RoboSnailArena>() <= 2_097_152);`.
   - Zero dynamic heap allocations during real-time streaming loops.

2. **BeltramiCallosalShell 2mm Cable Specular Glare Filter**:
   - Directional 9-tap stencil exterior-curl filter evaluates transverse curvature.
   - Rejects isotropic diffuse floor glare (100.0% rejection) while recovering 100.0% (1.0000 rate) of 2mm thin cable spikes.

3. **Online Rank-2 $\mathfrak{so}(16)$ Cayley Rotor TTT**:
   - Closed-form rank-2 Cayley transformation:
     $$R = I + \frac{2}{1 + \alpha^2} (A + A^2) \in \mathrm{SO}(16)$$
   - Exactly preserves $L_2$ norm with $0.000\mathrm{e}{+}00$ radial drift.

4. **Scale-Invariant Sheaf Cavity CSP**:
   - Computes cavity token dimension via $S_{\mathrm{tok}} = \sqrt{S_{\mathrm{ref}}^2 - A_{\mathrm{ref}}}$.
   - Solves square hollow frame docking and Jordan container containment.

5. **Kinematic Contact-Chain Sheaf**:
   - Models components as stalks in $\mathfrak{se}(2)$: $[v_x, v_y, \omega]^T$.
   - Enforces unilateral non-penetration coboundary condition $(\delta\mathbf{v})(e) \ge 0$.
   - Propagates multi-body pushes, handles container nesting co-movement, and collapses displacement upon hitting fixed wall obstacles.

6. **Portable WebGL2 3D Studio (`studio/robosnail_portable_studio.html`)**:
   - Single-file zero-dependency client-side WebGL2 studio running at 60 Hz.
   - Interactive actuation, live telemetry, and visual proof receipts.

## Build and Test Instructions

### Rust Kernel (`#![no_std]`)
```powershell
cd robosnail
cargo test --release
```

### Python Bindings & Pytest
```powershell
pytest -o pythonpath=. tests/test_robosnail.py -v
```
