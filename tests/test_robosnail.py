"""Unit and integration tests for RoboSnail embodied robotics spoke."""

import os
import sys
import numpy as np
import pytest

from robosnail.arena import RoboSnailArenaBridge


def test_arena_size_and_sram_residency():
    """Verify RoboSnailArena is strictly <= 2.0 MB (2,097,152 bytes) SRAM."""
    bridge = RoboSnailArenaBridge(width=64, height=64)
    assert bridge.arena_size <= 2_097_152, f"Arena size {bridge.arena_size} exceeds 2.0 MB limit!"
    print(f"\n[PASS] Verified RoboSnailArena size: {bridge.arena_size} bytes (< 2.0 MB limit).")


def test_beltrami_callosal_shell_2mm_cable_glare_filter():
    """Verify BeltramiCallosalShell achieves 1.0000 spike recovery for 2mm cables under glare."""
    bridge = RoboSnailArenaBridge(width=64, height=64)
    w = 64
    input_frame = np.zeros((w, w), dtype=np.float32)
    glare_mask = np.zeros((w, w), dtype=np.uint8)
    true_cable_mask = np.zeros((w, w), dtype=np.float32)

    # Smooth diffuse floor glare
    cy, cx = 32.0, 32.0
    for y in range(w):
        for x in range(w):
            dy = (y - cy) / 16.0
            dx = (x - cx) / 16.0
            dist_sq = dy * dy + dx * dx
            glare_val = 0.90 * np.exp(-0.5 * dist_sq)
            input_frame[y, x] = glare_val
            if glare_val > 0.15:
                glare_mask[y, x] = 1

    # 2mm thin cable crossing horizontally at y=32 (1-2 px thick)
    for x in range(16, 48):
        true_cable_mask[32, x] = 1.0
        input_frame[32, x] = 0.15  # Dark cable on bright specular glare floor

    rec, rej = bridge.process_cable_glare(input_frame, glare_mask, true_cable_mask)
    assert rec >= 0.9999, f"Expected 1.0000 spike recovery, got {rec:.4f}"
    assert rej >= 0.95, f"Expected high glare rejection >= 0.95, got {rej:.4f}"
    print(f"\n[PASS] 2mm Cable Glare Filter: Recovery={rec:.4f}, Glare Rejection={rej:.4f}")


def test_so16_cayley_rotor_exact_zero_norm_drift():
    """Verify Online Rank-2 so(16) Cayley rotor achieves exact 0.000e+00 norm drift."""
    bridge = RoboSnailArenaBridge(width=64, height=64)
    drift = bridge.step_cayley_rotor(u_idx=0, v_idx=1, alpha=0.35)
    assert abs(drift) < 1e-6, f"Norm drift {drift} exceeds 1e-6!"
    print(f"\n[PASS] Online Rank-2 so(16) Cayley Rotor: Max norm drift={drift:.3e}")


def test_scale_invariant_sheaf_cavity_csp():
    """Verify Scale-Invariant Sheaf Cavity CSP (S_tok = sqrt(S_ref^2 - A_ref))."""
    bridge = RoboSnailArenaBridge(width=64, height=64)
    # 8x8 outer frame, area = 48 (hollow 4x4 inner cavity, area = 64 - 48 = 16 -> s_tok = 4.0)
    s_tok = bridge.solve_cavity_csp(outer_side=8.0, frame_area=48.0)
    assert abs(s_tok - 4.0) < 1e-4, f"Expected s_tok=4.0, got {s_tok}"

    # 10x10 outer frame, area = 64 (cavity area = 100 - 64 = 36 -> s_tok = 6.0)
    s_tok2 = bridge.solve_cavity_csp(outer_side=10.0, frame_area=64.0)
    assert abs(s_tok2 - 6.0) < 1e-4, f"Expected s_tok=6.0, got {s_tok2}"
    print(f"\n[PASS] Cavity CSP: S_tok={s_tok:.1f} and {s_tok2:.1f} match theory exactly.")


def test_kinematic_contact_sheaf_propagation():
    """Verify forward kinematic contact chain displacement propagation and studio UP/DOWN orientation."""
    bridge = RoboSnailArenaBridge(width=64, height=64)
    dx, dy = bridge.solve_contact_sheaf(ego_id=0, dx=1.0, dy=0.0)
    assert (dx, dy) == (1.0, 0.0)

    studio_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "studio", "robosnail_portable_studio.html")
    with open(studio_path, "r", encoding="utf-8") as f:
        html = f.read()
    assert 'onclick="actuateEgo(0, 1)">▲ UP</button>' in html
    assert 'onclick="actuateEgo(0, -1)">▼ DOWN</button>' in html
    print(f"\n[PASS] Contact Sheaf Kinematics & WebGL2 Studio UP/DOWN Orientation: (dx, dy)=({dx}, {dy})")
