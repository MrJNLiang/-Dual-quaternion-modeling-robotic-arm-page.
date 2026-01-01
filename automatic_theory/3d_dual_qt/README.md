# Dual Quaternion Arm Lab

This single-page app visualizes a 4-DOF (3R + 1P) serial arm with dual quaternion (qt) kinematics.
It links formulas, UI values, and the 3D scene so each parameter can be inspected and edited.

## Run

Option 1: local server (recommended)

```bash
python3 -m http.server 8000
```

Then open `http://localhost:8000/3d_dual_qt/index.html`.

Option 2: open the file directly

Open `3d_dual_qt/index.html` in a browser. This build uses local `vendor/` scripts, so it works without a server.

## What is included

- 4-DOF chain with adjustable link lengths, joint angles, prismatic displacement, and joint axes.
- Dual quaternion qt form for each joint, intermediate chain results, and translation extraction.
- Multiplicative error model with editable error dual quaternion.
- Target pose input (translation + axis-angle or direct qt), plus two trajectories in 3D.
- Jacobian columns with geometric meaning and a finite-difference verification button.
- H-infinity formula block and a computed qdot from the displayed controller structure.

## Notes

- The "ScLERP" option uses a normalized DQ lerp for clarity and stability.
- Joint axes are always normalized; custom axes auto-normalize.
