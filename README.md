# PLANAR LAB

PLANAR LAB is an experimental computational-mechanics environment for planar linkage design. This build implements a four-bar mechanism workflow with interactive geometry, branch selection, cycle sampling, differentiated kinematics, bounded path-fit optimization, and portable data exports.

## Implemented in build 0.2.0

- Four-bar position analysis using circle-circle intersection.
- Open and crossed assembly branches, geometry presets, and animated input crank.
- Grashof classification, transmission angle, loop-closure residual, and coupler locus.
- Velocity and acceleration analysis from differentiated vector loop-closure equations.
- Full-cycle sampling and CSV export.
- Deterministic bounded coordinate-search optimization against simple target paths.
- SVG coupler-trace export and versioned JSON project export.
- Six workspaces: Mechanism Studio, Mathematical Laboratory, Motion & Optimization Lab, CAD Export Studio, Interactive Web Demos, Developer Hub.

## Run locally

Requires Node.js 18+.

```bash
npm install
npm run dev
```

Open the local URL printed by Vite.

## Validation

```bash
npm test
npm run build
```

## Engineering conventions

- Planar four-bar linkage; ground pivot O2=(0,0), O4=(d,0).
- Length units: millimeters.
- UI angles: degrees; trigonometric calculations: radians.
- Positive angular direction: counter-clockwise.
- The velocity and acceleration solvers are intended for regular configurations. Near a toggle/singular configuration, the determinant of the linear system approaches zero and the result is returned as unavailable.
- The current path optimizer is a deterministic bounded coordinate search. It is useful for exploratory design but does not guarantee a global optimum.

## Explicit limitations

Native STEP/DXF solid modeling, a production-grade general multibody solver, 3D rigid-body dynamics, and a compiled C++/Python engine are not implemented in this build. SVG, CSV, and JSON exports are implemented. These distinctions are intentional: the UI does not label planned capabilities as finished.
