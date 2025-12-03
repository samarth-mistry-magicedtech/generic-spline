# Generic Cable System (UPM)

Unity Spline-Based Procedural Cable / Wire Generator

---

## Overview
The GenericCable system is a fully procedural, highly configurable cable/wire generator built on top of Unity’s Splines package. It automatically generates a smooth spline between two points and builds a UV-mapped tubular mesh around it.

### Features
- **Automatic cable routing** between two transforms
- **Natural sag/dip simulation**
- **Basic obstacle avoidance** using physics linecasts
- **Adjustable** thickness, smoothness, resolution, and material
- **Auto-updates** in Editor and Play mode
- **Procedural mesh** generation with normals + UVs
- **Debug Gizmos** for visualizing the spline
- **VR-ready** usage across simulation, training, robotics, medical ECG wires, etc.

> Screenshot placeholder (overview)
>
> ![Cable Overview](Documentation~/images/image1.png)

---

## System Architecture

### Auto-Required Unity Components
The script adds and manages:
- **SplineContainer** — stores the Bezier spline
- **MeshFilter** — holds the generated mesh
- **MeshRenderer** — displays the cable mesh

### Processing Pipeline
1. Detect/resolve endpoints
2. Compute midpoint with sag/dip
3. Apply optional obstacle avoidance
4. Generate a 3-knot spline (Start → Mid → End)
5. Sample spline positions
6. Build UV-mapped tubular mesh
7. Update mesh + bounds
8. Render cable immediately

---

## 1) Cable Endpoints

### Fields
- `startPoint` → Beginning of the cable
- `endPoint` → End of the cable
- `useChildEndpoint` → If true, a named child (e.g., "Tip") is used instead
- `childName` → Child transform name for finer control

### Behavior
- Endpoints are resolved automatically; if the child does not exist, parent transform is used
- Cable updates automatically when endpoints move/rotate
- Works in Edit Mode & Play Mode

### Use cases
- ECG electrodes (Tip transforms)
- Plugs, connectors, VR controllers
- Wires from tools, gadgets, panels

> Screenshot placeholder (endpoints)
>
> ![Endpoints](Documentation~/images/image1.png)

---

## 2) Cable Sag / Dip Simulation

### Configuration
- `sagStrength` → 0 = straight, 1 = heavy sag
- `sagProfile` → AnimationCurve shaping the sag along spline
- `dipAxis` → Direction the cable droops (default: down)
- `dipReference` → Transform whose orientation defines dip direction
- `useReferenceDipAxis` → Enables reference-based sagging

### How It Works
A midpoint is created between start and end points:

```
Vector3 mid = (p0 + p3) * 0.5f
             - dipAxis.normalized * (distance * sagStrength);
```

Then `sagProfile` is applied along the sampled spline.

### Use cases
- Natural hanging wires
- Moving sag direction for VR headset cables (head rotation)
- Dynamic machine parts with changing orientation

---

## 3) Obstacle Avoidance (Simple Mode)

### Features
- Performs a `Physics.Linecast` from start → end
- When an obstacle is detected, the midpoint is pushed along the collider normal
- `clearanceRadius` controls push distance
- `ignoreCableLayer` prevents detecting the cable itself
- Uses `obstacleLayers` mask for filtering

### Suitable For
- Cables routing around walls, machines, drawers
- Wires avoiding the player body
- Simple scene-aware cable shaping

### Limitations (By Design)
- Only adjusts the midpoint
- Not a full routing system
- Lightweight and safe for real-time updates

---

## 4) Sampling & Quality Settings

### Fields
- `segments` → Total spline samples (higher = smoother cable)

### Sampling Steps
- Normalize parameter `t` from 0 → 1
- Evaluate spline at each `t`
- Convert spline local point to world space
- Apply `sagProfile`
- Store all points for mesh generation

Higher `segments` = smoother cable but more vertices. Recommended: **64–96** for VR cables.

---

## 5) Procedural Mesh Generation

### Adjustable Mesh Properties
- `cableRadius` → Cylinder thickness
- `radialSegments` → Vertices per ring
- `cableMaterial` → Material applied via MeshRenderer

### Mesh Construction Flow
- For each sampled point, compute tangent, normal, binormal
- Generate circular ring of vertices
- Build triangles between consecutive rings
- Apply normals, UVs, bounds

### UV Layout
- U wraps around the cable
- V flows along cable length for seamless cylindrical textures (stripes/spirals/procedural shaders)

---

## 6) Auto-Update Rules

### Editor
- Updates via `OnValidate()` and editor-only refresh
- Reacts when variables change

### Runtime
- Updates only when endpoint positions change
- Controlled by `updateInPlayMode` and cached positions

### Benefits
- Efficient for moving/animated endpoints
- Ideal for VR hands, robots, tools, machines

---

## 7) Debug Gizmos
- Draws the spline in the Scene View (yellow line)
- Appears even when not playing
- Useful for level designers and debugging collision avoidance

---

## Installation

### A) Add as Local UPM Package (Recommended)
1. Open Unity → Window → Package Manager
2. Click **+** → **Add package from disk…**
3. Select `Packages/com.magicedtech.generic-spline/package.json`

### B) Import .unitypackage (Legacy Option)
If you have `GenericCable.unitypackage`:
1. Assets → Import Package → Custom Package…
2. Select `GenericCable.unitypackage`
3. Import the script(s) and assets

---

## Quick User Guide

### 1. Basic Setup
1. Create an empty GameObject (e.g., `Cable`)
2. Add the `GenericCable` component
3. Assign:
   - `Start Point` → any Transform
   - `End Point` → any Transform
4. Assign a Material on the MeshRenderer
   - Use an existing material, or create a new one (Right-click → Create → Material)
   - The cable will appear as soon as a material is assigned

### 2. Core Modifications
- **Cable Curve / Sag**
  - `sagStrength` — 0 = straight, 1 = heavy sag
  - `sagProfile` — curve shaping along length
- **Dip Direction**
  - `dipAxis` — direction the cable sags (default: Down)
  - `dipReference` + `useReferenceDipAxis` — make sag follow a rotating object (e.g., VR tools/machines)

### 3. Cable Shape Settings
- **Smoothness (segments)** — more points = smoother, heavier
- **Thickness (cableRadius)** — cable thickness
- **Roundness (radialSegments)** — vertices per ring

### 4. Obstacle Avoidance
- `useObstacleAvoidance` — enable simple linecast avoidance
- `clearanceRadius` — how far the cable stays from obstacles
- `ignoreCableLayer` — ignore the cable itself
- Good for walls, machines, player body in VR, tool surfaces

### 5. Optional: Child Endpoint
If your object has a `"Tip"` child:
- Enable `useChildEndpoint`
- Set `childName` if different
- Great for connectors, VR tool tips, and ECG electrode tips

> Screenshot placeholder (inspector/setup)
>
> ![Inspector Setup](Documentation~/images/image1.png)

---

## Source Code
A fully improved GenericCable Package is included with this UPM. If you maintain an external distribution, include a link here.

`https://drive.google.com/file/d/12PLcBey8_psdPjJmireBpkSyvVvCKP5j/view?usp=drive_link`

---

## Should we add starter materials?
Yes—recommended. Two options:
- **Samples~/** — Provide example materials and a sample scene users can import via Package Manager (best practice for examples).
- **Runtime/Materials/** — Include a minimal neutral PBR material so the cable renders immediately.

If you want, I can add:
- `Samples~/Basic Materials` with a simple metallic rubber cable material and a demo scene
- `Runtime/Materials/Cable_Default.mat` as a default assignment

---

## Folder Suggestions
- `Runtime/` — scripts, core assets
- `Editor/` — editor tooling and gizmos
- `Documentation~/images/` — documentation images (excluded from build)
- `Samples~/` — optional samples for users to import

---

## License & Changelog
Add a `LICENSE` and `CHANGELOG.md` if distributing publicly.
