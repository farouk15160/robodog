# CAD

Input to the model pipeline. Nothing here is edited by hand as part of the
software build; `tools/cad_to_model.py` reads it and produces
`ros2_ws/src/robodog_description/config/robot_parameters.yaml`, from which the
URDF, the MuJoCo model, the inverse kinematics and the documentation are all
derived.

```
cad/
  robot/onshape_export/      the robot assembly, exported from Onshape
    urdf/urdf.urdf           85 part-links, 12 revolute joints, flat tree
    meshes/*.stl             per-part geometry, ~41 MB
  actuators/
    robstride02_official.step   vendor CAD for the actuator
    provenance.json             where it came from, and its SHA-256
```

## About the export

The Onshape export is a **flat** tree: every part is its own link, welded
together by fixed joints, and the assembly is saved at an arbitrary mate pose
with the legs splayed. It is not usable as a robot description directly. The
pipeline turns it into a canonical 13-body model — see
`docs/robodog_architecture.pdf` §3.1 for the six steps, and note two things it
does that matter:

- **The mate limits are ignored.** They are modelling artefacts (for example
  `dof_fl0` spans `[-0.704, 0.866]`, asymmetric and different on every leg) and
  describe how the assembly was posed, not what the mechanism can do.
- **The actuators are substituted.** The assembly was drawn around MyActuator
  RMD-X8 V3 parts at 760.8 g each; the pipeline replaces their mass properties
  with the ROBSTRIDE02's 380 g and swaps the visual mesh for a generated
  envelope of the correct 78.5 × 41.5 mm size.

## Regenerating after a CAD change

```bash
python3 tools/cad_to_model.py        # geometry, inertia, visual transforms
python3 tools/prepare_meshes.py      # decimate visual meshes, 41 MB -> 2.3 MB
ros2 run robodog_sim generate_models # rebuild the MuJoCo models
cd docs && make                      # rebuild the figures and the PDF
```

`cad_to_model.py` prints a verification report. The canonical chain should
reproduce the CAD joint positions to within a few micrometres and the four legs
should agree with each other to under 2 µm; if either number grows, the
assembly has changed in a way the pipeline's assumptions did not expect.

## Third-party geometry

Two files here are not ours. See `NOTICE` in the repository root for their
origin and terms. `provenance.json` records the exact source URL and a SHA-256
of the RS02 STEP file as downloaded, so it can be verified or re-fetched.
