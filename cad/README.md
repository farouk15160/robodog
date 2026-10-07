# CAD

The active source is `cad/urdf/urdf/urdf.urdf`, with its original meshes in
`cad/urdf/meshes/`. The earlier `cad/robot/onshape_export/` assembly and RS02
vendor STEP remain historical reference material.

`tools/cad_to_model.py` aggregates the export into the canonical floating base
and twelve joint bodies. It transforms mesh origins, mass centers and inertia
tensors into the same frame convention used by the URDF, IK and MuJoCo.
`robot_parameters.yaml` records source provenance and the generated mass budget.

The new approximately 7.549 kg CAD export includes tiny motor placeholders,
not twelve correctly weighted RS06 actuators. Their existing mass and inertia
must be removed before adding twelve 621 g RS06 units (7.452 kg). The base ROS
configuration is 19.719271 kg after battery, electronics and mounting
allowances. The active loaded test model adds 8.280729 kg of centered,
removable simulation ballast for an exact 28.000 kg total. The ballast is a
test load, not a CAD-derived robot component. Payload placement and several
component masses remain estimates.

The knee actuator remains on the upper leg and drives the knee through a
2:1 belt reduction: two actuator-output turns per knee turn. Pulley/belt mass
allowances do not establish geometry clearance or belt strength. The CAD mate
limits are not treated as verified mechanical stops; check actual assembly
clearance before hardware calibration.

## Regeneration

```bash
python3 tools/cad_to_model.py
python3 tools/prepare_meshes.py
ros2 run robodog_sim generate_models
python3 tools/make_doc_facts.py
```

Build/source the ROS workspace before running installed executables. Verify
URDF/MJCF mass, transforms, mesh availability, joint ranges and unobstructed hip
motion after regeneration. A model that stands can still contain a collision
that prevents locomotion.

See `NOTICE` and `cad/actuators/provenance.json` for the archived third-party
actuator geometry. RS06 specifications are sourced in
`ros2_ws/src/robodog_description/config/robstride06.yaml`.
