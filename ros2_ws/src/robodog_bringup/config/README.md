Bring-up overrides go here. Package defaults live with the package that owns
them, so that a parameter has exactly one authoritative source:

    robodog_description/config/robot_parameters.yaml   geometry, inertia (generated)
    robodog_description/config/robstride06.yaml        actuator data and limits
    robodog_control/config/control.yaml                rates and impedance gains
    robodog_control/config/gaits.yaml                  gait library
    robodog_hardware/config/robstride_bus.yaml         CAN topology, motor map
    robodog_perception/config/nuwa_hp60c.yaml          camera intrinsics, range
    robodog_sim/config/simulation.yaml                 physics fidelity
    robodog_web/config/web.yaml                        GUI panels and limits

`robot_parameters.yaml` selects the actuator file through `actuator_config`.
The current RS06 configuration includes the external knee ratio and efficiency.

Startup flags `auto_enable:=auto auto_stand:=auto` enable and stand simulation,
but leave real CAN hardware disabled. The RS06 backend refuses normal enable or
motion on joints that have not been marked `calibrated: true` after physical
commissioning. Real IMU/base feedback is not integrated: hardware travel commands
are rejected while base feedback is unavailable.

The GUI binds localhost and validates WebSocket origins by default. Changing
network exposure is a separate deployment decision, not a requirement for local
simulation use.
