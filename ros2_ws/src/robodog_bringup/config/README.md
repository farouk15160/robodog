Bring-up overrides go here. Package defaults live with the package that owns
them, so that a parameter has exactly one authoritative source:

    robodog_description/config/robot_parameters.yaml   geometry, inertia (generated)
    robodog_description/config/robstride02.yaml        actuator data and limits
    robodog_control/config/control.yaml                rates and impedance gains
    robodog_control/config/gaits.yaml                  gait library
    robodog_hardware/config/robstride_bus.yaml         CAN topology, motor map
    robodog_perception/config/nuwa_hp60c.yaml          camera intrinsics, range
    robodog_sim/config/simulation.yaml                 physics fidelity
    robodog_web/config/web.yaml                        GUI panels and limits
