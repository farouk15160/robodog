"""
Five-room test house, 12 x 9 m.

            x=0          x=4.5      x=8        x=12
     y=9     +------------+----------+-----------+
             | R2 bedroom | R4 hall  | R5 shop   |
     y=4.5   +-----[D]----+---[D]----+----[D]----+
             | R1 living  |    R3 kitchen        |
     y=0     +-----[D]----+----------------------+

Every indoor locomotion problem the robot has to solve is represented, and each
is placed with enough approach room to be attempted in isolation rather than
only as part of a cluttered route.

  doorways     0.90 m openings with a 20 mm sill to step over
  narrow gaps  0.45 m (sofa/bookshelf), 0.43 m (kitchen slalom), 0.60 m (hall
               chicane), against a 0.289 m foot span and 0.240 m body width
  stairs       4 x 80 mm (realistic first target) and 3 x 120 mm (stretch),
               both with 1.5 m of clear approach
  ramp         12 deg, about where 0.9 friction stops holding a static stance
  under-table  0.40-0.46 m clearance versus a 0.32 m stance: duck-under is a
               choice the planner can make, not something merely blocked
  debris       mixed-height blocks for foothold selection
  movable      free bodies that topple, so contact estimation and recovery are
               exercised rather than assumed

Geometry is checked, not asserted: robodog_sim/world_check.py measures every
clearance quoted above from this spec, and test/test_world.py fails if an edit
turns a designed challenge into a wall or an open field.
"""
from __future__ import annotations

from ..world_spec import (DOOR_W, Prim, World, Waypoint, balance_beam, gap_course,
                          ramp, stairs, stairs_down, stepping_stones, table,
                          terrain_tiles, wall)

W, H = 12.0, 9.0
MID_X, MID_Y = 4.5, 4.5
SHOP_X = 8.0


def build() -> World:
    w = World("house", size=(W, H))

    # ---------------- shell ----------------
    w.add(*wall("out_s", 0, 0, W, 0))
    w.add(*wall("out_e", W, 0, W, H))
    w.add(*wall("out_n", W, H, 0, H))
    w.add(*wall("out_w", 0, H, 0, 0))

    # ---------------- interior walls; door spans are measured ALONG each wall
    w.add(*wall("div_living_kitchen", MID_X, 0, MID_X, MID_Y, doors=[(1.4, 1.4 + DOOR_W)]))
    w.add(*wall("div_living_bed", 0, MID_Y, MID_X, MID_Y, doors=[(1.5, 1.5 + DOOR_W)]))
    w.add(*wall("div_bed_hall", MID_X, MID_Y, MID_X, H, doors=[(1.5, 1.5 + DOOR_W)]))
    w.add(*wall("div_kitchen_hall", MID_X, MID_Y, SHOP_X, MID_Y, doors=[(0.9, 0.9 + DOOR_W)]))
    # Placed at the far east so the doorway does not open straight onto the
    # stair flight; the checker enforces the resulting 0.5 m approach.
    w.add(*wall("div_kitchen_shop", SHOP_X, MID_Y, W, MID_Y, doors=[(3.0, 3.0 + DOOR_W)]))
    w.add(*wall("div_hall_shop", SHOP_X, MID_Y, SHOP_X, H, doors=[(2.5, 2.5 + DOOR_W)]))

    # ================= R1 living (0-4.5, 0-4.5) =================
    # Sofa east face 0.975, bookshelf west face 1.425 -> a 0.450 m gap.
    w.add(Prim("sofa", "box", (0.55, 2.60, 0.22), (0.85, 1.80, 0.44), tag="soft", friction=0.7))
    w.add(Prim("bookshelf", "box", (1.60, 3.30, 0.55), (0.35, 1.10, 1.10), tag="furniture"))
    w.add(*table("coffee_table", 3.00, 1.70, w=1.00, d=0.60, h=0.40, leg=0.06))
    w.add(Prim("tv_unit", "box", (4.10, 3.40, 0.25), (0.40, 1.30, 0.50), tag="furniture"))
    w.add(Prim("crate_a", "box", (2.10, 3.90, 0.14), (0.28, 0.28, 0.28),
               tag="obstacle", movable=True, mass=1.2))
    w.add(Prim("crate_b", "box", (2.60, 4.05, 0.11), (0.22, 0.22, 0.22),
               tag="obstacle", movable=True, mass=0.8))

    # ================= R2 bedroom (0-4.5, 4.5-9) =================
    w.add(Prim("bed", "box", (1.15, 7.60, 0.18), (2.00, 1.45, 0.36), tag="soft", friction=0.7))
    w.add(Prim("wardrobe", "box", (4.05, 8.15, 0.60), (0.60, 1.30, 1.20), tag="furniture"))
    w.add(*table("nightstand", 2.60, 8.40, w=0.45, d=0.40, h=0.42, leg=0.05))
    # 12 mm rug: low enough that a naive controller ignores it and a careful one
    # does not. Useful for checking contact-estimator sensitivity.
    w.add(Prim("rug", "box", (2.30, 5.90, 0.006), (2.20, 1.60, 0.012),
               tag="soft", friction=0.6))
    # chest east face 2.85, wardrobe west face 3.75 -> a 0.90 m gap (easy route)
    w.add(Prim("chest", "box", (2.55, 7.05, 0.22), (0.60, 0.45, 0.44), tag="furniture"))

    # ================= R3 kitchen (4.5-12, 0-4.5) =================
    w.add(Prim("counter_island", "box", (6.60, 1.15, 0.28), (2.20, 0.75, 0.56), tag="furniture"))
    w.add(Prim("counter_run", "box", (11.60, 2.20, 0.30), (0.55, 3.00, 0.60), tag="furniture"))
    w.add(*table("dining_table", 6.60, 3.30, w=1.40, d=0.80, h=0.46, leg=0.06))
    for i, (cx, cy) in enumerate([(5.45, 3.30), (7.75, 3.30)]):
        w.add(Prim(f"chair{i}", "box", (cx, cy, 0.23), (0.42, 0.42, 0.46), tag="furniture"))
    w.add(Prim("bin", "cylinder", (9.20, 0.70, 0.30), (0.20, 0.60), tag="obstacle",
               movable=True, mass=2.0))
    # poles 0.55 m centre-to-centre, radius 0.06 -> a 0.430 m slalom gate
    w.add(Prim("pole_a", "cylinder", (9.60, 2.45, 0.60), (0.06, 1.20), tag="obstacle"))
    w.add(Prim("pole_b", "cylinder", (9.60, 3.00, 0.60), (0.06, 1.20), tag="obstacle"))

    # ================= R4 hall (4.5-8, 4.5-9) =================
    # Deliberately sparse: the room for gait tuning at speed, with one gate.
    w.add(Prim("hall_step", "box", (6.25, 5.60, 0.045), (1.60, 0.40, 0.09), tag="stair"))
    # block faces at 5.95 and 6.55 -> a 0.600 m gate
    w.add(Prim("hall_block_w", "box", (5.70, 7.60, 0.30), (0.50, 0.50, 0.60), tag="obstacle"))
    w.add(Prim("hall_block_e", "box", (6.80, 7.60, 0.30), (0.50, 0.50, 0.60), tag="obstacle"))
    w.add(Prim("hall_crate", "box", (7.40, 6.30, 0.16), (0.32, 0.32, 0.32),
               tag="obstacle", movable=True, mass=1.5))

    # ================= R5 workshop (8-12, 4.5-9) =================
    # Three climbing tests side by side, all climbing +x from a common approach
    # lane at x < 9.4, reachable from the doorway at y 7.0-7.9.
    w.add(*stairs("stairs_low", 9.20, 5.30, steps=4, rise=0.08, run=0.26,
                  width=1.10, landing=0.70))
    w.add(*stairs("stairs_steep", 9.40, 6.85, steps=3, rise=0.12, run=0.30,
                  width=1.00, landing=0.60))
    w.add(ramp("ramp_12deg", 10.15, 8.20, length=1.50, width=0.90, angle_deg=12.0))
    w.add(*table("workbench", 11.45, 6.05, w=0.50, d=1.20, h=0.50, leg=0.07))
    # Debris sits in the approach lane, so reaching either flight means either
    # crossing it or taking the 0.50 m gap to its east.
    for i, (dx, dy, dz) in enumerate([(0.00, 0.00, 0.05), (0.32, 0.14, 0.09),
                                      (0.16, 0.38, 0.04), (0.48, 0.44, 0.07),
                                      (-0.20, 0.30, 0.11), (0.62, 0.08, 0.06)]):
        w.add(Prim(f"debris{i}", "box", (8.50 + dx, 6.10 + dy, dz / 2.0),
                   (0.22, 0.22, dz), tag="obstacle"))

    # ---------------- waypoints ----------------
    w.waypoints = [
        Waypoint("start", (3.00, 0.70), 0.0, "spawn, living room"),
        Waypoint("living_gap", (1.20, 3.30), 1.5708, "0.45 m gap, sofa to bookshelf"),
        Waypoint("door_living_kitchen", (4.50, 1.85), 0.0, "0.90 m doorway + sill"),
        Waypoint("under_table", (6.60, 3.30), 0.0, "0.40 m clearance, duck-under"),
        Waypoint("kitchen_slalom", (9.60, 2.72), 1.5708, "0.43 m gate between poles"),
        Waypoint("door_kitchen_hall", (5.85, 4.50), 1.5708, "doorway to the hall"),
        Waypoint("hall_run", (6.25, 6.60), 1.5708, "open run for gait tuning"),
        Waypoint("hall_chicane", (6.25, 7.60), 1.5708, "0.60 m gate"),
        Waypoint("door_hall_shop", (8.00, 7.45), 0.0, "doorway to the workshop"),
        Waypoint("stairs_low_foot", (8.80, 5.30), 0.0, "4 x 80 mm stair, base"),
        Waypoint("stairs_low_top", (10.60, 5.30), 0.0, "stair landing", z=0.32),
        Waypoint("stairs_steep_foot", (8.95, 7.15), 0.0, "3 x 120 mm stretch case"),
        Waypoint("stairs_steep_top", (10.60, 6.85), 0.0, "steep landing", z=0.36),
        Waypoint("ramp_foot", (9.10, 8.20), 0.0, "12 deg ramp, base"),
        Waypoint("debris", (8.25, 5.75), 0.0, "mixed-height foothold field"),
        Waypoint("bedroom", (1.80, 5.90), 1.5708, "rug crossing"),
        Waypoint("door_kitchen_shop", (11.45, 4.50), 1.5708, "doorway, kitchen to shop"),
    ]
    return w


def build_flat() -> World:
    """Open-air proving ground, 24 x 24 m, spawn at the centre.

    Eight independent lanes radiating from the origin, so each locomotion
    problem can be attempted on its own without first crossing the others.
    The house is for navigation; this is for gait and terrain work, and gait
    work should not be debugged against furniture at the same time.

        +x  measured run      flat, 1 m markers, for velocity tuning
        -x  step ladder       single steps 40 -> 200 mm
        +y  stairs            6 x 80 mm up, landing, 6 x 80 mm down
                              and a 4 x 130 mm flight beside it
        -y  slopes            5, 10, 15, 20 degrees
        NE  rough terrain     196 tiles, 12-70 mm, seeded
        NW  stepping stones   discrete pillars with gaps
        SE  gap course        platforms split by 100 -> 280 mm gaps
        SW  balance beam      220 mm wide, plus a slalom
    """
    w = World("flat", size=(24.0, 24.0), centred=True)

    # ---------------- +x : measured run ----------------
    # A clean lane with metre markers. Velocity tracking is measured here, not
    # inferred from a gait that is also fighting terrain.
    for d in range(1, 9):
        w.add(Prim(f"mark_{d}m", "box", (float(d), 0.0, 0.004), (0.05, 1.60, 0.008),
                   tag="target"))
    w.add(Prim("run_end", "box", (9.0, 0.0, 0.15), (0.12, 1.80, 0.30), tag="obstacle"))

    # ---------------- -x : step ladder ----------------
    # One step at a time, each taller than the last. 200 mm is past the point
    # where body clearance rather than leg lift becomes the limit.
    for i, h in enumerate((0.04, 0.08, 0.12, 0.16, 0.20)):
        w.add(Prim(f"step_{int(h*1000)}mm", "box", (-2.0 - i * 1.30, 0.0, h / 2.0),
                   (0.60, 1.70, h), tag="stair"))

    # ---------------- +y : stairs up, landing, stairs down ----------------
    w.add(*stairs("stair_up", -1.60, 3.40, steps=6, rise=0.08, run=0.28,
                  width=1.30, landing=1.00))
    # Starts exactly where the landing ends: a gap here would be a hole at
    # 0.48 m, which is not the test anyone intended.
    w.add(*stairs_down("stair_dn", 1.08, 3.40, steps=6, rise=0.08, run=0.28,
                       width=1.30))
    w.add(*stairs("stair_steep", -1.40, 5.60, steps=4, rise=0.13, run=0.30,
                  width=1.10, landing=0.80))

    # ---------------- -y : slopes ----------------
    for i, deg in enumerate((5.0, 10.0, 15.0, 20.0)):
        w.add(ramp(f"slope_{int(deg)}deg", 2.20, -3.00 - i * 1.70,
                   length=2.20, width=1.30, angle_deg=deg))

    # ---------------- NE : rough terrain ----------------
    w.add(*terrain_tiles("rough", 5.60, 5.60, nx=14, ny=14, pitch=0.30,
                         tile=0.28, max_h=0.07, seed=7))

    # ---------------- NW : stepping stones ----------------
    w.add(*stepping_stones("stone", -7.60, 4.20, n=8, pitch=0.44, size=0.24,
                           height=0.11, jitter=0.04, seed=11))

    # ---------------- SE : gap course ----------------
    w.add(*gap_course("gap", 4.40, -6.20, gaps=(0.10, 0.16, 0.22, 0.28),
                      platform=0.70, width=1.40, height=0.12))

    # ---------------- SW : balance beam and slalom ----------------
    w.add(*balance_beam("beam", -5.20, -4.20, length=2.60, width=0.22, height=0.16))
    for i in range(5):
        w.add(Prim(f"slalom_{i}", "cylinder", (-8.20 + i * 0.90,
                                               -6.60 + (0.40 if i % 2 else -0.40), 0.35),
                   (0.06, 0.70), tag="obstacle"))

    w.waypoints = [
        Waypoint("start", (0.0, 0.0), 0.0, "spawn, open in every direction"),
        Waypoint("run", (4.0, 0.0), 0.0, "measured run, 1 m markers"),
        Waypoint("steps", (-1.2, 0.0), 3.1416, "40-200 mm step ladder"),
        Waypoint("stairs_foot", (-2.4, 3.40), 0.0, "6 x 80 mm, up then down"),
        Waypoint("stairs_top", (0.90, 3.40), 0.0, "landing", z=0.48),
        Waypoint("stairs_steep", (-2.3, 5.60), 0.0, "4 x 130 mm stretch case"),
        Waypoint("slopes", (0.6, -3.00), 0.0, "5, 10, 15 and 20 degrees"),
        Waypoint("rough", (3.10, 5.60), 0.0, "196-tile rough terrain"),
        Waypoint("stones", (-8.40, 4.20), 0.0, "stepping stones"),
        Waypoint("gaps", (3.80, -6.20), 0.0, "100-280 mm gaps"),
        Waypoint("beam", (-7.20, -4.20), 0.0, "220 mm balance beam"),
        Waypoint("slalom", (-9.20, -6.60), 0.0, "pole slalom"),
    ]
    return w


WORLDS = {"house": build, "flat": build_flat}
