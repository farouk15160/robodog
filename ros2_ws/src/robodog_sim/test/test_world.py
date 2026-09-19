"""
Test-world geometry.

The house docstring quotes specific clearances; these measure them from the
spec. A careless edit to a position turns a designed challenge into either a
wall or an open field, and nothing else in the system would notice.
"""
import math

import pytest

from robodog_sim.world_check import (footprint, narrow_gaps, overlapping_pairs,
                                     polygon_gap, waypoint_clearance,
                                     waypoints_inside_objects, z_range)
from robodog_sim.world_spec import DOOR_W, World
from robodog_sim.worlds.house import WORLDS

# robot dimensions the world is designed against
FOOT_SPAN_M = 0.289
BODY_WIDTH_M = 0.240
STANCE_HEIGHT_M = 0.320


@pytest.fixture(scope="module", params=sorted(WORLDS))
def world(request) -> World:
    return WORLDS[request.param]()


def by_name(world: World, name: str):
    return next(p for p in world.prims if p.name == name)


def gap_between(world: World, a: str, b: str) -> float:
    return polygon_gap(footprint(by_name(world, a)), footprint(by_name(world, b)))


# --------------------------------------------------------------------------- #
# structural
# --------------------------------------------------------------------------- #
def test_no_overlapping_geometry(world):
    """MuJoCo resolves interpenetration by pushing bodies apart at the first
    step, which silently moves the test environment out from under you."""
    ov = overlapping_pairs(world)
    assert not ov, "\n".join(f"{a} <-> {b} by {-d*1000:.0f} mm" for a, b, d in ov)


def test_no_waypoint_is_inside_an_object(world):
    assert not waypoints_inside_objects(world)


def test_every_object_is_inside_the_floor_plan(world):
    x0, y0, x1, y1 = world.bounds
    for p in world.prims:
        fp = footprint(p)
        assert fp[:, 0].min() >= x0 - 0.2 and fp[:, 0].max() <= x1 + 0.2, p.name
        assert fp[:, 1].min() >= y0 - 0.2 and fp[:, 1].max() <= y1 + 0.2, p.name


def test_nothing_floats_or_is_buried(world):
    for p in world.prims:
        lo, hi = z_range(p)
        assert lo > -0.06, f"{p.name} extends {lo*1000:.0f} mm below the floor"
        assert hi > 0.0, f"{p.name} is entirely below the floor"


def test_waypoints_are_unique_and_include_a_spawn(world):
    names = [wp.name for wp in world.waypoints]
    assert len(names) == len(set(names))
    assert "start" in names


# --------------------------------------------------------------------------- #
# the house: the clearances its docstring claims
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def house() -> World:
    return WORLDS["house"]()


def test_doorways_are_full_width(house):
    """Every door gap must be the nominal 0.90 m: a doorway narrowed by an
    edit would turn a routine traverse into an unintended squeeze test."""
    sills = [p for p in house.prims if p.tag == "door"]
    assert len(sills) == 6, [p.name for p in sills]
    for s in sills:
        span = max(s.size[0], s.size[1])
        assert span == pytest.approx(DOOR_W, abs=1e-6), f"{s.name} is {span:.3f} m"


def test_door_sills_are_low_but_present(house):
    for s in [p for p in house.prims if p.tag == "door"]:
        lo, hi = z_range(s)
        assert 0.01 < hi < 0.05, f"{s.name} sill is {hi*1000:.0f} mm"


@pytest.mark.parametrize("a,b,want", [
    ("sofa", "bookshelf", 0.450),        # living-room gap
    ("hall_block_w", "hall_block_e", 0.600),   # hall chicane
    ("pole_a", "pole_b", 0.430),         # kitchen slalom
])
def test_designed_gaps_have_their_documented_width(house, a, b, want):
    got = gap_between(house, a, b)
    assert got == pytest.approx(want, abs=0.002), f"{a}<->{b} is {got*1000:.0f} mm"


def test_designed_gaps_are_narrower_than_the_foot_span(house):
    """If a gap were wider than the robot's stance it would not be a test."""
    for a, b in [("sofa", "bookshelf"), ("pole_a", "pole_b")]:
        assert gap_between(house, a, b) < FOOT_SPAN_M + 0.20


def test_designed_gaps_still_admit_the_body(house):
    for a, b in [("sofa", "bookshelf"), ("pole_a", "pole_b"),
                 ("hall_block_w", "hall_block_e")]:
        assert gap_between(house, a, b) > BODY_WIDTH_M


def test_tables_can_be_walked_under(house):
    """Clearance must exceed the stance height, or duck-under is not a choice
    the planner can make -- it is simply blocked."""
    for name in ["coffee_table_top", "dining_table_top", "workbench_top"]:
        lo, _ = z_range(by_name(house, name))
        assert lo > 0.33, f"{name} clears only {lo*1000:.0f} mm"


def test_stair_flights_match_their_documented_geometry(house):
    low = [p for p in house.prims if p.name.startswith("stairs_low_step")]
    steep = [p for p in house.prims if p.name.startswith("stairs_steep_step")]
    assert len(low) == 4 and len(steep) == 3
    assert z_range(low[0])[1] == pytest.approx(0.08, abs=1e-6)
    assert z_range(low[-1])[1] == pytest.approx(0.32, abs=1e-6)
    assert z_range(steep[0])[1] == pytest.approx(0.12, abs=1e-6)
    assert z_range(steep[-1])[1] == pytest.approx(0.36, abs=1e-6)


def test_stair_rise_is_within_the_leg_lift_budget(house):
    """The RISE between consecutive treads is what the leg must clear. Treads
    are modelled as solid blocks from the floor up (so no foot can catch under
    an overhang), which is why the absolute tread height is not the rise."""
    for flight in ("stairs_low", "stairs_steep"):
        tops = [z_range(p)[1] for p in house.prims
                if p.name.startswith(f"{flight}_step")]
        rises = [tops[0]] + [b - a for a, b in zip(tops, tops[1:])]
        assert max(rises) <= 0.15, f"{flight} rises {max(rises)*1000:.0f} mm"
        assert len(set(round(r, 6) for r in rises)) == 1, f"{flight} rise is uneven"


def test_ramp_angle_is_near_the_friction_limit(house):
    r = by_name(house, "ramp_12deg")
    assert math.degrees(abs(r.rpy[1])) == pytest.approx(12.0, abs=0.1)


def test_stair_landings_are_reachable_waypoints(house):
    """A landing waypoint must declare the height it stands at, otherwise the
    clearance check reads the landing itself as an obstacle."""
    for name in ["stairs_low_top", "stairs_steep_top"]:
        wp = next(w for w in house.waypoints if w.name == name)
        assert wp.z > 0.0
        assert waypoint_clearance(house, wp)[0] > 0.0


def test_approach_lanes_to_the_stairs_are_clear(house):
    """Each flight needs room in front of it or the test is about squeezing
    into position rather than about climbing."""
    for name in ["stairs_low_foot", "stairs_steep_foot", "ramp_foot"]:
        wp = next(w for w in house.waypoints if w.name == name)
        assert waypoint_clearance(house, wp)[0] >= 0.25, name


def test_all_five_rooms_are_represented(house):
    assert house.size == (12.0, 9.0)
    tags = {p.tag for p in house.prims}
    assert {"wall", "door", "furniture", "obstacle", "stair", "ramp", "soft"} <= tags


def test_movable_objects_have_mass(house):
    movable = [p for p in house.prims if p.movable]
    assert movable, "nothing is pushable: contact recovery cannot be exercised"
    for p in movable:
        assert p.mass > 0.0, p.name


def test_narrow_gap_report_is_sorted_and_positive(house):
    gaps = narrow_gaps(house)
    assert gaps and all(g.distance > 0 for g in gaps)
    assert gaps == sorted(gaps, key=lambda g: g.distance)
