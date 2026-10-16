"""Coordinate and JSON-contract tests independent of the planning algorithm."""

from datetime import datetime, timezone

import pytest
from pydantic import ValidationError
from pyproj import Geod
from shapely import from_wkt
from shapely.geometry import Point

from uav_planner.v2.coordinates import CoordinateError, CoordinateFrame, convert_coordinates
from uav_planner.v2.models import (
    Airway, AircraftProfile, Bounds3D, CoordinateRequest, CoordinateSpec,
    HeightCorrection, Scene, Task, TimedTrajectory,
)
from uav_planner.v2.profiles import default_profile


AT = datetime(2026, 10, 16, 4, tzinfo=timezone.utc)


def spec(**updates):
    return CoordinateSpec(origin_lon=104.78, origin_lat=29.34, origin_alt_msl=300, **updates)


def test_always_xy_east_north_and_reference_origin():
    frame = CoordinateFrame(spec())
    local = frame.to_local("POINT Z (104.78 29.34 320)", "MSL")
    assert local.coords[0] == pytest.approx((0, 0, 20), abs=1e-8)
    east = frame.to_local("POINT Z (104.781 29.34 320)", "MSL")
    north = frame.to_local("POINT Z (104.78 29.341 320)", "MSL")
    assert east.x > 90 and abs(east.y) < 0.01
    assert north.y > 100 and abs(north.x) < 0.01


@pytest.mark.parametrize("wkt", [
    "POINT Z (104.781 29.341 320)",
    "LINESTRING Z (104.78 29.34 320, 104.781 29.341 340)",
    "LINESTRING Z (104.78 29.34 320, 104.78 29.34 340)",
    "POLYGON ((104.78 29.34, 104.781 29.34, 104.781 29.341, 104.78 29.34))",
])
def test_round_trip_preserves_vertices_and_dimensionality(wkt):
    frame = CoordinateFrame(spec())
    expected = from_wkt(wkt)
    actual = from_wkt(frame.to_wgs84(frame.to_local(wkt, "MSL")))
    assert actual.equals_exact(expected, tolerance=1e-10)
    assert actual.has_z == expected.has_z


def test_vertical_flight_is_valid_3d_geometry():
    frame = CoordinateFrame(spec())
    local = frame.to_local("LINESTRING Z (104.78 29.34 320, 104.78 29.34 380)", "MSL")
    assert tuple(local.coords) == ((0, 0, 20), (0, 0, 80))


def test_declared_agl_and_geoid_conversions_are_invertible():
    frame = CoordinateFrame(spec(terrain_msl_m=310, geoid_separation_m=28))
    assert frame.to_local("POINT Z (104.78 29.34 20)", "AGL").z == 30
    assert frame.to_local("POINT Z (104.78 29.34 358)", "ELLIPSOID").z == 30
    assert from_wkt(frame.to_wgs84(Point(0, 0, 30), "AGL")).z == 20
    assert from_wkt(frame.to_wgs84(Point(0, 0, 30), "ELLIPSOID")).z == 358


def test_agl_uses_spatial_terrain_evidence_in_both_directions():
    frame = CoordinateFrame(spec(), terrain_provider=lambda lon, lat, at: 310 if lon < 104.781 else None)
    local = frame.to_local("POINT Z (104.78 29.34 20)", "AGL")
    assert local.z == 30
    assert from_wkt(frame.to_wgs84(local, "AGL")).z == 20
    with pytest.raises(CoordinateError, match="AGL requires"):
        frame.to_local("POINT Z (104.782 29.34 20)", "AGL")


@pytest.mark.parametrize("reference", ["AGL", "ELLIPSOID", "QNH", "QNE"])
def test_missing_height_evidence_is_an_error(reference):
    with pytest.raises(CoordinateError) as caught:
        CoordinateFrame(spec()).to_local("POINT Z (104.78 29.34 20)", reference, AT)
    assert caught.value.code == "HEIGHT_REFERENCE_UNAVAILABLE"


def correction(**updates):
    fields = dict(reference="QNH", offset_m=12, source="station-report", version="2026-10-16T04",
                  valid_from="2026-10-16T03:00:00Z", valid_to="2026-10-16T05:00:00Z")
    fields.update(updates)
    return HeightCorrection(**fields)


def test_pressure_correction_requires_fresh_time_and_is_invertible():
    frame = CoordinateFrame(spec(height_corrections=[correction()]))
    local = frame.to_local("POINT Z (104.78 29.34 320)", "QNH", AT)
    assert local.z == 32
    assert from_wkt(frame.to_wgs84(local, "QNH", AT)).z == 320
    for at in [None, datetime(2026, 10, 16, 6, tzinfo=timezone.utc), datetime(2026, 10, 16, 4)]:
        with pytest.raises(CoordinateError, match="requires|No valid"):
            frame.to_local("POINT Z (104.78 29.34 320)", "QNH", at)


def test_pressure_correction_requires_spatial_coverage_and_no_ambiguity():
    far = correction(geometry_wkt="POLYGON ((100 20, 101 20, 101 21, 100 20))")
    with pytest.raises(CoordinateError, match="No valid"):
        CoordinateFrame(spec(height_corrections=[far])).to_local("POINT Z (104.78 29.34 320)", "QNH", AT)
    with pytest.raises(CoordinateError, match="Multiple"):
        CoordinateFrame(spec(height_corrections=[correction(), correction(version="other")])).to_local("POINT Z (104.78 29.34 320)", "QNH", AT)


def test_projection_radius_limit_both_directions():
    frame = CoordinateFrame(spec())
    lon, lat, _ = Geod(ellps="WGS84").fwd(104.78, 29.34, 90, 50001)
    with pytest.raises(CoordinateError) as caught:
        frame.to_local(f"POINT Z ({lon} {lat} 320)", "MSL")
    assert caught.value.code == "COORDINATE_RANGE_EXCEEDED"
    with pytest.raises(CoordinateError):
        frame.to_wgs84(Point(50001, 0, 20))
    lon, lat, _ = Geod(ellps="WGS84").fwd(104.78, 29.34, 90, 49999)
    assert frame.to_local(f"POINT Z ({lon} {lat} 320)", "MSL").x == pytest.approx(49999)


def test_bidirectional_batch_conversion_preserves_z_and_context():
    request = CoordinateRequest(coordinates=spec(), direction="wgs84_to_local", alt_reference="MSL",
                                geometries_wkt=["POINT Z (104.78 29.34 320)", "POINT Z (104.781 29.34 330)"])
    result = convert_coordinates(request)
    assert result.coordinate_context["axis_order"] == "longitude,latitude,altitude"
    reverse = request.model_copy(update={"direction": "local_to_wgs84", "geometries_wkt": result.geometries_wkt})
    assert convert_coordinates(reverse).geometries_wkt == request.geometries_wkt


@pytest.mark.parametrize("wkt", ["POINT EMPTY", "POINT Z (104 29 NaN)", "POINT (181 29)", "nonsense", "POINT M (104 29 7)", "LINESTRING Z (104 29 7, 104 29 7)"])
def test_rejects_unusable_coordinate_wkt(wkt):
    with pytest.raises(CoordinateError):
        CoordinateFrame(spec()).to_local(wkt, "MSL")


def test_polygon_footprints_remain_2d_without_vertical_evidence():
    value = "POLYGON ((104.78 29.34, 104.781 29.34, 104.781 29.341, 104.78 29.34))"
    assert not CoordinateFrame(spec()).to_local(value, "AGL").has_z


@pytest.mark.parametrize("updates", [{"origin_lon": "104"}, {"origin_lat": True}, {"origin_alt_msl": float("inf")}, {"max_radius_m": 50001}, {"unknown": 1}])
def test_coordinate_model_is_strict_and_finite(updates):
    values = dict(origin_lon=104.78, origin_lat=29.34)
    values.update(updates)
    with pytest.raises(ValidationError):
        CoordinateSpec(**values)


def task(**updates):
    values = dict(id="t1", start_wkt="POINT Z (104.78 29.34 320)", goal_wkt="POINT Z (104.781 29.34 320)",
                  departure_time="2026-10-16T04:00:00Z", alt_reference="MSL")
    values.update(updates)
    return Task(**values)


@pytest.mark.parametrize("updates", [{"alt_reference": "WGS84"}, {"departure_time": "2026-10-16T04:00:00"}, {"departure_time": 1729000000}, {"route_mode": "preferred"}, {"start_wkt": "POINT (104 29)"}, {"start_wkt": "POINT Z (500 29 30)"}, {"payload_kg": "1"}, {"wat": True}])
def test_task_model_rejects_ambiguous_or_untyped_inputs(updates):
    with pytest.raises(ValidationError):
        task(**updates)


def test_task_altitude_reference_is_required():
    fields = task().model_dump()
    del fields["alt_reference"]
    with pytest.raises(ValidationError):
        Task(**fields)


def airway(**updates):
    grid = dict(minLon=104.78, maxLon=104.781, minLat=29.34, maxLat=29.341, minAlt=310, maxAlt=350)
    fields = dict(id="a", centerLine="LINESTRING Z (104.78 29.34 320, 104.781 29.34 320)", widthMeters=40,
                  minAltitude=310, maxAltitude=350, altReference="MSL", startGrid=grid, endGrid=grid)
    fields.update(updates)
    return Airway(**fields)


def test_external_airway_full_width_and_required_constraints():
    assert airway().width_m == 40
    with pytest.raises(ValidationError):
        airway(climbConstraint={"maxClimbRate": 3})
    with pytest.raises(ValidationError):
        airway(turnConstraint={"minTurnRadius": 0})
    fields = airway().model_dump()
    del fields["startGrid"]
    with pytest.raises(ValidationError):
        Airway(**fields)


def test_scene_ids_bounds_and_dynamic_trajectory_validation():
    bounds = Bounds3D(x_min=-100, x_max=100, y_min=-100, y_max=100, z_min=0, z_max=200)
    with pytest.raises(ValidationError):
        Scene(coordinates=spec(), bounds=bounds, time_origin=AT, airways=[airway(), airway()])
    with pytest.raises(ValidationError):
        TimedTrajectory(id="moving", trajectory_wkt="LINESTRING Z (104.78 29.34 320, 104.781 29.34 320)",
                        vertex_times_s=[0], alt_reference="MSL")
    with pytest.raises(ValidationError):
        TimedTrajectory(id="moving", trajectory_wkt="LINESTRING Z (104.78 29.34 320, 104.781 29.34 320)",
                        vertex_times_s=[1, 1], alt_reference="MSL")


def test_profile_defaults_are_explicit_and_independent():
    profile = default_profile()
    assert (profile.max_speed_mps, profile.cruise_speed_mps, profile.endurance_s) == (12, 6, 2040)
    assert (profile.max_range_m, profile.payload_kg, profile.min_turn_radius_m) == (18000, 0, 0)
    assert "safety_margin_m" in profile.engineering_values
    profile.engineering_values.clear()
    assert default_profile().engineering_values
    with pytest.raises(ValidationError):
        AircraftProfile(cruise_speed_mps=13)
