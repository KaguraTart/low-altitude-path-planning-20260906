"""Strict JSON contracts for geographic, metre/second v2 planning inputs."""

from __future__ import annotations

import math
from datetime import datetime
from typing import Annotated, Literal

from pydantic import (
    AliasChoices, AwareDatetime, BaseModel, BeforeValidator, ConfigDict, Field,
    JsonValue, StrictBool, StrictInt, StrictStr, field_validator, model_validator,
)
from shapely import from_wkt, get_coordinates
from shapely.geometry.base import BaseGeometry


def _finite_number(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("must be a JSON number, not a string or boolean")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError("must be finite")
    return float(value)


Number = Annotated[float, BeforeValidator(_finite_number)]
Positive = Annotated[Number, Field(gt=0)]
NonNegative = Annotated[Number, Field(ge=0)]


def _timestamp(value: object) -> object:
    if not isinstance(value, (datetime, str)):
        raise ValueError("timestamp must be an ISO 8601 string or aware datetime")
    return value


Timestamp = Annotated[AwareDatetime, BeforeValidator(_timestamp)]
AltitudeReference = Literal["EG", "MSL", "AGL", "ELLIPSOID", "QNE", "QNH"]
RouteMode = Literal["strict", "prefer", "free"]
RULE_TYPES = (
    "HEIGHT_LAYER", "WIDTH", "DIRECTION", "SPEED", "TIME", "AREA", "CLIMB",
    "TURN", "SEPARATION", "NO_FLY", "BUILDING", "ZONE", "WEATHER", "SIGNAL",
    "PRIORITY", "HOVER", "ENDURANCE", "PAYLOAD", "WAYPOINT", "RTL",
)
RuleType = Literal[
    "HEIGHT_LAYER", "WIDTH", "DIRECTION", "SPEED", "TIME", "AREA", "CLIMB",
    "TURN", "SEPARATION", "NO_FLY", "BUILDING", "ZONE", "WEATHER", "SIGNAL",
    "PRIORITY", "HOVER", "ENDURANCE", "PAYLOAD", "WAYPOINT", "RTL",
]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True, validate_default=True)


def validated_wkt(value: str, *, kind: str | None = None, require_z: bool = False) -> str:
    try:
        geometry = from_wkt(value)
    except MemoryError:
        raise
    except Exception as exc:
        raise ValueError("invalid WKT geometry") from exc
    if geometry.is_empty or geometry.geom_type == "GeometryCollection":
        raise ValueError("WKT must be a nonempty, homogeneous geometry")
    if getattr(geometry, "has_m", False):
        raise ValueError("WKT M coordinates are unsupported; use Z altitude")
    if kind is not None and geometry.geom_type != kind:
        raise ValueError(f"WKT must contain a {kind}")
    if require_z and not geometry.has_z:
        raise ValueError("WKT must include explicit Z altitude")
    coords = get_coordinates(geometry, include_z=geometry.has_z)
    if not all(math.isfinite(float(v)) for row in coords for v in row):
        raise ValueError("WKT coordinates must be finite")
    # GEOS regards vertical and stationary trajectories as zero-length in XY.
    # Their Z coordinates and separate timestamps express flight or waiting.
    def vertical(component: BaseGeometry) -> bool:
        if component.geom_type != "LineString" or not component.has_z:
            return False
        vertices = list(component.coords)
        return len({(x, y) for x, y, _ in vertices}) == 1

    vertical_line = vertical(geometry) or (
        geometry.geom_type == "MultiLineString"
        and all(part.is_valid or vertical(part) for part in geometry.geoms)
    )
    if not geometry.is_valid and not vertical_line:
        raise ValueError("WKT geometry is topologically invalid")
    return value


WKT = Annotated[StrictStr, BeforeValidator(validated_wkt)]


def validated_geographic_wkt(value: str) -> str:
    validated_wkt(value)
    for lon, lat in get_coordinates(from_wkt(value)):
        if not -180 <= lon <= 180 or not -90 <= lat <= 90:
            raise ValueError("geographic WKT must use WGS84 longitude/latitude")
    return value


GeographicWKT = Annotated[StrictStr, BeforeValidator(validated_geographic_wkt)]


class Bounds3D(StrictModel):
    x_min: Number
    x_max: Number
    y_min: Number
    y_max: Number
    z_min: Number
    z_max: Number

    @model_validator(mode="after")
    def ordered(self) -> Bounds3D:
        for axis in "xyz":
            if getattr(self, f"{axis}_max") <= getattr(self, f"{axis}_min"):
                raise ValueError(f"{axis}_max must exceed {axis}_min")
        return self


class GeographicBounds(StrictModel):
    minLon: Annotated[Number, Field(ge=-180, le=180)] = Field(validation_alias=AliasChoices("minLon", "x_min", "minLongitude"))
    maxLon: Annotated[Number, Field(ge=-180, le=180)] = Field(validation_alias=AliasChoices("maxLon", "x_max", "maxLongitude"))
    minLat: Annotated[Number, Field(ge=-90, le=90)] = Field(validation_alias=AliasChoices("minLat", "y_min", "minLatitude"))
    maxLat: Annotated[Number, Field(ge=-90, le=90)] = Field(validation_alias=AliasChoices("maxLat", "y_max", "maxLatitude"))
    minAlt: Number = Field(validation_alias=AliasChoices("minAlt", "z_min", "minAltitude"))
    maxAlt: Number = Field(validation_alias=AliasChoices("maxAlt", "z_max", "maxAltitude"))

    @model_validator(mode="after")
    def ordered(self) -> GeographicBounds:
        if self.maxLon < self.minLon or self.maxLat < self.minLat or self.maxAlt < self.minAlt:
            raise ValueError("grid maxima must be greater than or equal to minima")
        return self


class HeightCorrection(StrictModel):
    """Pressure correction: EG ellipsoid altitude = reported altitude + offset_m."""

    target_reference: Literal["EG"] = "EG"
    reference: Literal["QNE", "QNH"]
    offset_m: Number
    source: StrictStr = Field(min_length=1)
    version: StrictStr = Field(min_length=1)
    valid_from: Timestamp
    valid_to: Timestamp
    geometry_wkt: GeographicWKT | None = None

    @model_validator(mode="after")
    def ordered(self) -> HeightCorrection:
        if self.valid_to <= self.valid_from:
            raise ValueError("height correction valid_to must exceed valid_from")
        return self


class CoordinateSpec(StrictModel):
    origin_lon: Annotated[Number, Field(ge=-180, le=180)]
    origin_lat: Annotated[Number, Field(ge=-90, le=90)]
    origin_alt_ellipsoid: Number = Field(default=0, description="WGS84 ellipsoid height (EG) of the local origin, metres. local z = EG - origin_alt_ellipsoid.")
    max_radius_m: Positive = 50000
    terrain_ellipsoid_m: Number | None = None
    geoid_separation_m: Number | None = None
    terrain_source: StrictStr | None = None
    geoid_source: StrictStr | None = None
    height_corrections: list[HeightCorrection] = Field(default_factory=list)


class AircraftProfile(StrictModel):
    id: StrictStr = "dji_mini_4_pro"
    name: StrictStr = "DJI Mini 4 Pro (engineering planning profile)"
    max_speed_mps: Positive = 12
    cruise_speed_mps: Positive = 6
    climb_speed_mps: Positive = 3
    descent_speed_mps: Positive = 3
    max_yaw_rate_deg_s: Positive = 60
    acceleration_mps2: Positive = 2
    deceleration_mps2: Positive = 2
    min_turn_radius_m: NonNegative = 0
    radius_m: Positive = 0.3
    height_m: Positive = 0.2
    safety_margin_m: NonNegative = 5
    endurance_s: Positive = 2040
    max_range_m: Positive = 18000
    payload_kg: NonNegative = 0
    rtl_altitude_m: NonNegative = 20
    can_hover: StrictBool = True
    supported_bands: list[StrictStr] = Field(default_factory=lambda: ["2.4GHz", "5.8GHz"])
    low_battery_percent: Annotated[Number, Field(ge=0, le=100)] = 20
    engineering_values: list[StrictStr] = Field(default_factory=lambda: [
        "max_speed_mps", "cruise_speed_mps", "climb_speed_mps", "descent_speed_mps",
        "max_yaw_rate_deg_s", "acceleration_mps2", "deceleration_mps2",
        "min_turn_radius_m", "radius_m", "height_m", "safety_margin_m",
        "rtl_altitude_m",
        "low_battery_percent",
    ])

    @model_validator(mode="after")
    def consistent(self) -> AircraftProfile:
        if self.cruise_speed_mps > self.max_speed_mps:
            raise ValueError("cruise_speed_mps must not exceed max_speed_mps")
        return self


class Volume(StrictModel):
    id: StrictStr = Field(min_length=1)
    geometry_wkt: GeographicWKT
    min_altitude: Number
    max_altitude: Number
    alt_reference: AltitudeReference
    buffer_m: NonNegative = 0
    valid_from: Timestamp | None = None
    valid_to: Timestamp | None = None

    @model_validator(mode="after")
    def consistent(self) -> Volume:
        if self.max_altitude < self.min_altitude:
            raise ValueError("max_altitude must be >= min_altitude")
        _ordered_times(self.valid_from, self.valid_to)
        return self


class Connector(Volume):
    airway_ids: list[StrictStr] = Field(default_factory=list)


class ClimbConstraint(StrictModel):
    maxClimbRate: Positive
    maxDescentRate: Positive


class TurnConstraint(StrictModel):
    maxTurnAngle: Annotated[Positive, Field(le=180)]
    minTurnRadius: NonNegative


class Airway(StrictModel):
    allowed_profile_ids: list[StrictStr] = Field(default_factory=list)
    id: StrictStr = Field(min_length=1)
    centerLine: GeographicWKT
    widthMeters: Positive
    minAltitude: Number
    maxAltitude: Number
    altReference: AltitudeReference
    startGrid: GeographicBounds
    endGrid: GeographicBounds
    status: Literal["ACTIVE", "INACTIVE"] = "ACTIVE"
    direction: Literal["BOTH", "FORWARD", "REVERSE"] = "BOTH"
    valid_from: Timestamp | None = None
    valid_to: Timestamp | None = None
    climbConstraint: ClimbConstraint | None = None
    turnConstraint: TurnConstraint | None = None

    @field_validator("centerLine")
    @classmethod
    def line_z(cls, value: str) -> str:
        return validated_wkt(value, kind="LineString", require_z=True)

    @model_validator(mode="after")
    def consistent(self) -> Airway:
        if self.maxAltitude <= self.minAltitude:
            raise ValueError("maxAltitude must exceed minAltitude")
        vertices=list(from_wkt(self.centerLine).coords)
        for point,grid,label in ((vertices[0],self.startGrid,"startGrid"),(vertices[-1],self.endGrid,"endGrid")):
            if not (grid.minLon <= point[0] <= grid.maxLon and grid.minLat <= point[1] <= grid.maxLat and grid.minAlt <= point[2] <= grid.maxAlt):
                raise ValueError(f"centerLine endpoint must lie inside {label}, including altitude")
        if any(p[2] < self.minAltitude or p[2] > self.maxAltitude for p in vertices):
            raise ValueError("centerLine reference altitude must lie inside its hard altitude band")
        _ordered_times(self.valid_from, self.valid_to)
        return self

    @property
    def center_line_wkt(self) -> str:
        return self.centerLine

    @property
    def geometry_wkt(self) -> str:
        return self.centerLine

    @property
    def width_m(self) -> float:
        return self.widthMeters

    @property
    def min_altitude(self) -> float:
        return self.minAltitude

    @property
    def max_altitude(self) -> float:
        return self.maxAltitude

    @property
    def alt_reference(self) -> str:
        return self.altReference


class ControlZone(StrictModel):
    id: StrictStr = Field(min_length=1)
    geom: GeographicWKT
    boundary2d: GeographicWKT | None = None
    centerPoint: GeographicWKT | None = None
    minAlt: Number
    maxAlt: Number
    zoneType: StrictStr
    shapeParams: dict[StrictStr, JsonValue] = Field(default_factory=dict)
    altReference: AltitudeReference
    areaM2: NonNegative | None = Field(default=None, validation_alias=AliasChoices("areaM2", "area"))
    volumeM3: NonNegative | None = Field(default=None, validation_alias=AliasChoices("volumeM3", "volume"))
    level: StrictStr | StrictInt | None = None
    status: Literal["ACTIVE", "INACTIVE"] = "ACTIVE"
    effective_start: Timestamp | None = None
    effective_end: Timestamp | None = None

    @model_validator(mode="after")
    def consistent(self) -> ControlZone:
        if self.maxAlt < self.minAlt:
            raise ValueError("maxAlt must be >= minAlt")
        _ordered_times(self.effective_start, self.effective_end)
        return self

    @property
    def geometry_wkt(self) -> str:
        return self.geom

    @property
    def min_altitude(self) -> float:
        return self.minAlt

    @property
    def max_altitude(self) -> float:
        return self.maxAlt

    @property
    def alt_reference(self) -> str:
        return self.altReference

    @property
    def valid_from(self) -> datetime | None:
        return self.effective_start

    @property
    def valid_to(self) -> datetime | None:
        return self.effective_end


class ScopeGrid(StrictModel):
    x: StrictInt | None = None
    y: StrictInt | None = None
    z: StrictInt | None = None
    geometry: GeographicWKT | None = None

    @model_validator(mode="after")
    def located(self) -> ScopeGrid:
        indices = (self.x, self.y, self.z)
        if self.geometry is not None:
            if any(i is not None for i in indices):
                raise ValueError("scope grid uses geometry or x/y/z, not both")
            geom = from_wkt(self.geometry)
            if geom.geom_type not in {"Point", "Polygon"} or not geom.has_z:
                raise ValueError("scope grid geometry requires POINT Z or POLYGON Z")
        elif any(i is None or i < 0 for i in indices):
            raise ValueError("scope grid requires nonnegative x, y and z indices")
        return self


class ScopeGeometry(StrictModel):
    type: Literal["CONTROL_ZONE", "NO_FLY_ZONE", "ADMIN_DIVISION", "AIRSPACE", "LANDING_SITE"]
    geom: GeographicWKT | None = None
    center_point: GeographicWKT | None = None
    boundary_2d: GeographicWKT | None = None

    @model_validator(mode="after")
    def consistent(self) -> ScopeGeometry:
        if not any((self.geom, self.center_point, self.boundary_2d)):
            raise ValueError("scope feature requires a geometry")
        if self.type != "LANDING_SITE" and not (self.geom or self.boundary_2d):
            raise ValueError("region scope requires geom or boundary_2d; a center is not a region")
        if self.geom:
            if self.type == "ADMIN_DIVISION":
                raise ValueError("ADMIN_DIVISION uses boundary_2d and center_point")
            validated_wkt(self.geom, kind="Point" if self.type == "LANDING_SITE" else "Polygon", require_z=True)
        if self.center_point:
            if self.type in {"NO_FLY_ZONE", "AIRSPACE"}:
                raise ValueError("this scope feature does not use center_point")
            validated_wkt(self.center_point, kind="Point", require_z=self.type != "ADMIN_DIVISION")
        if self.boundary_2d:
            geom = from_wkt(self.boundary_2d)
            if geom.geom_type not in {"Polygon", "MultiPolygon"} or geom.has_z:
                raise ValueError("boundary_2d requires a two-dimensional POLYGON or MULTIPOLYGON")
        return self


class RuleScope(StrictModel):
    grid: list[ScopeGrid] = Field(default_factory=list)
    geom: list[ScopeGeometry] = Field(default_factory=list)

    @model_validator(mode="after")
    def nonempty(self) -> RuleScope:
        if not self.grid and not self.geom:
            raise ValueError("scope requires grid or geom entries")
        return self


class Rule(StrictModel):
    id: StrictStr = Field(min_length=1)
    rule_type: RuleType = Field(validation_alias=AliasChoices("rule_type", "type"))
    content: dict[StrictStr, JsonValue] = Field(validation_alias=AliasChoices("content", "constraints"), description="Rule parameters. HEIGHT_LAYER defaults to EG metres; WIDTH.width is full width in metres; DIRECTION.directions uses north=0 degrees clockwise; TIME.weekdays uses Sunday=0, Monday=1 through Saturday=6.")
    scope: RuleScope | None = Field(default=None, description="Rule application geometry in WGS84/EG. Missing scope applies to the scene. Grid indices require scene.grid geometry metadata.")
    resource_ids: list[StrictStr] = Field(default_factory=list)
    effective_start: Timestamp | None = None
    effective_end: Timestamp | None = None

    @field_validator("effective_start", "effective_end", mode="before")
    @classmethod
    def platform_timestamp(cls, value: object) -> object:
        if value == "":
            return None
        if isinstance(value, str):
            try:
                parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
            except ValueError:
                return value
            if parsed.tzinfo is None:
                from zoneinfo import ZoneInfo
                return parsed.replace(tzinfo=ZoneInfo("Asia/Shanghai"))
        return value

    @model_validator(mode="after")
    def consistent(self) -> Rule:
        _ordered_times(self.effective_start, self.effective_end)
        if self.scope:
            grid_types = {"BUILDING", "WEATHER", "SIGNAL", "WIDTH", "PRIORITY"}
            geom_types = {"NO_FLY", "ZONE", "AREA", "PRIORITY"}
            if self.scope.grid and self.rule_type not in grid_types:
                raise ValueError("scope.grid is not applicable to this rule type")
            if self.scope.geom and self.rule_type not in geom_types:
                raise ValueError("scope.geom is not applicable to this rule type")
        return self


class FieldCell(StrictModel):
    geometry_wkt: GeographicWKT | None = None
    bounds: Bounds3D | None = None
    min_altitude: Number | None = None
    max_altitude: Number | None = None
    values: dict[StrictStr, JsonValue]
    valid_from: AwareDatetime | None = None
    valid_to: AwareDatetime | None = None

    @model_validator(mode="after")
    def located(self) -> FieldCell:
        if (self.geometry_wkt is None) == (self.bounds is None):
            raise ValueError("field cell needs exactly one of geometry_wkt or bounds")
        if self.min_altitude is not None and self.max_altitude is not None and self.max_altitude < self.min_altitude:
            raise ValueError("max_altitude must be >= min_altitude")
        _ordered_times(self.valid_from, self.valid_to)
        return self


class Dataset(StrictModel):
    id: StrictStr = Field(min_length=1)
    kind: Literal["terrain", "weather", "signal", "noise", "area", "grid", "risk", "corridor"]
    version: StrictStr = Field(min_length=1)
    valid_from: Timestamp | None = None
    valid_to: Timestamp | None = None
    cells: list[FieldCell] = Field(default_factory=list)
    geometry_wkt: GeographicWKT | None = None

    @model_validator(mode="after")
    def consistent(self) -> Dataset:
        _ordered_times(self.valid_from, self.valid_to)
        return self


GridIndex = tuple[StrictInt, StrictInt, StrictInt]


class GridRange(StrictModel):
    start: GridIndex
    end: GridIndex

    @model_validator(mode="after")
    def ordered(self) -> GridRange:
        if any(a < 0 or b < a for a, b in zip(self.start, self.end)):
            raise ValueError("grid range must have nonnegative ordered indices")
        return self


class GridCell(StrictModel):
    bounds: Bounds3D
    code: StrictStr | None = None
    occupied: StrictBool = True


class GridSpec(StrictModel):
    level: Annotated[StrictInt, Field(ge=0)] | None = None
    crs: StrictStr = "LOCAL_AEQD"
    origin: tuple[Number, Number, Number]
    resolution: tuple[Positive, Positive, Positive]
    shape: tuple[Annotated[StrictInt, Field(gt=0)], Annotated[StrictInt, Field(gt=0)], Annotated[StrictInt, Field(gt=0)]]
    occupied: list[GridIndex] = Field(default_factory=list)
    occupied_ranges: list[GridRange] = Field(default_factory=list)
    cells: list[GridCell] = Field(default_factory=list)
    codes: dict[StrictStr, StrictStr] = Field(default_factory=dict)

    @model_validator(mode="after")
    def in_bounds(self) -> GridSpec:
        for index in self.occupied:
            if any(i < 0 or i >= n for i, n in zip(index, self.shape)):
                raise ValueError("occupied grid index is out of bounds")
        for item in self.occupied_ranges:
            if any(i >= n for i, n in zip(item.end, self.shape)):
                raise ValueError("occupied grid range is out of bounds")
        return self


class TimedTrajectory(StrictModel):
    id: StrictStr = Field(min_length=1)
    trajectory_wkt: GeographicWKT
    vertex_times_s: list[NonNegative] = Field(min_length=1)
    alt_reference: AltitudeReference
    horizontal_separation_m: NonNegative = 5
    vertical_separation_m: NonNegative = 5

    @model_validator(mode="after")
    def consistent(self) -> TimedTrajectory:
        geometry = from_wkt(self.trajectory_wkt)
        if geometry.geom_type not in {"Point", "LineString"} or not geometry.has_z:
            raise ValueError("timed trajectory must be POINT Z or LINESTRING Z")
        if len(get_coordinates(geometry)) != len(self.vertex_times_s):
            raise ValueError("timed trajectory vertices must match vertex_times_s")
        if any(b <= a for a, b in zip(self.vertex_times_s, self.vertex_times_s[1:])):
            raise ValueError("timed trajectory times must strictly increase")
        return self


class Scene(StrictModel):
    scene_type: Literal["LOW_ALTITUDE"] = "LOW_ALTITUDE"
    network_version: StrictStr | None = Field(default=None, description="Upstream airway network version; separate from the compiled scene content version.")
    coordinates: CoordinateSpec
    bounds: Bounds3D
    resolution: tuple[Positive, Positive, Positive] = (25, 25, 10)
    horizon_s: Positive = 3600
    time_origin: Timestamp
    airways: list[Airway] = Field(default_factory=list)
    connectors: list[Connector] = Field(default_factory=list)
    obstacles: list[Volume] = Field(default_factory=list)
    dynamic_obstacles: list[TimedTrajectory] = Field(default_factory=list)
    control_zones: list[ControlZone] = Field(default_factory=list)
    rules: list[Rule] = Field(default_factory=list)
    datasets: list[Dataset] = Field(default_factory=list)
    profiles: list[AircraftProfile] = Field(default_factory=list)
    grid: GridSpec | None = None

    @model_validator(mode="after")
    def unique_ids(self) -> Scene:
        for name in ("airways", "connectors", "obstacles", "dynamic_obstacles", "control_zones", "rules", "datasets", "profiles"):
            ids = [item.id for item in getattr(self, name)]
            if len(ids) != len(set(ids)):
                raise ValueError(f"{name} ids must be unique")
        return self

    @property
    def zones(self) -> list[ControlZone]:
        return self.control_zones

    @property
    def buildings(self) -> list[Volume]:
        return self.obstacles

    @property
    def horizon_seconds(self) -> float:
        return self.horizon_s


class Waypoint(StrictModel):
    wkt: GeographicWKT
    earliest: Timestamp | None = None
    latest: Timestamp | None = None
    hover_s: NonNegative = 0
    hover: StrictBool = False

    @field_validator("wkt")
    @classmethod
    def point_z(cls, value: str) -> str:
        return validated_wkt(value, kind="Point", require_z=True)

    @model_validator(mode="after")
    def consistent(self) -> Waypoint:
        _ordered_times(self.earliest, self.latest, equal=True)
        return self


class AircraftState(StrictModel):
    at: Timestamp | None = None
    battery_percent: Annotated[Number, Field(ge=0, le=100)] | None = None
    remaining_flight_time_s: NonNegative | None = None
    remaining_range_m: NonNegative | None = None
    home_wkt: GeographicWKT | None = None
    alt_reference: AltitudeReference | None = None

    @field_validator("home_wkt")
    @classmethod
    def point_z(cls, value: str | None) -> str | None:
        return validated_wkt(value, kind="Point", require_z=True) if value is not None else None


class SearchOptions(StrictModel):
    algorithm: Literal["ASTAR", "DIJKSTRA"] = "ASTAR"
    neighbors: Literal[6, 18, 26] = 26
    grid_level: Annotated[StrictInt, Field(ge=0)] | None = None
    max_expanded_nodes: Annotated[StrictInt, Field(gt=0)] | None = None

    @field_validator("neighbors", mode="before")
    @classmethod
    def integer_neighbors(cls, value):
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("neighbors must be an integer")
        return value


class Task(StrictModel):
    search: SearchOptions | None = None
    engine_id: Literal["ENG-06", "ENG-07"] | None = None
    id: StrictStr = Field(min_length=1)
    start_wkt: GeographicWKT
    goal_wkt: GeographicWKT
    departure_time: Timestamp
    alt_reference: AltitudeReference
    route_mode: RouteMode = "strict"
    waypoints: list[Waypoint] = Field(default_factory=list)
    earliest_arrival: Timestamp | None = None
    latest_arrival: Timestamp | None = None
    profile_id: StrictStr = "dji_mini_4_pro"
    aircraft: AircraftProfile | None = None
    state: AircraftState | None = None
    payload_kg: NonNegative = 0
    priority: StrictInt = 0
    start_connector_id: StrictStr | None = None
    end_connector_id: StrictStr | None = None
    rule_ids: list[StrictStr] | None = None

    @field_validator("start_wkt", "goal_wkt")
    @classmethod
    def point_z(cls, value: str) -> str:
        return validated_wkt(value, kind="Point", require_z=True)

    @model_validator(mode="after")
    def consistent(self) -> Task:
        expected = "ENG-07" if self.route_mode == "free" else "ENG-06"
        if self.engine_id is None:
            self.engine_id = expected
        elif self.engine_id != expected:
            raise ValueError("ENG-06 uses strict/prefer; ENG-07 uses free")
        _ordered_times(self.earliest_arrival, self.latest_arrival, equal=True)
        if self.latest_arrival is not None and self.latest_arrival < self.departure_time:
            raise ValueError("latest_arrival precedes departure_time")
        return self


class CoordinateRequest(StrictModel):
    geometries_wkt: list[WKT] = Field(min_length=1)
    direction: Literal["wgs84_to_local", "local_to_wgs84"]
    alt_reference: AltitudeReference
    coordinates: CoordinateSpec
    at: Timestamp | None = None


class CoordinateResponse(StrictModel):
    geometries_wkt: list[StrictStr]
    direction: Literal["wgs84_to_local", "local_to_wgs84"]
    alt_reference: AltitudeReference
    coordinate_context: dict[StrictStr, JsonValue]


class MotionSegment(StrictModel):
    airway_ids: list[StrictStr] = Field(default_factory=list)
    kind: Literal["line", "arc", "climb", "descent", "hover", "yaw"]
    start_time_s: NonNegative
    end_time_s: NonNegative
    start: tuple[Number, Number, Number]
    end: tuple[Number, Number, Number]
    length_m: NonNegative = 0
    distance_m: NonNegative = 0
    points: list[tuple[Number, Number, Number]] = Field(default_factory=list)
    times_s: list[NonNegative] = Field(default_factory=list)
    max_speed_mps: NonNegative | None = None
    radius_m: NonNegative | None = None
    center: tuple[Number, Number, Number] | None = None
    start_speed_mps: NonNegative | None = None
    end_speed_mps: NonNegative | None = None
    acceleration_mps2: Number | None = None
    deceleration_mps2: NonNegative | None = None
    peak_speed_mps: NonNegative | None = None
    accel_time_s: NonNegative | None = None
    cruise_time_s: NonNegative | None = None
    decel_time_s: NonNegative | None = None
    sweep_angle_deg: Number | None = None
    profile: dict[StrictStr, JsonValue] = Field(default_factory=dict)

    @model_validator(mode="after")
    def consistent(self) -> MotionSegment:
        if self.end_time_s < self.start_time_s:
            raise ValueError("motion segment end precedes start")
        return self


class Keypoint(StrictModel):
    types: list[StrictStr]
    wkt: WKT
    time_s: NonNegative
    grid_indices: GridIndex | None = None
    grid_code: StrictStr | None = None
    waypoint_index: Annotated[StrictInt, Field(ge=0)] | None = None


class RouteMetrics(StrictModel):
    distance_m: NonNegative = 0
    flight_time_s: NonNegative = 0
    wait_time_s: NonNegative = 0


class Violation(StrictModel):
    code: StrictStr
    message: StrictStr
    rule_id: StrictStr | None = None
    time_s: Number | None = None
    details: dict[StrictStr, JsonValue] = Field(default_factory=dict)


class RouteValidation(StrictModel):
    valid: StrictBool
    violations: list[Violation] = Field(default_factory=list)


class RuleReport(StrictModel):
    rule_id: StrictStr = Field(validation_alias=AliasChoices("rule_id", "id"))
    rule_type: RuleType
    status: Literal["applied", "not_applicable", "inactive", "failed", "unsupported"]
    message: StrictStr = ""
    details: dict[StrictStr, JsonValue] = Field(default_factory=dict)
    reasons: list[StrictStr] = Field(default_factory=list)
    dataset_versions: dict[StrictStr, StrictStr] = Field(default_factory=dict)


class SearchReport(StrictModel):
    algorithm: Literal["ASTAR", "DIJKSTRA"]
    neighbors: Literal[6, 18, 26]
    grid_level: StrictInt | None = None
    expanded_nodes: Annotated[StrictInt, Field(ge=0)]
    max_expanded_nodes: Annotated[StrictInt, Field(gt=0)] | None = None


class TraversedGrid(StrictModel):
    grid_indices: GridIndex
    grid_code: StrictStr | None = None
    entry_time_s: NonNegative
    exit_time_s: NonNegative


class RouteResult(StrictModel):
    search_report: SearchReport | None = None
    traversed_grids: list[TraversedGrid] = Field(default_factory=list)
    grid_context: dict[StrictStr, JsonValue] = Field(default_factory=dict)
    engine_id: Literal["ENG-06", "ENG-07"]
    task_id: StrictStr
    alt_reference: AltitudeReference
    coordinate_context: dict[StrictStr, JsonValue]
    status: Literal["planned", "failed", "timeout"]
    code: StrictStr | None = None
    message: StrictStr = ""
    route_mode: RouteMode
    trajectory_wkt: WKT | None = None
    start_wkt: WKT
    goal_wkt: WKT
    departure_time: Timestamp
    time_origin: Timestamp | None = None
    action: Literal["mission", "return_to_home"] = "mission"
    vertex_times_s: list[NonNegative] = Field(default_factory=list)
    motion_segments: list[MotionSegment] = Field(default_factory=list)
    keypoints: list[Keypoint] = Field(default_factory=list)
    metrics: RouteMetrics = Field(default_factory=RouteMetrics)
    validation: RouteValidation = Field(default_factory=lambda: RouteValidation(valid=False))
    rule_report: list[RuleReport] = Field(default_factory=list)


class Conflict(StrictModel):
    task_ids: list[StrictStr]
    start_time_s: NonNegative
    end_time_s: NonNegative
    minimum_distance_m: NonNegative | None = None


class BatchResult(StrictModel):
    planned_at: Timestamp
    request_id: StrictStr
    scene_id: StrictStr
    scene_version: StrictStr
    routes: list[RouteResult]
    success_count: Annotated[StrictInt, Field(ge=0)]
    failed_count: Annotated[StrictInt, Field(ge=0)]
    conflicts: list[Conflict] = Field(default_factory=list)
    elapsed_ms: NonNegative
    engine: StrictStr
    coordinate_context: dict[StrictStr, JsonValue] = Field(default_factory=dict)
    stage_timings_ms: dict[StrictStr, NonNegative] = Field(default_factory=dict)


def _ordered_times(start: datetime | None, end: datetime | None, *, equal: bool = False) -> None:
    if start is not None and end is not None and (end < start if equal else end <= start):
        raise ValueError("end time must follow start time")


# Explicit aliases for callers that prefer contract-oriented names.
SceneRequest = Scene
TaskRequest = Task
Zone = ControlZone
CoordinateBatchRequest = CoordinateRequest
CoordinateBatchResponse = CoordinateResponse
