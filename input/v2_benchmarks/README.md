# Frozen v2 acceptance workloads

`manifest.json` contains the SHA-256 and byte length of each canonical input.
The seed is **20261016**. The manifest's fixed `frozen_at` value is the synthetic
scenario epoch, not a claim about filesystem creation time. The first fixture
files were written at **2026-10-16 09:58:23 UTC**, before this generator's first
planner execution. `--freeze` regenerates the deterministic bytes and refuses
to overwrite any differing existing fixture or manifest.

The checked-in fixture bytes and manifest hashes are authoritative on every
platform. Loading a workload always checks those exact hashes. Regenerating a
scene with a different Python/PROJ/GEOS build can change the final WKT decimal
through floating-point roundoff, so the portability test compares generated
data semantically: keys, list order, scalar strings, integers and booleans stay
exact; floating-point JSON numbers may differ by at most 1e-9 absolute. WKT
geometry type, Z dimension, component/ring structure and coordinate order stay
exact, with at most 2e-11 absolute difference in each longitude, latitude or Z
coordinate. This is micrometre-scale horizontal tolerance, not a planning
geometry tolerance. Failures identify the affected JSON location concisely.
This regeneration-only comparison does not relax fixture integrity:
`--freeze` still refuses any byte difference in an existing file and never
rewrites the authoritative files to match another platform's rounding.

Each suite contains 1,000 task IDs in a 5 km × 5 km × 300 m scene, with
25/25/10 m resolution, a 3,600 s horizon, four engineering aircraft profiles,
100 shared airways, 100 buildings and 20 control zones. No task has more than
one intermediate waypoint. Strict and free suites use their named route mode;
mixed cycles strict/prefer/free and applies all twenty rules.

There are 100 corridor groups of ten concurrent requests. The ten requests in
each group share an actual 300–380 m route and original departure, so all 1,000
tasks participate in continuous conflicts before scheduling. Different groups
occupy separate 500 m tiles; a group is a capacity bottleneck, not a set of ten
spatially separated paths. Ground waiting does not reserve an aircraft before
its actual launch. The independent constructive schedule uses conservative
160 s launch slots. Its rest-to-rest analytic acceleration and braking flights
finish within the arrival windows; it never calls a planner, production motion
builder or scheduler.

Mixed additionally contains 100 climbing flights, 100 collinear business
waypoint flights, 100 right-angle flights with marked stationary yaw, and ten
triggered low-battery returns whose home differs from their mission goal.
The speed cap reduces every profile's horizontal cruise; the climb cap reduces
the vertical flight speed. Weather, signal and noise data have full coverage
plus 100 explicitly unsafe/noisy regions. Included area bands, reduced airway
width, one-way direction, buffers, payload, endurance, time and waypoint bounds
restrict the admissible candidates. Twenty executable counterfactual witnesses
check each rule's effect, including a measurable quiet/noisy cost difference,
an actual separation conflict and a low-battery RTL trigger. Witnesses are
reported separately from successful route validation.

```bash
python tools/benchmark_v2.py --freeze
python tools/benchmark_v2.py --validate-only
python tools/benchmark_v2.py --suite mixed --repeats 10 --scales 10 100 1000 --output /tmp/v2-benchmark-results
python tools/benchmark_v2.py --base-url http://127.0.0.1:8000 --repeats 10 --scales 10 100 1000 --output /tmp/v2-benchmark-http
```

Scene registration is measured separately, then the scene is preloaded.
Each measured run includes request construction, real SDK/HTTP planning and
complete result JSON serialization. Independent validation occurs after the
stop time and requires every requested route to succeed. Every route receives
geometry/rule revalidation, a native certificate rebuilt from its exported
analytic motion segments, endpoint/window checks and an independent continuous
pair check. The latter resamples the analytic time law with a 0.02 m chord-error
bound and inflates the required separation by 0.04 m. Failed or omitted routes
cannot count as a fast successful run. Reports retain every repetition,
fixture hashes, validation evidence, wait/distance/flight metrics, serialized
size, process high-water RSS, and min/median/p95/max timing.

The runner reports the environment; CPU and memory limits must be imposed by
the caller/container. Host SDK results do not by themselves prove the
8-CPU/8-GB deployment acceptance criterion. HTTP RSS is the local validation
process's high-water mark; measure service/container RSS separately.
