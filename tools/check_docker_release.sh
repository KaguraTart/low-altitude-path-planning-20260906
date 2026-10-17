#!/usr/bin/env bash
# Exercise the installed wheel in an isolated container; no production ports.
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 || ! $2 =~ ^[0-9a-f]{40}$ ]]; then
  echo 'Usage: bash tools/check_docker_release.sh IMAGE FULL_GIT_SHA [OUTPUT_DIR]' >&2
  exit 2
fi
uav_image=$1
uav_revision=$2
uav_output=${3:-output/ci-docker}
uav_container="uav-release-check-${GITHUB_RUN_ID:-local}-$$"
mkdir -p "$uav_output"

cleanup() {
  local status=$?
  trap - EXIT
  if docker container inspect "$uav_container" >/dev/null 2>&1; then
    docker logs "$uav_container" > "$uav_output/server.log" 2>&1 || true
    docker cp "$uav_container:/tmp/uav-release-http" "$uav_output/http-smoke" >/dev/null 2>&1 || true
    if [[ $status -ne 0 ]]; then
      tail -n 80 "$uav_output/server.log" >&2
    fi
    docker rm -f "$uav_container" >/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT

# These limits cover functionality. The dedicated 8 CPU / 8 GiB performance
# acceptance remains separate from timing on shared GitHub runners.
docker run -d --name "$uav_container" --cpus=2 --memory=2g --memory-swap=2g \
  "$uav_image" > "$uav_output/container-id.txt"
docker inspect "$uav_container" \
  --format '{"image_id":{{json .Image}},"git_sha":{{json (index .Config.Labels "org.opencontainers.image.revision")}},"nano_cpus":{{.HostConfig.NanoCpus}},"memory_bytes":{{.HostConfig.Memory}}}' \
  > "$uav_output/image.json"
uav_actual_revision=$(docker inspect "$uav_container" --format '{{index .Config.Labels "org.opencontainers.image.revision"}}')
if [[ $uav_actual_revision != "$uav_revision" ]]; then
  echo "Image revision mismatch: expected $uav_revision, received $uav_actual_revision" >&2
  exit 1
fi

uav_ready=false
for ((attempt=0; attempt<30; attempt++)); do
  if docker exec "$uav_container" python -c \
    'import json, urllib.request; r=json.load(urllib.request.urlopen("http://127.0.0.1:8000/api/v2/health", timeout=2)); assert r["status"] == "ok" and r["native_available"]' \
    >/dev/null 2>&1; then
    uav_ready=true
    break
  fi
  if [[ $(docker inspect "$uav_container" --format '{{.State.Running}}') != true ]]; then
    break
  fi
  sleep 1
done
if [[ $uav_ready != true ]]; then
  echo 'Docker service did not become ready within 30 probes' >&2
  exit 1
fi

docker cp "$uav_container:/app/docs/API.md" "$uav_output/API.md" >/dev/null
docker cp "$uav_container:/app/docs/openapi.json" "$uav_output/openapi.json" >/dev/null
cmp docs/API.md "$uav_output/API.md"
cmp docs/openapi.json "$uav_output/openapi.json"
docker exec "$uav_container" python -c \
  'import importlib.util; assert importlib.util.find_spec("src") is None; assert importlib.util.find_spec("uav_planner.api") is None'
docker exec -i "$uav_container" python - \
  --base-url http://127.0.0.1:8000 --expected-git-sha "$uav_revision" \
  --openapi-snapshot /app/docs/openapi.json --archive-dir /tmp/uav-release-http < tools/smoke_v2.py > "$uav_output/smoke.json"
cat "$uav_output/smoke.json"
