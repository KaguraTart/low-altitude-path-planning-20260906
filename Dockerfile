ARG BASE_IMAGE=public.ecr.aws/docker/library/python:3.11-slim@sha256:e88e9763f943ec1834f992a4b51e0f24500486803e8bc534e5767af9ea65f6ce
FROM ${BASE_IMAGE} AS builder
WORKDIR /build
ENV PIP_DISABLE_PIP_VERSION_CHECK=1
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    sed -i 's|http://deb.debian.org|https://deb.debian.org|g' /etc/apt/sources.list.d/debian.sources \
    && apt-get -o Acquire::Retries=3 update && apt-get -o Acquire::Retries=3 install -y --no-install-recommends g++ cmake ninja-build \
    && rm -rf /var/lib/apt/lists/*
ARG PIP_INDEX_URL=https://pypi.org/simple
COPY requirements-runtime.lock ./
RUN python -m pip wheel --no-cache-dir --wheel-dir /wheels -r requirements-runtime.lock
COPY pyproject.toml CMakeLists.txt README.md ./
COPY native/ ./native/
COPY uav_planner/ ./uav_planner/
COPY server/ ./server/
COPY serve.py ./
RUN python -m pip wheel --no-cache-dir --no-deps --wheel-dir /wheels ".[server]"

FROM ${BASE_IMAGE} AS runtime
WORKDIR /app
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
ARG GIT_SHA=unknown
LABEL org.opencontainers.image.revision=$GIT_SHA
ENV UAV_GIT_SHA=$GIT_SHA
COPY --from=builder /wheels /wheels
RUN python -m pip install --no-cache-dir --no-index --find-links=/wheels uav-path-planning[server] \
    && rm -rf /wheels
COPY docs/API.md docs/openapi.json ./docs/
EXPOSE 8000
CMD ["python", "-m", "serve", "--host", "0.0.0.0", "--port", "8000"]
