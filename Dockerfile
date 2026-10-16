ARG BASE_IMAGE=python:3.11-slim
FROM ${BASE_IMAGE} AS builder
WORKDIR /build
ENV PIP_DISABLE_PIP_VERSION_CHECK=1
ARG PIP_INDEX_URL=https://pypi.org/simple
RUN apt-get update && apt-get install -y --no-install-recommends g++ cmake ninja-build \
    && rm -rf /var/lib/apt/lists/*
COPY pyproject.toml CMakeLists.txt README.md requirements-runtime.lock ./
RUN python -m pip wheel --no-cache-dir --wheel-dir /wheels -r requirements-runtime.lock
COPY native/ ./native/
COPY uav_planner/ ./uav_planner/
COPY server/ ./server/
COPY src/ ./src/
COPY serve.py ./
RUN python -m pip wheel --no-cache-dir --no-deps --wheel-dir /wheels ".[server]"

FROM ${BASE_IMAGE} AS runtime
WORKDIR /app
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 MPLCONFIGDIR=/tmp/matplotlib
ARG GIT_SHA=unknown
LABEL org.opencontainers.image.revision=$GIT_SHA
ENV UAV_GIT_SHA=$GIT_SHA
COPY --from=builder /wheels /wheels
RUN python -m pip install --no-cache-dir --no-index --find-links=/wheels uav-path-planning[server] \
    && rm -rf /wheels
COPY input/ ./input/
EXPOSE 8000
CMD ["python", "-m", "serve", "--host", "0.0.0.0", "--port", "8000"]
