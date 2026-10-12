# 部署指南

> 把路径规划服务跑起来并对外提供访问。
>
> 接口字段与调用示例见 [`../../docs/API.md`](../../docs/API.md)。

---

## 0. 安全前提（先读这一节）

**服务不做任何认证**——能访问到端口的人就能提交规划任务。请按部署范围选对应做法：

| 部署范围 | 做法 | 是否可接受 |
|----------|------|-----------|
| 仅本机 | 绑 `127.0.0.1` | ✅ |
| 可信内网（自己人） | 绑 `0.0.0.0`，确保在受控网段 | ✅ |
| **公网** | **必须在前面加一层边缘鉴权** | ⚠️ 不加不行 |

**边缘鉴权的三种做法**（都不需要改代码）：

| 方式 | 说明 |
|------|------|
| **Cloudflare Access** | 在边缘校验身份，未登录的人到不了源站。最省事 |
| **反向代理 Basic Auth** | Nginx / Caddy 加一行配置 |
| **VPN / Tailscale** | 不暴露公网，只在虚拟内网可达 |

> ⚠️ **资源限制不等于鉴权**。给容器限 CPU 能防止它把宿主机拖垮，但拦不住
> 谁在调用服务——访问者照样能把分配的那点 CPU 跑满。两件事要分开做。

---

## 1. 直接跑（不用 Docker）

```bash
conda activate uav_path_planning
python serve.py --host 127.0.0.1 --port 8000     # 仅本机
python serve.py --host 0.0.0.0   --port 8000     # 同网段可访问
```

启动后会打印本机与局域网地址，把 `http://<IP>:8000/docs` 发给对接方即可。

---

## 2. Docker 部署（推荐）

### 2.1 Dockerfile

放在仓库根目录：

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# 先装依赖，利用镜像层缓存：只改代码时不必重装依赖
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY uav_planner/ ./uav_planner/
COPY server/ ./server/
COPY src/ ./src/
COPY serve.py ./
COPY input/ ./input/

EXPOSE 8000

# 绑 0.0.0.0 是容器内的必要写法；对外暴露范围由 -p 参数控制
CMD ["python", "serve.py", "--host", "0.0.0.0", "--port", "8000"]
```

`.dockerignore`（避免把无关内容打进构建上下文）：

```
output/
tests/
tools/
internal/
docs/
.git/
__pycache__/
*.pyc
```

### 2.2 构建与运行

```bash
docker build -t uav-planner:1.0.0 .

# 只给本机访问
docker run -d --name uav-planner \
  -p 127.0.0.1:8000:8000 \
  --cpus=2 \
  --memory=2g \
  uav-planner:1.0.0

# 给同网段访问（前面务必有边缘鉴权）
docker run -d --name uav-planner \
  -p 8000:8000 \
  --cpus=2 \
  --memory=2g \
  uav-planner:1.0.0
```

**资源限制要加**：规划是 CPU 密集的（万级批量实测 437 秒），不加限制会把宿主机拖满。

| 参数 | 作用 |
|------|------|
| `--cpus=2` | 最多用 2 核。超出不会更强，只会让队列排长 |
| `--memory=2g` | 上限。**要加**——异步作业表在内存里，不限内存有风险 |

### 2.3 关于 `--workers`

`serve.py` 固定单进程（`--workers 1`），这是有意的：

- 规划内部已多线程并行，多进程会互相抢核；
- **异步作业表存在进程内存里**，多进程会导致轮询落到没有该作业的进程。

想提高吞吐，调 `--cpus` 比加 worker 有效。

---

## 3. 经 Cloudflare Tunnel 对外

不需要公网 IP、不开端口、源站不可直连。

```bash
brew install cloudflared

# 服务绑回环
docker run -d --name uav-planner -p 127.0.0.1:8000:8000 \
  --cpus=2 --memory=2g uav-planner:1.0.0

# 建隧道
cloudflared tunnel --url http://127.0.0.1:8000
```

长期使用改命名隧道 + 自有域名，配置 `~/.cloudflared/config.yml`：

```yaml
tunnel: uav-planner
credentials-file: /Users/<你>/.cloudflared/<Tunnel-ID>.json

ingress:
  - hostname: uav.你的域名.com
    service: http://127.0.0.1:8000
  - service: http_status:404
```

> ⚠️ 隧道只解决「怎么连」，**不解决「谁能连」**。公网暴露前请先配 §4 的
> Cloudflare Access 或限流规则。

---

## 4. 边缘鉴权与限流（公网必做）

### 4.1 Cloudflare Access（推荐，零代码）

控制台 → **Zero Trust → Access → Applications** → 添加应用：

1. 选 **Self-hosted**，域名填 `uav.你的域名.com`
2. 策略设为「Allow」+ 指定邮箱 / Google 账号 / 一次性 PIN
3. 保存

效果：未通过身份校验的请求在**边缘**就被挡回，根本到不了你的服务器。

### 4.2 Cloudflare Rate Limiting

控制台 → 你的域名 → **Security → WAF → Rate limiting rules**：

| 规则 | 建议阈值 |
|------|----------|
| 匹配 `/api/v1/*` | 每 IP 每分钟 300 次 |
| 匹配 `/api/v1/plan/batch`、`/api/v1/jobs` | 每 IP 每分钟 10 次 |

超出后边缘直接返回 429，源站不受影响。

### 4.3 Nginx Basic Auth（不想用 Cloudflare 时）

```nginx
server {
    listen 443 ssl;
    server_name uav.你的域名.com;

    location / {
        auth_basic "UAV Planner";
        auth_basic_user_file /etc/nginx/.htpasswd;
        proxy_pass http://127.0.0.1:8000;
        proxy_read_timeout 900s;      # 批量规划耗时长，别用默认的 60s
    }
}
```

```bash
# 生成密码文件
htpasswd -c /etc/nginx/.htpasswd 你的用户名
```

---

## 5. 排错

| 现象 | 原因 | 处理 |
|------|------|------|
| 容器起来但连不上 | 端口没映射或绑了回环 | 确认 `-p 8000:8000`；容器内必须绑 `0.0.0.0` |
| 宿主机卡死 | 没限 CPU | 加 `--cpus=2` |
| 容器被 OOM Kill | 没限内存或限得太小 | 加 `--memory=2g`；批量任务注意规模 |
| 批量请求超时 | 同步接口扛不住长任务 | 改用 `POST /api/v1/jobs`，或调大反代超时 |
| 轮询作业报「不存在」 | 多进程部署 | 保持 `--workers 1` |

**自检**：

```bash
curl -s http://127.0.0.1:8000/api/v1/health
curl -s -X POST http://127.0.0.1:8000/api/v1/plan/single \
  -H 'Content-Type: application/json' \
  -d '{"start":{"x":500,"y":500,"z":100},"goal":{"x":2500,"y":2500,"z":150}}'
```

---

## 6. 部署前清单

- [ ] 明确部署范围（本机 / 内网 / 公网），并据此选了对应做法
- [ ] 公网部署**已配置边缘鉴权**（Cloudflare Access 或反代 Basic Auth）
- [ ] 容器设了 `--cpus` 上限
- [ ] 容器设了 `--memory` 上限
- [ ] `--workers` 保持 1
- [ ] 反代超时已调大（批量规划可能跑几分钟）
- [ ] 确认这台机器的其他服务没有被一并暴露

---

## 附：相关文档

| 文档 | 内容 |
|------|------|
| [`../../docs/API.md`](../../docs/API.md) | 接口说明：参数、返回、错误码、示例、FAQ |
| [`DELIVERY.md`](DELIVERY.md) | 交付清单 |
