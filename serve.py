#!/usr/bin/env python3
"""
启动路径规划 REST 服务（供局域网联调）

用法:
    python serve.py                              # 0.0.0.0:8000，同网段可访问
    python serve.py --port 9000                  # 换端口
    python serve.py --host 127.0.0.1             # 只允许本机访问
    python serve.py --reload                     # 改代码自动重载（开发用）

启动后会打印可直接发给对接方的地址（含 /docs 交互文档）。
"""

from __future__ import annotations

import argparse
import os
import socket
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def lan_ips() -> list:
    """
    找出本机的局域网 IPv4 地址

    两个来源取并集：一是按主机名解析（多网卡时能拿全），二是朝外的 UDP
    套接字（不需要真的联网，只为让内核挑出默认出口网卡）。单靠前者在
    macOS 上常只拿到回环地址。
    """
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except (socket.gaierror, OSError):
        pass

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ips.add(s.getsockname()[0])
        finally:
            s.close()
    except OSError:
        pass

    return sorted(ip for ip in ips if not ip.startswith("127."))




def main():
    parser = argparse.ArgumentParser(description="启动路径规划 REST 服务")
    parser.add_argument("--host", default="0.0.0.0",
                        help="监听地址。0.0.0.0 = 同网段可访问（联调默认）；"
                             "127.0.0.1 = 仅本机")
    parser.add_argument("--port", type=int, default=8000, help="监听端口")
    parser.add_argument("--reload", action="store_true",
                        help="代码变更自动重载（开发用，生产勿开）")
    parser.add_argument("--workers", type=int, default=1,
                        help="uvicorn 进程数。规划是 CPU 密集且内部已多线程并行，"
                             "多进程会互相抢核；异步作业表也是进程内的，"
                             "多进程会导致轮询落到没有该作业的进程，保持 1")
    parser.add_argument("--log-level", default="info", help="日志级别")
    args = parser.parse_args()

    try:
        import uvicorn
    except ImportError:
        print("[!] 缺少依赖 uvicorn。请先安装：")
        print("    pip install 'fastapi>=0.110' 'uvicorn[standard]>=0.27'")
        sys.exit(1)

    print("=" * 62)
    print("  低空无人机路径规划算法 —— REST 服务")
    print("=" * 62)
    print(f"  监听: {args.host}:{args.port}")
    print()
    print("  本机访问:")
    print(f"    交互文档  http://127.0.0.1:{args.port}/docs")
    print(f"    健康检查  http://127.0.0.1:{args.port}/api/v1/health")

    if args.host == "0.0.0.0":
        ips = lan_ips()
        if ips:
            print()
            print("  局域网访问:")
            for ip in ips:
                print(f"    http://{ip}:{args.port}/docs")
        else:
            print()
            print("  [!] 未探测到局域网地址，请用 `ipconfig getifaddr en0` 自行确认")

        print()
        print("  注意：本服务不做认证。请确保处于可信网络，")
        print("       若需公网访问，务必在前面加一层边缘鉴权，详见 internal/docs/DEPLOYMENT.md")

    print("=" * 62)
    print()

    uvicorn.run(
        "server.app:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        workers=args.workers,
        log_level=args.log_level,
    )


if __name__ == "__main__":
    main()
