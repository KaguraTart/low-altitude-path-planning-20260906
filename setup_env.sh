#!/usr/bin/env bash
# 一键环境配置 + 项目运行脚本
#
# 用法:
#   ./setup_env.sh             # 创建/激活 conda 环境并安装依赖
#   ./setup_env.sh plan        # 仅运行 demo 规划
#   ./setup_env.sh stress      # 仅运行压力测试
#   ./setup_env.sh visualize   # 为已有产物生成可视化
#   ./setup_env.sh all         # 完整流程: 规划 + 压力测试 + 可视化

set -e

ENV_NAME="uav_path_planning"
PY_VERSION="3.11"

# 1. 创建环境（如果已存在则跳过）
if ! conda env list | grep -q "^${ENV_NAME} "; then
    echo "[+] 创建 conda 环境: ${ENV_NAME} (python=${PY_VERSION})"
    conda create -n "${ENV_NAME}" "python=${PY_VERSION}" -y
fi

# 2. 安装依赖
echo "[+] 安装/更新依赖 (numpy, scipy, matplotlib, plotly, pydantic)"
conda install -n "${ENV_NAME}" -y \
    "numpy>=1.24.0" "scipy>=1.10.0" "matplotlib>=3.7.0" \
    "plotly>=5.15.0" "pydantic>=2.0.0"

# 3. 进入项目目录
cd "$(dirname "$0")"

# 4. 验证安装
echo "[+] 验证依赖..."
conda run -n "${ENV_NAME}" python -c \
    "import numpy, scipy, matplotlib, plotly, pydantic; print('  ✓ plotly', plotly.__version__)"

# 5. 根据参数执行任务
case "${1:-}" in
    plan)
        echo "[+] 单机规划..."
        conda run -n "${ENV_NAME}" python plan_single.py --output-dir ./output/single "${@:2}"
        conda run -n "${ENV_NAME}" python visualize.py --input ./output/single

        echo "[+] 多机批量规划..."
        conda run -n "${ENV_NAME}" python plan_multi.py --output-dir ./output/multi --conflict-check
        conda run -n "${ENV_NAME}" python visualize.py --input ./output/multi
        ;;
    single)
        echo "[+] 单机规划..."
        conda run -n "${ENV_NAME}" python plan_single.py --output-dir ./output/single "${@:2}"
        conda run -n "${ENV_NAME}" python visualize.py --input ./output/single
        ;;
    multi)
        echo "[+] 多机批量规划..."
        conda run -n "${ENV_NAME}" python plan_multi.py --output-dir ./output/multi --conflict-check
        conda run -n "${ENV_NAME}" python visualize.py --input ./output/multi
        ;;
    stress)
        echo "[+] 运行压力测试 (10/100/1000/10000, FULL vs DISPATCH)..."
        conda run -n "${ENV_NAME}" python -u run_stress_test.py \
            --scales 10 100 1000 10000 --workers 8 --max-iterations 50000 \
            --compare --output ./output/stress "$@"
        conda run -n "${ENV_NAME}" python visualize_stress.py --input ./output/stress
        ;;
    visualize)
        echo "[+] 为已有产物生成可视化..."
        for d in ./output/single ./output/multi; do
            if [ -d "$d" ]; then
                conda run -n "${ENV_NAME}" python visualize.py --input "$d"
            fi
        done
        if [ -d ./output/stress ]; then
            conda run -n "${ENV_NAME}" python visualize_stress.py --input ./output/stress
        fi
        ;;
    all)
        echo "[+] 完整流程..."
        conda run -n "${ENV_NAME}" python plan_single.py --output-dir ./output/single
        conda run -n "${ENV_NAME}" python visualize.py --input ./output/single
        conda run -n "${ENV_NAME}" python plan_multi.py --output-dir ./output/multi --conflict-check
        conda run -n "${ENV_NAME}" python visualize.py --input ./output/multi
        conda run -n "${ENV_NAME}" python -u run_stress_test.py \
            --scales 10 100 1000 10000 --workers 8 --max-iterations 50000 \
            --compare --output ./output/stress "$@"
        conda run -n "${ENV_NAME}" python visualize_stress.py --input ./output/stress
        ;;
    *)
        echo ""
        echo "[✓] 环境已就绪: conda activate ${ENV_NAME}"
        echo "  可执行命令:"
        echo "    ./setup_env.sh plan         # 单机 + 多机批量 + 可视化"
        echo "    ./setup_env.sh single       # 仅单机规划"
        echo "    ./setup_env.sh multi        # 仅多机批量"
        echo "    ./setup_env.sh stress       # 压力测试 + 可视化"
        echo "    ./setup_env.sh visualize    # 为已有产物生成可视化"
        echo "    ./setup_env.sh all          # 全流程"
        echo "    conda activate ${ENV_NAME}  # 手动进入环境"
        ;;
esac