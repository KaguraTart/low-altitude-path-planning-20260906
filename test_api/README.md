# 接口测试包

本目录提供完整输入、测试脚本和测试报告。脚本通过 HTTPS 调用当前 `/api/v2` 接口。

默认服务地址为 `https://api.kaguratart.com`。Swagger 地址为 [`https://api.kaguratart.com/docs#/`](https://api.kaguratart.com/docs#/)。

接口字段、测试 JSON 参数和修改步骤见 [接口字段与修改说明](API_FIELDS.md)。

按业务需求选择接口，见[需求与 API 对照表](REQUIREMENTS_API_MAPPING.md)。表中包含 16 项需求、完整接口地址、参数用法和调用示例。

完整公网实测记录见 [测试报告](reports/20261017T135804Z_public/REPORT.md)。

## 运行测试

使用 Python 3.11 或更新版本。脚本只使用标准库。

复制整个 `test_api` 目录。打开终端，进入该目录。

```bash
cd test_api
python run_tests.py
```

默认运行接口示例和三套批量数据。脚本保存完整请求、完整响应、JSON 报告和 Markdown 报告。默认输出目录为 `results/<UTC时间>/`。

脚本返回 `0` 时，全部检查通过。脚本返回 `1` 时，打开 `REPORT.md` 查看失败记录。

## 选择测试

只检查示例和全部 14 个接口：

```bash
python run_tests.py --examples-only
```

检查交付数据的 SHA-256：

```bash
python run_tests.py --verify-data
```

指定服务地址和输出目录：

```bash
python run_tests.py --base-url https://api.kaguratart.com --output results/manual
```

每次为 `--output` 指定新目录。

`--base-url` 也接受 `https://api.kaguratart.com/docs#/`。脚本使用其中的服务地址。

设置 HTTP 请求等待时间和异步作业轮询时间：

```bash
python run_tests.py --timeout 120 --poll-timeout 300
```

这两个参数使用秒。它们控制测试客户端的等待时间。

## 测试内容

| 测试 | 检查内容 |
|---|---|
| 服务与文档 | 健康状态、作业计数、服务版本、能力、Swagger 和 OpenAPI |
| 场景 | 注册场景，按场景 ID 查询编译结果 |
| 单任务 | 使用 `strict`、`prefer`、`free` 规划，并验证完整轨迹 |
| 坐标 | 执行 WGS84 到局部米制转换，再执行反向转换 |
| 批量示例 | 提交完整批量输入，检查成功数量、轨迹验证和冲突结果 |
| 异步作业 | 提交批量输入，轮询作业，检查完整结果 |
| 错误输入 | 检查未知字段的 400 响应，以及未知场景、未知作业的 404 响应 |
| 接口示例 | 获取单任务和批量示例，并提交示例中的输入 |
| 三套批量数据 | 分别提交 strict、mixed、free 的 1000 个任务 |

脚本从 OpenAPI 读取接口清单。报告逐项列出全部 14 个 v2 接口的 HTTP 调用结果。检查记录列出业务验证结果。

## 输入数据

| 文件 | 用途 |
|---|---|
| `data/examples/single_strict_request.json` | 严格航路单机规划 |
| `data/examples/single_prefer_request.json` | 优先航路单机规划 |
| `data/examples/single_free_request.json` | 自由空域单机规划 |
| `data/examples/batch_request.json` | 同步批量规划和异步作业 |
| `data/examples/coordinates_to_local_request.json` | WGS84 与米制坐标转换 |
| `data/examples/validate_request.json` | 轨迹独立验证 |
| `data/benchmarks/{strict,mixed,free}_scene.json` | 三套完整场景，提交到场景登记接口 |
| `data/benchmarks/{strict,mixed,free}_tasks.json` | 每套 1000 个完整任务 |

脚本先登记场景，再将返回的 `scene_id` 与任务数组组合成批量请求。实际发送的组合请求保存在 `http/*_input.json` 中。

WKT 坐标顺序为经度、纬度、高度。数据使用 WGS84 经纬度和 MSL 高度，高度单位为米。混合任务包含低电量返航。脚本按 `state.home_wkt` 检查返航终点。

## 文件

| 路径 | 内容 |
|---|---|
| `API_FIELDS.md` | 全部接口的输入输出字段、单位、默认值和修改方法 |
| `REQUIREMENTS_API_MAPPING.md` | 16 项业务需求与 API 的对应关系、参数和调用方法 |
| `run_tests.py` | 命令行测试入口 |
| `client.py` | HTTPS 调用和完整 HTTP 记录 |
| `reporting.py` | Markdown 报告生成器 |
| `data/examples/` | 六组完整接口请求 |
| `data/benchmarks/` | 三套场景和三套各 1000 个任务 |
| `data/manifest.json` | 数据文件清单与 SHA-256 |
| `reports/` | 本次交付的公网测试报告和完整 HTTP 记录 |
| `results/` | 使用者运行脚本后生成的结果 |

每次测试的输出目录包含以下文件：

| 路径 | 内容 |
|---|---|
| `REPORT.md` | 北京时间、服务版本、检查结果和接口覆盖 |
| `report.json` | 完整测试状态、耗时、接口覆盖和 HTTP 交换记录 |
| `http/*_request.json` | HTTP 方法、地址、请求头和输入文件路径 |
| `http/*_input.json` | 实际发送的完整 JSON 输入 |
| `http/*_output.json`、`http/*_output.html` | 实际收到的完整响应 |

读取 `reports/` 中报告的服务提交 SHA，确认该次结果对应的运行版本。重跑脚本会记录当前服务版本。
