# 环境配置说明

## 版本要求

| 工具或语言 | 最低要求 |
| ---------- | -------- |
| Python     | 3.10     |
| C++        | C++17    |
| CMake      | 3.16     |

项目不依赖额外的 C++ 库。C++ 编译器只需要支持 C++17，Windows、Linux
和 macOS 均可使用对应平台的编译器。

## C++环境

`cpp/CMakeLists.txt` 使用以下构建设置：

- CMake 最低版本：`3.16`
- C++ 标准：`C++17`
- 构建目标：`pyrobo_contestant`
- 构建类型：默认使用 `Release`

C++和Cmake环境安装请查看笔试题统一撰写的环境安装教程。

检查编译器和 CMake：

```text
cmake --version
c++ --version
```

只构建 C++ 部分：

```text
python python/scripts/build.py
```

清理后重新构建：

```text
python python/scripts/build.py --clean
```

## Python环境

Python 最低版本为 3.10。项目依赖写在根目录的 `requirements.txt` 中，
当前固定为以下 4 个包：

- `numpy==2.2.6`
- `Pillow==12.3.0`
- `pygame==2.6.1`
- `PyYAML==6.0.3`

Windows：

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Linux/macOS：

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

检查 Python 版本：

```text
python --version
```

## 构建并运行

在项目根目录执行。脚本会先构建 C++ 答题代码，再启动 Python 仿真器。

Windows：

```text
run.bat
```

Linux/macOS：

```sh
./run.sh
```

如果只想查看仿真界面：

```text
Windows:   run.bat --display
Linux/macOS: ./run.sh --display
```

运行未知地图场景：

```text
Windows:   run.bat --scenario unknown
Linux/macOS: ./run.sh --scenario unknown
```

