#!/usr/bin/env python3
"""Check the user-supplied controller interface without launching it."""
import os
from pathlib import Path
import sys
from base_config import load_config

def controller_path():
    if not load_config()[0]["motion_output_enabled"]:
        raise ValueError("EGO4P2第1步仅基础配置/只读输入验收；新运动输出尚未启用。")
    raw=os.environ.get('EGO4P2_CONTROLLER_EXECUTABLE','')
    if not raw:
        raise ValueError('EGO4P2尚未实现新控制算法。请先实现新节点，并设置EGO4P2_CONTROLLER_EXECUTABLE绝对路径。接口见docs/history/NEW_CONTROLLER_INTERFACE.md。')
    path=Path(raw).expanduser()
    root=Path(__file__).resolve().parents[1]
    if not path.is_absolute() or not path.is_file() or not os.access(path,os.X_OK):
        raise ValueError('新控制器必须是存在且可执行的绝对路径')
    resolved=path.resolve()
    if not resolved.is_relative_to(root) or path.name in {'depth_angular_controller','depth_angular_controller_goal','paper_replay','paper_replay_goal'}:
        raise ValueError('控制器必须来自EGO4P2内部，不能回退到旧版控制器')
    return str(resolved)

if __name__=='__main__':
    try:print(controller_path())
    except ValueError as exc:
        print(str(exc),file=sys.stderr);raise SystemExit(2)
