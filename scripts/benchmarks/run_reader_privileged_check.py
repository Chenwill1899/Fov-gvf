#!/usr/bin/env python3
"""Compatibility entry point for the current P2 FIFO5 step-1 check."""
import runpy
from pathlib import Path
runpy.run_path(str(Path(__file__).with_name('run_step1_fifo.py')), run_name='__main__')
