"""Correctness-only Isaac flights with a frozen per-ball comparator."""
import importlib.util
from pathlib import Path
import sys
out=Path(__file__).resolve().parent
app=out.parents[2]/'src/auto_cruise/supervise.py'
spec=importlib.util.spec_from_file_location('adaptive_supervision',app)
supervisor=importlib.util.module_from_spec(spec)
spec.loader.exec_module(supervisor)
supervisor.EXPERIMENT=out/'parity'
if not any(arg in sys.argv[1:] for arg in ('--legacy-spatial','--direct-only','--directional')):
    sys.argv.append('--legacy-spatial')
sys.argv.extend(['--headless','--correctness-only','--seconds','40','--time-scale','1'])
supervisor.main()
