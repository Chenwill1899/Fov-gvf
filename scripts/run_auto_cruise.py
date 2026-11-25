#!/usr/bin/python3
"""Launch spherical directional-depth cruise in Isaac with five audited FIFO5 threads."""
import argparse
import os
from pathlib import Path

def add_policy_arguments(parser):
    policies = parser.add_mutually_exclusive_group()
    policies.add_argument('--directional', dest='policy', action='store_const', const='directional',
                          help='follow a spherical streamline with depth-certified braking (default)')
    policies.add_argument('--legacy-spatial', dest='policy', action='store_const', const='legacy-spatial',
                          help='run the previous spatial-reference cruise policy for comparison')
    policies.add_argument('--direct-only', dest='policy', action='store_const', const='direct-only',
                          help='run the historical cruise policy with candidate search disabled')
    parser.set_defaults(policy='directional')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=['r05','hangar'],default='r05')
    parser.add_argument('--headless',action='store_true')
    add_policy_arguments(parser)
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--output-root',type=Path,help='directory for simulation runs and caches')
    args=parser.parse_args()
    app=Path(__file__).resolve().parents[1]/'src/auto_cruise/supervise.py'
    supervisor=['/usr/bin/python3','-IB',str(app),'--scenario',args.scenario,'--time-scale','1',
                '--seconds','40' if args.scenario=='r05' else '16']
    # Resolve the caller's state directory before sudo changes the environment.
    output_root = args.output_root or Path(os.environ.get('XDG_STATE_HOME') or Path.home()/'.local/state')/'ego4p4'
    supervisor += ['--output-root',str(output_root.resolve())]
    supervisor.append('--'+args.policy)
    if args.headless: supervisor.append('--headless')
    if args.check:
        cmd=[*supervisor,'--check']
    else:
        if os.getuid()==0: parser.error('launch as the ordinary project user')
        cmd=['/usr/bin/sudo','/usr/bin/prlimit','--rtprio=0:5','--rttime=5000000:6000000',
             '--','/usr/bin/setpriv',f'--reuid={os.getuid()}', f'--regid={os.getgid()}','--init-groups',
             '--bounding-set=-all','--inh-caps=-all','--ambient-caps=-all','--no-new-privs','--',*supervisor]
    os.execv(cmd[0],cmd)

if __name__=='__main__':
    main()
