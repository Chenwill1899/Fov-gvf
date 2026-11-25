"""Check route intent against independent Euclidean polyline geometry."""
from pathlib import Path
import csv
import json
import sys
import numpy as np

project = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(project / "src/auto_cruise"))
from spherical_guide import Radial, RouteIntent

scenarios = json.loads((project/"src/auto_cruise/scenarios.json").read_text())
for name, scene in scenarios.items():
    file = project/scene["guide"]
    goal = np.asarray(scene["goal"])
    with file.open() as stream:
        xyz = np.array([[float(row[k]) for k in "xyz"] for row in csv.DictReader(stream)])
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))]
    spans = np.arange(0., arc[-1]-.25, .5)
    points = np.vstack([np.column_stack([np.interp(spans, arc, xyz[:,j]) for j in range(3)]), goal])
    intent = RouteIntent.load(file, Radial.from_xyz(goal))
    np.testing.assert_allclose([p.xyz() for p in intent.points], points, atol=2e-10, rtol=0)
    seg = np.diff(points, axis=0)
    length = np.linalg.norm(seg, axis=1)
    arc = np.r_[0., np.cumsum(length)]
    axes = seg/length[:, None]
    bends = np.r_[np.linalg.norm(np.diff(axes, axis=0), axis=1)/(.5*(length[:-1]+length[1:])), 0.]
    for i in range(len(seg)):
        # Offset the measured pose laterally on alternating chords to exercise the 6 m/s cap.
        pose = points[i]+.37*seg[i]+(np.array([0.,.23,0.]) if i%2 else 0.)
        start, end = max(0,intent.index-2), min(len(seg),intent.index+10)
        rel = pose-points[start:end]
        along = np.clip(np.sum(rel*axes[start:end], axis=1), 0., length[start:end])
        miss = np.linalg.norm(rel-axes[start:end]*along[:,None], axis=1)
        j = int(np.argmin(miss))
        progress = max(intent.offset, arc[start+j]+along[j])
        final = arc[-1]-progress < 4.
        aim, last, cap = intent.update(Radial.from_xyz(pose), 11.)
        assert final == last
        s = min(arc[-1], progress+.55+.08*11.)
        expected = goal if final else [np.interp(s, arc, points[:,k]) for k in range(3)]
        np.testing.assert_allclose(aim.xyz(), expected, atol=3e-10, rtol=0)
        idx = min(len(seg)-1,np.searchsorted(arc, progress, side="right")-1)
        stop = max(idx+1,int(np.searchsorted(arc,progress+3.)))
        kappa = np.max(bends[idx:stop])
        grade = np.max(np.abs(axes[idx:stop,2]))
        wanted = 12./(1.+6.*max(0.,kappa-.07)+2.5*max(0.,grade-.12))
        wanted = 12. if final else max(3., min(wanted, 6. if miss[j]>.20 else 12.))
        assert abs(wanted-cap)<1e-8
    print(name, len(seg), "segments verified")
