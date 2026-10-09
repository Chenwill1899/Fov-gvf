"""New magnitude-free descriptive assessment; no import of the 20261007 policy."""
from __future__ import annotations
import hashlib
import json
import math
import random
import statistics
import itertools
from pathlib import Path


def load(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def evidence(path):
    return {"path": str(Path(path).resolve()), "sha256": sha(path)}


def write_new(path, value):
    path=Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as out:
        json.dump(value, out, indent=2, allow_nan=False)
        out.write("\n")


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def quantile(values, probability):
    a=sorted(values)
    u=(len(a)-1)*probability
    i=int(u)
    return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(u-i)


def summary(values):
    if not values or not all(finite(x) for x in values):
        raise ValueError("nonempty finite numeric sample required")
    return dict(n=len(values), mean=statistics.mean(values), p50=quantile(values,.5),
        p95=quantile(values,.95), max=max(values), min=min(values),
        std=statistics.stdev(values) if len(values)>1 else 0.)


def direction_class(ci, good, bad, n):
    if n<3:return "insufficient_repeats"
    if good==bad==0:return "unchanged"
    if ci[0]>0 and good/n>=.75:return "reproducible_gain"
    if ci[1]<0 and bad/n>=.75:return "reproducible_regression"
    return "unresolved_variation"


def paired(before, after, higher_better=False, draws=20000):
    if len(before)!=len(after) or not before:
        raise ValueError("equal nonempty paired samples required")
    b=summary(before);a=summary(after);n=len(before)
    delta=[(y-x) if higher_better else (x-y) for x,y in zip(before,after)]
    good=sum(v>0 for v in delta);bad=sum(v<0 for v in delta)
    # Absolute improvements avoid undefined relative changes when baseline is zero.
    if n<=5:
        boot=[statistics.mean(delta[i] for i in index) for index in itertools.product(range(n),repeat=n)]
        method="exact empirical paired bootstrap over all n^n resamples"
    else:
        rng=random.Random(20261008)
        boot=[statistics.mean(delta[rng.randrange(n)] for _ in range(n)) for _ in range(draws)]
        method="20000 deterministic paired bootstrap resamples, seed 20261008"
    ci=[quantile(boot,.025),quantile(boot,.975)]
    nt=good+bad
    return dict(before=before,after=after,before_summary=b,after_summary=a,
        absolute_improvement=statistics.mean(delta),paired_improvements=delta,
        relative_improvement=statistics.mean(delta)/abs(b["mean"]) if b["mean"] else None,
        bootstrap_method=method,absolute_bootstrap_95pct_ci=ci,improved_pairs=good,worsened_pairs=bad,tied_pairs=n-nt,
        sign_p_benefit=sum(math.comb(nt,k) for k in range(good,nt+1))/2**nt if nt else 1.,
        classification=direction_class(ci,good,bad,n))
