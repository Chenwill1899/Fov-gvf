"""Geometry-only route intent; positions are bearings and ranges in the world frame."""
from bisect import bisect_right
import csv
from math import atan2, cos, hypot, isfinite, pi, remainder, sin
from typing import NamedTuple


class Radial(NamedTuple):
    az: float
    el: float
    r: float

    @classmethod
    def from_xyz(cls, xyz):
        x, y, z = xyz
        if not all(map(isfinite, xyz)):
            raise ValueError("nonfinite route or physical position")
        h = hypot(x, y)
        return cls(atan2(y, x), atan2(z, h), hypot(z, h))

    def xyz(self):
        horizontal = self.r*cos(self.el)
        return horizontal*cos(self.az), horizontal*sin(self.az), self.r*sin(self.el)


def separation(a, b):
    """Tangent east/north projections of b and cosine of its angle to a."""
    da = remainder(b.az-a.az, 2*pi)
    cd = cos(da)
    ca, cb = cos(a.el), cos(b.el)
    sa, sb = sin(a.el), sin(b.el)
    east = cb*sin(da)
    north = (sin(b.el-a.el)+sa*cb*(1-cd) if cd >= 0
             else sin(b.el+a.el)-sa*cb*(1+cd))
    return east, north, max(-1., min(1., sa*sb+ca*cb*cd))


def add(a, b, k=1.):
    """Compose two radial displacements in their common great-circle plane."""
    br = b.r*k
    if br == 0.:
        return a
    east, north, c = separation(a, b)
    n = hypot(east, north)
    along = ((a.r+br) - br*n*n/(1+c) if c >= 0.
             else (a.r-br) + br*n*n/(1-c))
    side = br*n
    radius = hypot(along, side)
    if n == 0.:
        return (Radial(a.az, a.el, radius) if along >= 0. else
                Radial(remainder(a.az+pi, 2*pi), -a.el, radius))
    arc = atan2(side, along)
    sa, ca, s = sin(a.el), cos(a.el), sin(arc)
    meridian = ca*cos(arc)-sa*s*north/n
    across = s*east/n
    return Radial(remainder(a.az+atan2(across, meridian), 2*pi),
                  atan2(sa*cos(arc)+ca*s*north/n, hypot(meridian, across)), radius)


class Polyline:
    def __init__(self, points):
        self.points, self.segments, self.arc = [points[0]], [], [0.]
        for p in points[1:]:
            segment = add(p, self.points[-1], -1.)
            if segment.r <= 1e-12:
                continue
            self.points.append(p)
            self.segments.append(segment)
            self.arc.append(self.arc[-1]+segment.r)

    def index_at(self, s):
        return min(len(self.segments)-1, max(0, bisect_right(self.arc, s)-1))

    def at(self, s):
        if s >= self.arc[-1]:
            return self.points[-1]
        i = self.index_at(s)
        return add(self.points[i], self.segments[i], (s-self.arc[i])/self.segments[i].r)


class RouteIntent(Polyline):
    def __init__(self, points):
        super().__init__(points)
        self.index = 0
        self.offset = 0.
        self.bend = []
        self.grade = [abs(sin(s.el)) for s in self.segments]
        for first, second in zip(self.segments, self.segments[1:]):
            e, n, c = separation(first, second)
            angle = atan2(hypot(e, n), c)
            self.bend.append(2*sin(angle/2)/(.5*(first.r+second.r)))
        self.bend.append(0.)

    @classmethod
    def load(cls, path, goal):
        if not path:
            return cls([goal])
        with path.open(newline="") as stream:
            source = [Radial.from_xyz(tuple(float(row[k]) for k in "xyz"))
                      for row in csv.DictReader(stream)]
        if not source:
            raise ValueError("empty route")
        guide = Polyline(source)
        sampled = []
        while .5*len(sampled) < guide.arc[-1]-.25:
            sampled.append(guide.at(.5*len(sampled)))
        return cls([*sampled, goal])

    def update(self, pose, speed):
        if not self.segments:
            return self.points[-1], True, 12.
        nearest = float("inf")
        progress = self.offset
        for i in range(max(0, self.index-2), min(len(self.segments), self.index+10)):
            rel = add(pose, self.points[i], -1.)
            e, n, c = separation(self.segments[i], rel)
            along = min(self.segments[i].r, max(0., rel.r*c))
            miss = hypot(rel.r*hypot(e, n), rel.r*c-along)
            if miss < nearest:
                nearest = miss
                progress = self.arc[i]+along
        self.offset = max(self.offset, progress)
        self.index = self.index_at(self.offset)
        if self.arc[-1]-self.offset < 4.:
            return self.points[-1], True, 12.
        target = self.at(min(self.arc[-1], self.offset+.55+.08*speed))
        last = self.index+1
        while last < len(self.segments) and self.arc[last] < self.offset+3.:
            last += 1
        bend = max(self.bend[self.index:last])
        grade = max(self.grade[self.index:last])
        cap = 12./(1.+6.*max(0., bend-.07)+2.5*max(0., grade-.12))
        return target, False, max(3., min(cap, 6. if nearest > .20 else 12.))
