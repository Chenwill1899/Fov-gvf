"""Shared goal input and arrival criterion; no map or planner access."""
import math


class GoalTrial:
    def __init__(self, protocol, exploration_stall_s=0.):
        self.protocol = protocol
        self.goal = tuple(protocol['goal'])
        self.warmup = protocol['warmup_s']
        self.wall_start = None
        self.hold_start = None
        self.stall_start = None
        self.exploration_stall_s = max(0., exploration_stall_s)

    def intent(self, position, now):
        delta = tuple(g - p for g, p in zip(self.goal, position))
        distance = math.sqrt(sum(x*x for x in delta))
        if now < self.warmup or distance <= self.protocol['input_deadband_m']:
            return (0., 0., 0.)
        speed = min(self.protocol['max_speed_mps'], distance)
        scale = speed / distance
        if abs(delta[2]) > 1e-9:
            scale = min(scale, self.protocol['max_vertical_speed_mps'] / abs(delta[2]))
        return tuple(scale*x for x in delta)

    def update(self, position, velocity, now, wall, blocked):
        if now < self.warmup:
            return 'RUNNING'
        if self.wall_start is None:
            self.wall_start = wall
        if blocked:
            return 'COLLISION_GUARD'
        distance = math.dist(position, self.goal)
        speed = math.sqrt(sum(v*v for v in velocity))
        if distance <= self.protocol['arrival_radius_m'] and speed <= self.protocol['arrival_speed_mps']:
            if self.hold_start is None:
                self.hold_start = now
            if now - self.hold_start >= self.protocol['arrival_hold_s']:
                return 'ARRIVED'
        else:
            self.hold_start = None
        if self.exploration_stall_s > 0.:
            has_input = any(abs(x)>1e-6 for x in self.intent(position, now))
            if has_input and speed < .02:
                if self.stall_start is None:
                    self.stall_start = now
                if now-self.stall_start >= self.exploration_stall_s:
                    return 'ABORTED_EXPLORATION_STALL'
            else:
                self.stall_start = None
        if now - self.warmup >= self.protocol['timeout_s']:
            return 'TIMEOUT'
        return 'RUNNING'

    def result(self, status, position, now, wall, blocks):
        return dict(status=status, simulation_elapsed_s=max(0., now-self.warmup),
                    wall_elapsed_s=wall-self.wall_start if self.wall_start is not None else None,
                    final_position=list(position), goal=list(self.goal),
                    final_distance_m=math.dist(position, self.goal), collision_blocks=blocks,
                    protocol=self.protocol, exploration_stall_s=self.exploration_stall_s)
