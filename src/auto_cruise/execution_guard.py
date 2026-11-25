"""Execution end of the private, lockstep simulator/controller pipe."""
import math
import os
import select
import struct
import time

RECEIPT = struct.Struct('=QQ3d')   # applied command ID, completed physics step, actual input
REPLY = struct.Struct('=4dQ')      # requested input, navigation mode, command ID
ZERO = (0., 0., 0.)


def exchange(child, chunks, wall_timeout):
    # Simulation time is frozen throughout the transaction, including on timeout.
    end = time.monotonic() + wall_timeout
    writer, reader = child.stdin.fileno(), child.stdout.fileno()
    for chunk in chunks:
        rest = memoryview(chunk).cast('B')
        while rest:
            if not select.select([], [writer], [], max(0., end-time.monotonic()))[1]:
                raise TimeoutError("command input timeout")
            n = os.write(writer, rest[:4096])
            rest = rest[n:]
    reply = bytearray()
    while len(reply) < REPLY.size:
        if not select.select([reader], [], [], max(0., end-time.monotonic()))[0]:
            raise TimeoutError("command output timeout")
        part = os.read(reader, REPLY.size-len(reply))
        if not part:
            raise EOFError("controller closed pipe")
        reply.extend(part)
    return REPLY.unpack(reply)


class ExecutionGuard:
    def __init__(self):
        self.step = self.applied_id = 0
        self.applied = ZERO
        self.fault = None
        self.rearmable = False
        self.rest_steps = 0
        self.stop_anchor = None

    def receipt(self):
        return RECEIPT.pack(self.applied_id, self.step, *self.applied)

    def request(self, reply):
        *command, mode, command_id = reply
        if (command_id != self.step+1 or mode not in (1, 2, 3, 5, 7, 8)
                or not all(math.isfinite(x) for x in command)
                or (mode in (3, 5) and tuple(command) != ZERO)):
            self.trip("invalid command, receipt mismatch or uncertified brake")
        if mode in (3, 5):
            self.trip("navigation transferred to brake")
        return (ZERO, 9) if self.fault else (tuple(command), int(mode))

    def trip(self, reason, rearmable=False):
        if self.fault is None:
            self.fault = reason
            self.rearmable = rearmable

    def applied_step(self, command, position, velocity, navigation=True):
        # Called only after the actuator integration and independent sweep succeed.
        self.step += 1
        self.applied = tuple(command)
        if not self.fault and navigation:
            self.applied_id = self.step
        elif self.applied != ZERO:
            raise ValueError("nonzero command after brake takeover")
        if self.fault and math.dist(velocity, ZERO) <= 1e-8:
            if self.stop_anchor is None or math.dist(self.stop_anchor, position) > 1e-8:
                self.stop_anchor = tuple(position)
                self.rest_steps = 0
            self.rest_steps += 1
        else:
            self.stop_anchor = None
            self.rest_steps = 0
