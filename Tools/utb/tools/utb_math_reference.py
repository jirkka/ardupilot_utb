#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline real-arithmetic PID/allocation reference with forward float32 error bounds.

Geometry, interval projection and closed-form recurrence define this model.
It neither imports production code nor uses a float32 copy of that code.
"""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_math_reference.py
# @brief      Nezávislé reálné PID/allocator rovnice a float32 forward error meze.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
from dataclasses import dataclass
import math
import struct

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Static data declarations
# --------------------------------------------------------------------------------------------------------------------
U = 2 ** -24
TINY = 2 ** -149


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Static data declarations
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      F32.
#
# @param[in]  value Value supplied to this operation.
# @return     Returned values: struct.unpack('<f', struct.pack('<f', value))[0].
#
# @details    Global module function. Calls: struct.unpack, struct.pack.
# --------------------------------------------------------------------------------------------------------------------
def f32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Static data declarations
# --------------------------------------------------------------------------------------------------------------------
EPS = f32(1e-5)
NEAR_ONE = f32(1 - EPS)


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Static data declarations
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
@dataclass
class Bound:
    value: float
    error: float = 0

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Add.
    #
    # @param[in]  other Other value used for comparison.
    # @return     Returned values: Bound(value, self.error + other.error + U * abs(value) + TINY).
    #
    # @details    Public class method. Calls: as_bound, Bound, abs.
    # --------------------------------------------------------------------------------------------------------------------
    def __add__(self, other):
        other = as_bound(other)
        value = self.value + other.value
        return Bound(value, self.error + other.error + U * abs(value) + TINY)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Neg.
    #
    # @param[in]  None
    # @return     Returned values: Bound(-self.value, self.error).
    #
    # @details    Public class method. Calls: Bound.
    # --------------------------------------------------------------------------------------------------------------------
    def __neg__(self):
        return Bound(-self.value, self.error)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Sub.
    #
    # @param[in]  other Other value used for comparison.
    # @return     Returned values: self + -as_bound(other).
    #
    # @details    Public class method. Calls: as_bound.
    # --------------------------------------------------------------------------------------------------------------------
    def __sub__(self, other):
        return self + -as_bound(other)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Mul.
    #
    # @param[in]  other Other value used for comparison.
    # @return     Computed result of this operation.
    #
    # @details    Public class method. Calls: as_bound, Bound, abs.
    # --------------------------------------------------------------------------------------------------------------------
    def __mul__(self, other):
        other = as_bound(other)
        value = self.value * other.value
        return Bound(value, abs(self.value) * other.error + abs(other.value) * self.error
                     + self.error * other.error + U * abs(value) + TINY)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Truediv.
    #
    # @param[in]  other Other value used for comparison.
    # @return     Returned values: Bound(value, (self.error + abs(value) * other.error) / bottom + U * abs(value) + TINY).
    #
    # @details    Public class method. Calls: as_bound, Bound, abs.
    # --------------------------------------------------------------------------------------------------------------------
    def __truediv__(self, other):
        other = as_bound(other)
        value = self.value / other.value
        bottom = abs(other.value) - other.error
        assert bottom > 0
        return Bound(value, (self.error + abs(value) * other.error) / bottom + U * abs(value) + TINY)


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      As bound.
#
# @param[in]  value Value supplied to this operation.
# @return     Returned values: value if isinstance(value, Bound) else Bound(value).
#
# @details    Global module function. Calls: isinstance, Bound.
# --------------------------------------------------------------------------------------------------------------------
def as_bound(value):
    return value if isinstance(value, Bound) else Bound(value)


# --------------------------------------------------------------------------------------------------------------------
# @brief      Bound any summation order; signs and powers of two are exact in this domain.
#
# @param[in]  values Values.
# @return     Computed result of this operation.
#
# @details    Global module function. Calls: list, Bound, map, math.fsum.
# --------------------------------------------------------------------------------------------------------------------
def sum_bound(values):
    """Bound any summation order; signs and powers of two are exact in this domain."""
    values = list(map(as_bound, values))
    gamma = len(values) * U / (1 - len(values) * U)
    return Bound(math.fsum(v.value for v in values), sum(v.error for v in values)
                 + gamma * sum(abs(v.value) for v in values) + len(values) * TINY)


# --------------------------------------------------------------------------------------------------------------------
# @brief      Clamp.
#
# @param[in]  value Value supplied to this operation.
# @param[in]  lower Lower.
# @param[in]  upper Upper.
# @return     Computed result of this operation.
#
# @details    Global module function. Calls: map, Bound, max, min.
# --------------------------------------------------------------------------------------------------------------------
def clamp(value, lower, upper):
    value, lower, upper = map(as_bound, (value, lower, upper))
    return Bound(max(lower.value, min(upper.value, value.value)), max(value.error, lower.error, upper.error))


# --------------------------------------------------------------------------------------------------------------------
# @brief      Geometry.
#
# @param[in]  frame Frame.
# @return     Computed model values for the supplied inputs.
#
# @details    Global module function. Calls: enumerate.
# --------------------------------------------------------------------------------------------------------------------
def geometry(frame):
    # Body FRD: x forward, y right, thrust along -z. r cross F -> (-y, x, 0).
    # M1 rear-right, M2 front-right, M3 rear-left, M4 front-left.
    positions = ((-1, 1), (1, 1), (-1, -1), (1, -1))
    # Rotor viewed from above: BF_X_REV CCW,CW,CW,CCW; reaction reverses rotor torque.
    reaction = (1, -1, -1, 1) if frame == 18 else (-1, 1, 1, -1)
    return [(-y / 2, x / 2, reaction[i] / 2) for i, (x, y) in enumerate(positions)]


# --------------------------------------------------------------------------------------------------------------------
# @brief      Allocate.
#
# @param[in]  command Normalized R/P/Y request.
# @param[in]  collective Collective.
# @param[in]  frame Frame.
# @param[in]  valid Valid.
# @return     Computed model values for the supplied inputs.
#
# @details    Global module function. Calls: dict, any, geometry, min.
# --------------------------------------------------------------------------------------------------------------------
def allocate(command, collective, frame=18, valid=True):
    zeros = {name: Bound(0) for name in
             ('m0', 'm1', 'm2', 'm3', 'a0', 'a1', 'a2', 'at', 'scale', 'shift')}
    flags = dict(mvalid=0, pos=0, neg=0, lo=0, hi=0, scaled=0, shifted=0)
    if not valid or frame not in (12, 18) or not math.isfinite(collective) or not 0 <= collective <= 1:
        return zeros | flags
    if any(not math.isfinite(c.value) or abs(c.value) > 1 for c in command):
        return zeros | flags
    matrix = geometry(frame)
    demand = [sum_bound(c * weight for c, weight in zip(command, row)) for row in matrix]
    low = min(demand, key=lambda v: v.value)
    high = max(demand, key=lambda v: v.value)
    # Extrema are 1-Lipschitz even if rounding changes which motor supplies the extremum.
    extremum_error = max(v.error for v in demand)
    low, high = Bound(low.value, extremum_error), Bound(high.value, extremum_error)
    span = high - low
    scale = Bound(1) if span.value <= 1 else Bound(1) / span
    if span.value <= 1:
        scale.error = span.error  # include uncertainty at the scaling boundary
    projected = clamp(Bound(collective), -scale * low, Bound(1) - scale * high)
    motors = [clamp(projected + scale * v, 0, 1) for v in demand]
    # Columns have unit squared norm and zero sum: transpose is the inverse on the moment subspace.
    achieved = [sum_bound(m * row[j] for m, row in zip(motors, matrix)) for j in range(3)]
    achieved_t = sum_bound(motors) * 0.25
    residual = [c - a for c, a in zip(command, achieved)] + [Bound(collective) - achieved_t]
    flags.update(mvalid=1, pos=sum(1 << i for i, r in enumerate(residual) if r.value > EPS),
                 neg=sum(1 << i for i, r in enumerate(residual) if r.value < -EPS),
                 lo=sum(1 << i for i, m in enumerate(motors) if m.value <= EPS),
                 hi=sum(1 << i for i, m in enumerate(motors) if m.value >= NEAR_ONE),
                 scaled=int(scale.value < NEAR_ONE), shifted=int(abs(projected.value - collective) > EPS))
    return ({f'm{i}': m for i, m in enumerate(motors)} | {f'a{i}': a for i, a in enumerate(achieved)}
            | dict(at=achieved_t, scale=scale, shift=projected - collective) | flags)


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class Reference:
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Initialize the instance from the supplied configuration and collaborators.
    #
    # @param[in]  None
    # @return     None.
    #
    # @details    Public class method. Calls: self.reset.
    # --------------------------------------------------------------------------------------------------------------------
    def __init__(self):
        self.reset()
        self.configuration = None

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Clear the component history so the next valid sample starts a new context.
    #
    # @param[in]  None
    # @return     None.
    #
    # @details    Public class method. Calls: Bound, range.
    # --------------------------------------------------------------------------------------------------------------------
    def reset(self):
        self.previous = None
        self.integral = [Bound(0) for _ in range(3)]
        self.acceleration = [Bound(0) for _ in range(3)]
        self.residual = [Bound(0) for _ in range(3)]

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Step.
    #
    # @param[in]  row Row.
    # @return     Computed model values for the supplied inputs.
    #
    # @details    Public class method. Calls: result.update, allocate, self.reset, Bound.
    # --------------------------------------------------------------------------------------------------------------------
    def step(self, row):
        gains = row['gains']
        if row['sample'] == 0 or row['action'] == 2 or self.configuration != gains:
            self.reset()
        self.configuration = gains
        result = {f'{name}{j}': Bound(0) for j in range(3) for name in ('e', 'p', 'i', 'd', 'raw', 'out', 'acc')}
        result.update({f'blocked{j}': 0 for j in range(3)})
        result.update(valid=0, prime=0)
        command = [Bound(0) for _ in range(3)]
        dt, target, measurement = row['dt'], row['target'], row['measurement']
        valid_input = (math.isfinite(dt) and f32(.0025) * .25 <= dt <= f32(.0025) * 4
                       and row['action'] != 3 and all(math.isfinite(v) for v in target + measurement))
        if row['action'] == 1:
            command = list(map(Bound, target))
            result['valid'] = 1
        elif not valid_input:
            self.reset()
        elif self.previous is None:
            self.previous = measurement.copy()
            result['prime'] = 1
        else:
            for j, (kp, ki, kd, imax, cutoff) in enumerate(gains):
                error = Bound(target[j]) - measurement[j]
                proportional = error * kp
                derivative = Bound(0)
                acceleration = Bound(0)
                if kd:
                    tau = Bound(1) / (Bound(math.pi, U * math.pi) * 2 * cutoff)
                    beta = Bound(dt) / (tau + dt)
                    raw_acceleration = (Bound(measurement[j]) - self.previous[j]) / dt
                    acceleration = sum_bound(((Bound(1) - beta) * self.acceleration[j], beta * raw_acceleration))
                    derivative = acceleration * -kd
                increment = error * ki * dt
                blocked = ((self.residual[j].value > EPS and increment.value > 0)
                           or (self.residual[j].value < -EPS and increment.value < 0))
                integral = clamp(self.integral[j] + (Bound(0) if blocked else increment), -imax, imax) if ki else Bound(0)
                raw = sum_bound((proportional, integral, derivative))
                output = clamp(raw, -1, 1)
                for name, value in zip(('e', 'p', 'i', 'd', 'raw', 'out', 'acc'),
                                       (error, proportional, integral, derivative, raw, output, acceleration)):
                    result[f'{name}{j}'] = value
                result[f'blocked{j}'] = int(blocked)
                command[j] = output
                self.integral[j], self.acceleration[j] = integral, acceleration
            self.previous = measurement.copy()
            result['valid'] = 1
        allocation = allocate(command, row['thrust'], row['frame'], bool(result['valid']))
        if result['valid'] and allocation['mvalid'] and row['action'] != 1:
            self.residual = [result[f'raw{j}'] - allocation[f'a{j}'] for j in range(3)]
        return result | allocation

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Private function prototypes
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private function prototypes
# ====================================================================================================================
# Private functions
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private functions
