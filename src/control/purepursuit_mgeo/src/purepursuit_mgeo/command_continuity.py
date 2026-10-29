"""Bounded continuation of a known command over a short transport gap.

The original receipt time is immutable. Calling this repeatedly never renews
the lease. Current stop decisions and mission ownership are checked by caller.
"""
import copy
import math


def continue_command(message, received, now, timeout, grace, healthy,
                     accel_cap=0.35):
    if not healthy or message is None or received is None:
        return None
    values=(received,now,timeout,grace,accel_cap)
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
        return None
    if not 0 < grace <= .3 or not 0 <= now-received-timeout <= grace:
        return None
    try:
        a,b,s=(float(getattr(message,k)) for k in ('accel','brake','steering'))
        if (message.longlCmdType != 1 or not all(math.isfinite(v) for v in (a,b,s))
                or not 0 <= a <= 1 or b != 0 or abs(s) > .7):
            return None
    except (AttributeError,TypeError,ValueError,OverflowError):
        return None
    result=copy.deepcopy(message)
    result.accel=min(a,max(0.,min(1.,accel_cap)))
    result.velocity=result.acceleration=0.
    return result
