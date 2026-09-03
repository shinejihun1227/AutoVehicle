"""ROS에 의존하지 않는 odometry noise/filter 계산부."""

from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def median_angle(values: List[float], reference: Optional[float] = None) -> float:
    """각도 wrap-around를 고려한 median을 계산한다."""

    if not values:
        raise ValueError("각도 값이 비어 있다.")
    ref = values[0] if reference is None else reference
    unwrapped = [ref + wrap_angle(value - ref) for value in values]
    ordered = sorted(unwrapped)
    return float(ordered[len(ordered) // 2])


@dataclass(frozen=True)
class MotionState:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float


class OdometryNoiseModel:
    """위치·yaw·속도에 white noise와 bias random walk를 추가한다."""

    def __init__(
        self,
        position_std_m: float,
        yaw_std_rad: float,
        velocity_std_mps: float,
        position_bias_rw_m_sqrt_s: float,
        yaw_bias_rw_rad_sqrt_s: float,
        velocity_bias_rw_mps_sqrt_s: float,
        seed: int,
    ) -> None:
        self.position_std = max(0.0, float(position_std_m))
        self.yaw_std = max(0.0, float(yaw_std_rad))
        self.velocity_std = max(0.0, float(velocity_std_mps))
        self.position_bias_rw = max(0.0, float(position_bias_rw_m_sqrt_s))
        self.yaw_bias_rw = max(0.0, float(yaw_bias_rw_rad_sqrt_s))
        self.velocity_bias_rw = max(0.0, float(velocity_bias_rw_mps_sqrt_s))
        self.seed = int(seed)
        self.rng = random.Random(self.seed)
        self.position_bias_x = 0.0
        self.position_bias_y = 0.0
        self.yaw_bias = 0.0
        self.velocity_bias_x = 0.0
        self.velocity_bias_y = 0.0

    def apply(self, state: MotionState, dt: float) -> MotionState:
        delta_t = max(1e-3, min(float(dt), 1.0))
        scale = math.sqrt(delta_t)
        self.position_bias_x += self.rng.gauss(0.0, self.position_bias_rw * scale)
        self.position_bias_y += self.rng.gauss(0.0, self.position_bias_rw * scale)
        self.yaw_bias += self.rng.gauss(0.0, self.yaw_bias_rw * scale)
        self.velocity_bias_x += self.rng.gauss(0.0, self.velocity_bias_rw * scale)
        self.velocity_bias_y += self.rng.gauss(0.0, self.velocity_bias_rw * scale)
        return MotionState(
            x=state.x + self.position_bias_x + self.rng.gauss(0.0, self.position_std),
            y=state.y + self.position_bias_y + self.rng.gauss(0.0, self.position_std),
            yaw=wrap_angle(
                state.yaw + self.yaw_bias + self.rng.gauss(0.0, self.yaw_std)
            ),
            vx=state.vx + self.velocity_bias_x + self.rng.gauss(0.0, self.velocity_std),
            vy=state.vy + self.velocity_bias_y + self.rng.gauss(0.0, self.velocity_std),
        )


class RobustOdometryFilter:
    """중복 median, EMA, jump/speed/yaw 이상치 거부 필터."""

    def __init__(
        self,
        median_window_size: int,
        ema_alpha: float,
        max_position_jump_m: float,
        max_measurement_speed_mps: float,
        max_yaw_jump_rad: float,
    ) -> None:
        window = max(1, int(median_window_size))
        self.x_window: Deque[float] = deque(maxlen=window)
        self.y_window: Deque[float] = deque(maxlen=window)
        self.yaw_window: Deque[float] = deque(maxlen=window)
        self.vx_window: Deque[float] = deque(maxlen=window)
        self.vy_window: Deque[float] = deque(maxlen=window)
        self.ema_alpha = max(0.0, min(1.0, float(ema_alpha)))
        self.max_position_jump_m = max(0.0, float(max_position_jump_m))
        self.max_measurement_speed_mps = max(0.0, float(max_measurement_speed_mps))
        self.max_yaw_jump_rad = max(0.0, float(max_yaw_jump_rad))
        self.last_accepted: Optional[MotionState] = None
        self.last_filtered: Optional[MotionState] = None

    def update(self, measurement: MotionState, dt: float) -> Optional[MotionState]:
        if self.last_accepted is not None:
            position_jump = math.hypot(
                measurement.x - self.last_accepted.x,
                measurement.y - self.last_accepted.y,
            )
            measurement_speed = position_jump / max(float(dt), 1e-3)
            yaw_jump = abs(wrap_angle(measurement.yaw - self.last_accepted.yaw))
            if (
                position_jump > self.max_position_jump_m
                or measurement_speed > self.max_measurement_speed_mps
                or yaw_jump > self.max_yaw_jump_rad
            ):
                return None

        self.last_accepted = measurement
        self.x_window.append(measurement.x)
        self.y_window.append(measurement.y)
        self.yaw_window.append(measurement.yaw)
        self.vx_window.append(measurement.vx)
        self.vy_window.append(measurement.vy)

        median_state = MotionState(
            x=float(sorted(self.x_window)[len(self.x_window) // 2]),
            y=float(sorted(self.y_window)[len(self.y_window) // 2]),
            yaw=median_angle(
                list(self.yaw_window),
                self.last_filtered.yaw if self.last_filtered else None,
            ),
            vx=float(sorted(self.vx_window)[len(self.vx_window) // 2]),
            vy=float(sorted(self.vy_window)[len(self.vy_window) // 2]),
        )

        if self.last_filtered is None or self.ema_alpha >= 1.0:
            filtered = median_state
        else:
            alpha = self.ema_alpha
            filtered = MotionState(
                x=(1.0 - alpha) * self.last_filtered.x + alpha * median_state.x,
                y=(1.0 - alpha) * self.last_filtered.y + alpha * median_state.y,
                yaw=wrap_angle(
                    self.last_filtered.yaw
                    + alpha * wrap_angle(median_state.yaw - self.last_filtered.yaw)
                ),
                vx=(1.0 - alpha) * self.last_filtered.vx + alpha * median_state.vx,
                vy=(1.0 - alpha) * self.last_filtered.vy + alpha * median_state.vy,
            )
        self.last_filtered = filtered
        return filtered
