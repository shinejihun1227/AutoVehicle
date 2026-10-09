"""속도 오차를 MORAI accel/brake 명령으로 변환하는 순수 Python 제어기."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


MPS_TO_KPH = 3.6


class VehicleSpeedSource:
    """Fresh vehicle speed, independent of the position estimator.

    The UDP bridge publishes EgoVehicleStatus.velocity in m/s. Both source
    time and receipt time are checked: a paused clock, repeated message, or
    expired UDP stream must not look like a stationary vehicle indefinitely.
    A sample is an immutable tuple so callbacks cannot expose partial state.
    """

    def __init__(self, timeout_sec: float = 0.5, max_speed_mps: float = 100.0):
        if not all(math.isfinite(v) and v > 0.0 for v in
                   (timeout_sec, max_speed_mps)):
            raise ValueError("Vehicle speed limits must be finite and positive")
        self.timeout_sec = float(timeout_sec)
        self.max_speed_mps = float(max_speed_mps)
        self._sample = None

    def observe(self, speed_mps: float, stamp: float, ros_now: float,
                wall_now: float) -> bool:
        values = (speed_mps, stamp, ros_now, wall_now)
        if (not all(math.isfinite(v) for v in values)
                or not 0.0 <= speed_mps <= self.max_speed_mps
                or stamp <= 0.0
                or not -0.05 <= ros_now - stamp <= self.timeout_sec):
            return False
        previous = self._sample
        if previous is not None and ros_now < previous[1] - 0.05:
            # A simulator clock reset starts a new timestamp epoch. Keep the
            # source unavailable until its first fresh sample, then recover;
            # an ordinary out-of-order packet with a current clock cannot
            # take this branch.
            self._sample = None
            previous = None
        if previous is not None and stamp <= previous[1]:
            return False
        self._sample = (float(speed_mps), float(stamp), float(wall_now))
        return True

    def sample(self, ros_now: float, wall_now: float) -> Optional[float]:
        sample = self._sample
        if sample is None or not all(math.isfinite(v) for v in (ros_now, wall_now)):
            return None
        speed, stamp, received = sample
        if (not -0.05 <= ros_now - stamp <= self.timeout_sec
                or not 0.0 <= wall_now - received <= self.timeout_sec):
            return None
        return speed


@dataclass(frozen=True)
class LongitudinalCommand:
    """한 주기 동안 사용할 종방향 명령."""

    requested_acceleration_mps2: float
    accel: float
    brake: float


class SpeedPIController:
    """목표속도-현재속도 오차를 accel/brake 페달값으로 변환한다.

    출력은 항상 accel 또는 brake 중 하나만 사용한다. 적분항은 출력 포화
    방향으로 오차가 계속 누적될 때 동결하여 wind-up을 방지한다.
    """

    def __init__(
        self,
        kp: float = 0.8,
        ki: float = 0.05,
        max_accel_mps2: float = 1.0,
        max_decel_mps2: float = 1.5,
        integral_limit_kph_s: float = 10.8,
        speed_error_deadband_kph: float = 0.1,
        accel_rise_rate_per_sec: float = 0.0,
        brake_rise_rate_per_sec: float = 0.0,
        pedal_release_rate_per_sec: float = 0.0,
    ) -> None:
        if kp < 0.0 or ki < 0.0:
            raise ValueError("PI gain은 음수가 될 수 없다.")
        if max_accel_mps2 <= 0.0 or max_decel_mps2 <= 0.0:
            raise ValueError("max_accel_mps2와 max_decel_mps2는 0보다 커야 한다.")
        if not all(math.isfinite(value) and value >= 0.0 for value in
                   (accel_rise_rate_per_sec, brake_rise_rate_per_sec,
                    pedal_release_rate_per_sec)):
            raise ValueError("Pedal slew rates must be finite and nonnegative")

        self.kp = float(kp)
        self.ki = float(ki)
        self.max_accel_mps2 = float(max_accel_mps2)
        self.max_decel_mps2 = float(max_decel_mps2)
        # 외부 속도 인터페이스는 km/h로 받되, PI의 물리 계산은 m/s와
        # m/s²를 사용한다. 적분항의 내부 단위는 (m/s)·s이다.
        self.integral_limit_mps_s = max(
            0.0, float(integral_limit_kph_s) / MPS_TO_KPH
        )
        self.speed_error_deadband_mps = max(
            0.0, float(speed_error_deadband_kph) / MPS_TO_KPH
        )
        self.integral_error_mps_s = 0.0
        self.accel_rise_rate_per_sec = float(accel_rise_rate_per_sec)
        self.brake_rise_rate_per_sec = float(brake_rise_rate_per_sec)
        self.pedal_release_rate_per_sec = float(pedal_release_rate_per_sec)
        self.last_accel = 0.0
        self.last_brake = 0.0

    def reset(self) -> None:
        self.integral_error_mps_s = 0.0
        self.last_accel = self.last_brake = 0.0

    @staticmethod
    def _slew(previous: float, target: float, rise: float, release: float, dt: float) -> float:
        rate = rise if target > previous else release
        if rate <= 0.0:
            return target
        step = rate * dt
        return max(previous - step, min(previous + step, target))

    @staticmethod
    def _finite_nonnegative(value: float) -> float:
        value = float(value)
        return max(0.0, value) if math.isfinite(value) else 0.0

    def update(
        self,
        target_speed_kph: float,
        measured_speed_kph: float,
        dt: float,
        stop: bool = False,
    ) -> LongitudinalCommand:
        """현재속도와 목표속도로 한 주기의 페달 명령을 계산한다."""

        if stop:
            self.reset()
            return LongitudinalCommand(0.0, 0.0, 1.0)

        target = self._finite_nonnegative(target_speed_kph) / MPS_TO_KPH
        measured = self._finite_nonnegative(measured_speed_kph) / MPS_TO_KPH
        dt = max(0.0, min(float(dt), 1.0)) if math.isfinite(float(dt)) else 0.0
        error = target - measured

        # A positive integral left over from acceleration must not keep the
        # throttle open above a newly lowered curve/route speed limit. The
        # reciprocal case matters after a prolonged signal stop as well.
        if error * self.integral_error_mps_s < 0.0:
            self.integral_error_mps_s = 0.0

        if abs(error) <= self.speed_error_deadband_mps:
            self.integral_error_mps_s *= max(0.0, 1.0 - 2.0 * dt)
            requested = 0.0
        else:
            previous_integral = self.integral_error_mps_s
            candidate_integral = previous_integral + error * dt
            candidate_integral = max(
                -self.integral_limit_mps_s,
                min(self.integral_limit_mps_s, candidate_integral),
            )
            requested = self.kp * error + self.ki * candidate_integral

            saturating_high = requested > self.max_accel_mps2 and error > 0.0
            saturating_low = requested < -self.max_decel_mps2 and error < 0.0
            if saturating_high or saturating_low:
                candidate_integral = previous_integral
                requested = self.kp * error + self.ki * candidate_integral

            self.integral_error_mps_s = candidate_integral

        requested = max(
            -self.max_decel_mps2,
            min(self.max_accel_mps2, requested),
        )

        if requested >= 0.0:
            accel = requested / self.max_accel_mps2
            brake = 0.0
        else:
            accel = 0.0
            brake = -requested / self.max_decel_mps2

        # Brake always releases a prior accelerator immediately; never issue
        # both pedals together. Only ordinary driving is slewed. stop=True
        # above still applies full brake without delay.
        if brake > 0.0:
            self.last_accel = 0.0
            brake = self._slew(self.last_brake, brake,
                               self.brake_rise_rate_per_sec,
                               self.pedal_release_rate_per_sec, dt)
        elif accel > 0.0:
            self.last_brake = 0.0
            accel = self._slew(self.last_accel, accel,
                               self.accel_rise_rate_per_sec,
                               self.pedal_release_rate_per_sec, dt)
        elif self.last_brake > 0.0:
            brake = self._slew(self.last_brake, 0.0, 0.0,
                               self.pedal_release_rate_per_sec, dt)
        else:
            accel = self._slew(self.last_accel, 0.0, 0.0,
                               self.pedal_release_rate_per_sec, dt)
        if measured >= target:
            # Releasing a pedal is immediate at the speed ceiling. Slewing a
            # positive accelerator towards zero here would intentionally
            # continue accelerating after the speed error has disappeared.
            accel = 0.0
        self.last_accel, self.last_brake = accel, brake

        return LongitudinalCommand(requested, accel, brake)
