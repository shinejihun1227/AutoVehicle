"""속도 오차를 MORAI accel/brake 명령으로 변환하는 순수 Python 제어기."""

from __future__ import annotations

import math
from dataclasses import dataclass


MPS_TO_KPH = 3.6


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
    ) -> None:
        if kp < 0.0 or ki < 0.0:
            raise ValueError("PI gain은 음수가 될 수 없다.")
        if max_accel_mps2 <= 0.0 or max_decel_mps2 <= 0.0:
            raise ValueError("max_accel_mps2와 max_decel_mps2는 0보다 커야 한다.")

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

    def reset(self) -> None:
        self.integral_error_mps_s = 0.0

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

        if abs(error) <= self.speed_error_deadband_mps:
            return LongitudinalCommand(0.0, 0.0, 0.0)

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
            # 현재 오차가 포화 방향인 동안에는 적분항을 더 쌓지 않는다.
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

        return LongitudinalCommand(requested, accel, brake)
