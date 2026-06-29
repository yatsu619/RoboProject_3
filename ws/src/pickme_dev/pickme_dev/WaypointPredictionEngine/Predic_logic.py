from collections import deque
import statistics


class BeltVelocityTracker:
    """
    Conveyor belt velocity estimation using:
    - Median filter for outlier suppression
    """

    def __init__(
        self,
        
        window_size: int = 32,
        
    ):
        self._prev_position: float | None = None
        self._prev_time_ms: float | None = None
        self._velocity: float = 0.0
        
        self._window = deque(maxlen=window_size)
        self._stop_count = 0
        

    def feed(self, position_m: float, timestamp_ms: float) -> float | None:
        if self._prev_position is None or self._prev_time_ms is None:
            self._prev_position = position_m
            self._prev_time_ms = timestamp_ms
            return None

        dx = position_m - self._prev_position
        dt = timestamp_ms - self._prev_time_ms

        self._prev_position = position_m
        self._prev_time_ms = timestamp_ms

        if dt <= 0:
            return None

        instant_velocity = dx / dt

       
        if instant_velocity >= 0:
            return self._velocity

        self._window.append(instant_velocity)

        if len(self._window) < self._window.maxlen:
            return None

        self._velocity = statistics.median(self._window)
        return self._velocity

    def clear(self):
        self._prev_position = None
        self._prev_time_ms = None
        self._velocity = 0.0
        self._window.clear()
        self._stop_count = 0