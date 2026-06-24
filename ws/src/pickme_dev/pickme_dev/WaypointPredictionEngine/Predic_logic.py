from collections import deque
import statistics
  
class ConveyorSpeedEstimator:
    """
    Förderband-Geschwindigkeitsschätzung mit:
    - Medianfilter
    - EMA-Glättung also know as PT1-Filter or Low-Pass-Filter as PT-1
    """

    def __init__(self, smoothing: float = 0.3, median_window: int = 32, drop_threshold: float = 0.8, required_drop_frames: int = 8):
        self._last_x: float | None = None
        self._last_ts_ms: int | None = None
        self._estimated_speed: float = 0.0
        self._alpha = smoothing
        self._speed_buffer = deque(maxlen=median_window)
        self._drop_counter = 0
        self._drop_threshold = drop_threshold
        self._required_drop_frames = required_drop_frames

    def update(self, x_m: float, ts_ms: float) -> float | None:

        if self._last_x is None or self._last_ts_ms is None:
            self._last_x = x_m
            self._last_ts_ms = ts_ms
            return None

        delta_x_m = x_m - self._last_x
        delta_t_s = (ts_ms - self._last_ts_ms) 

        self._last_x = x_m
        self._last_ts_ms = ts_ms

        if delta_t_s <= 0:
            return None

        raw_speed = delta_x_m / delta_t_s

        if raw_speed >= 0:
            return self._estimated_speed

        self._speed_buffer.append(raw_speed)

        if len(self._speed_buffer) < self._speed_buffer.maxlen:
            return None

        filtered_speed = statistics.median(self._speed_buffer)
        self._estimated_speed = filtered_speed
       
        return self._estimated_speed
    

    def reset(self):
        self._last_x = None
        self._last_ts_ms = None
        self._estimated_speed = 0.0
        self._speed_buffer.clear()
        self._drop_counter = 0