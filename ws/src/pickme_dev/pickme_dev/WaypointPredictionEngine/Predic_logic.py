from collections import deque
import statistics


class BeltVelocityTracker:
    """
    Estimates the velocity of a conveyor belt from consecutive position samples.

    The tracker computes the instantaneous velocity between two measurements and
    applies a median filter over a fixed-size sliding window to suppress noise
    and measurement outliers.

    Only negative velocities are considered valid, assuming the conveyor moves
    exclusively in the negative direction.
    """

    def __init__(self,window_size: int = 32, ):
        """
        Create a new velocity tracker.
        Args: window_size: Number of velocity samples used for the median filter.
        """

        self._prev_position: float | None = None
        self._prev_time_ms: float | None = None
        self._velocity: float = 0.0
        
        self._window = deque(maxlen=window_size)
        self._stop_count = 0
        

    def feed(self, position_m: float, timestamp_ms: float) -> float | None:
        """
        Process a new position measurement.

        The instantaneous velocity is calculated from the current and previous
        position sample. Once the sliding window is filled, the median of the
        collected velocities is returned as the estimated conveyor velocity.
        Args:
            position_m: Current conveyor position in meters.
            timestamp_ms: Timestamp of the measurement.
        Returns:
            The filtered conveyor velocity if enough samples are available,
            otherwise ``None``.
        """

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

        # Ignore measurements that violate the expected belt direction.
        if instant_velocity >= 0:
            return self._velocity

        self._window.append(instant_velocity)

        if len(self._window) < self._window.maxlen:
            return None

        self._velocity = statistics.median(self._window)
        return self._velocity

    def clear(self):
        """Reset the tracker and discard all previously collected samples."""
        self._prev_position = None
        self._prev_time_ms = None
        self._velocity = 0.0
        self._window.clear()
        self._stop_count = 0