"""Wall-clock pacing and optional stage timings for the single-step simulator."""
import math
import time


class RealtimePacer:
    """Keep a wall deadline without skipping physics or repaying long pauses."""

    def __init__(self, timestep, max_lag=0.1, *, clock=time.perf_counter, sleep=time.sleep):
        if not math.isfinite(timestep) or timestep <= 0:
            raise ValueError('timestep must be finite and positive')
        if not math.isfinite(max_lag) or max_lag < 0:
            raise ValueError('max_lag must be finite and nonnegative')
        self.timestep = timestep
        self.max_lag = max_lag
        self.clock = clock
        self.sleep = sleep
        self.deadline = clock()

    def wait(self):
        self.deadline += self.timestep
        now = self.clock()
        delay = self.deadline - now
        if delay > 0:
            self.sleep(delay)
        elif -delay > self.max_lag:
            # Renderer initialization/debugger pauses must not create minutes
            # of catch-up. Only the wall deadline changes, never simulation time.
            self.deadline = now - self.max_lag
