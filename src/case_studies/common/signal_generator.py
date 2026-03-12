# 3rd-party
import numpy as np


# TODO: decide if SignalGererator should handle arrays
#   - for params (e.g., array of amplitudes)
#   - should output always be an array
class SignalGenerator:
    def __init__(self, amplitude=1.0, frequency=0.001, y_offset=0.0):
        self.amplitude = amplitude
        self.frequency = frequency
        self.y_offset = y_offset
        self.rng = np.random.default_rng()

    def square(self, t):
        if isinstance(t, float):
            if t % (1.0 / self.frequency) <= 0.5 / self.frequency:
                out = self.amplitude + self.y_offset
            else:
                out = -self.amplitude + self.y_offset
        else:
            out = np.empty(np.size(t))
            mask = t % (1.0 / self.frequency) <= 0.5 / self.frequency
            out[mask] = self.amplitude + self.y_offset
            out[~mask] = -self.amplitude + self.y_offset
        return out

    def sawtooth(self, t):
        tmp = t % (0.5 / self.frequency)
        out = 4 * self.amplitude * self.frequency * tmp - self.amplitude + self.y_offset
        return out

    def step(self, t):
        if isinstance(t, float):
            if t >= 0.0:
                out = self.amplitude + self.y_offset
            else:
                out = self.y_offset
        else:
            out = np.empty(np.size(t))
            mask = t > 0.0
            out[mask] = self.amplitude + self.y_offset
            out[~mask] = self.y_offset
        return out

    def random(self, t):
        if isinstance(t, float):
            out = self.rng.normal(self.y_offset, self.amplitude)
        else:
            out = self.rng.normal(self.y_offset, self.amplitude, size=np.size(t))
        return out

    def sin(self, t):
        out = self.amplitude * np.sin(2 * np.pi * self.frequency * t) + self.y_offset
        return out
