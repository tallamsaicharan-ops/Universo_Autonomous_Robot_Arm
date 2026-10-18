"""Where raw counts come from. Calibration code only talks to EncoderSource.read_counts().

Stm32Source is the ONLY place to edit once the STM32 message format is known.
"""
import math
import random
from typing import Dict, Optional


class EncoderSource:
    joints = ('base_joint', 'shoulder_joint', 'elbow_joint')

    def read_counts(self) -> Dict[str, float]:
        raise NotImplementedError


class ManualEntrySource(EncoderSource):
    """Type the counts shown by the STM32 (serial monitor / display). Works today, no format needed."""

    def __init__(self, joints=None):
        self.joints = tuple(joints or self.joints)

    def read_counts(self) -> Dict[str, float]:
        out = {}
        for j in self.joints:
            out[j] = float(input(f'  raw count for {j}: ').strip())
        return out


class Stm32Source(EncoderSource):
    """TODO (when the STM32 format is known): open the link and parse one sample of raw counts.

    Fill in: transport (USB serial / CAN), port or interface, message layout, and which field is
    which joint. Must return {'base_joint': int, 'shoulder_joint': int, 'elbow_joint': int}.
    """

    def __init__(self, port: Optional[str] = None, **kwargs):
        self.port = port
        self.kwargs = kwargs

    def read_counts(self) -> Dict[str, float]:
        raise NotImplementedError('STM32 message format not defined yet - use ManualEntrySource '
                                  'or fill in Stm32Source.read_counts()')


class SimulatedSource(EncoderSource):
    """Fake encoders with a hidden true offset/direction - used by tests and --simulate."""

    def __init__(self, truth: Dict[str, dict], seed: int = 0, noise_counts: float = 0.0):
        # truth[joint] = {'counts_per_rev', 'direction', 'zero_offset_counts', 'kind'}
        self.truth = truth
        self.angles = {j: 0.0 for j in truth}
        self.rng = random.Random(seed)
        self.noise = noise_counts

    def set_angles(self, **angles_rad):
        self.angles.update(angles_rad)

    def read_counts(self) -> Dict[str, float]:
        out = {}
        for j, t in self.truth.items():
            c = t['zero_offset_counts'] + t['direction'] * self.angles[j] * t['counts_per_rev'] / (2 * math.pi)
            c += self.rng.gauss(0.0, self.noise) if self.noise else 0.0
            c = round(c)
            out[j] = c % t['counts_per_rev'] if t['kind'] == 'absolute' else c
        return out
