"""Raw encoder counts <-> URDF joint angles.

  angle = direction * (counts - zero_offset_counts) * 2*pi / counts_per_rev      (rad, URDF convention)

absolute (AMT212F-V): counts are single-turn 0 .. 2^bits - 1; the difference is wrapped to half a turn,
                      which is safe because shoulder and elbow move less than 180 deg.
incremental (OE-28):  counts are the STM32's signed accumulated count since power-up; zero comes from
                      the home reference; the base is continuous so the angle is wrapped to (-pi, pi].
"""
from dataclasses import dataclass
import math
from typing import Dict, List, Optional

from ._config import load_yaml

TWO_PI = 2.0 * math.pi


def _is_set(v) -> bool:
    return v is not None and not (isinstance(v, str) and v.strip().upper() == 'TBD')


def wrap_angle(a: float) -> float:
    a = math.fmod(a + math.pi, TWO_PI)
    if a <= 0.0:
        a += TWO_PI
    return a - math.pi


@dataclass
class EncoderModel:
    joint: str
    kind: str                         # 'absolute' or 'incremental'
    counts_per_rev: Optional[float]   # None while hardware details are TBD
    direction: Optional[int]          # +1 / -1, None until calibrated
    zero_offset_counts: Optional[float]
    calibrated: bool = False

    # ---------------- status ----------------
    def missing(self) -> List[str]:
        out = []
        if self.counts_per_rev is None:
            out.append('counts_per_rev (resolution_bits / pulses_per_rev TBD)')
        if self.direction is None:
            out.append('direction (run calibration)')
        if self.zero_offset_counts is None:
            out.append('zero_offset_counts (run calibration)')
        return out

    @property
    def ready(self) -> bool:
        return not self.missing()

    def _require(self, need_calibration=True):
        miss = self.missing() if need_calibration else (
            ['counts_per_rev'] if self.counts_per_rev is None else [])
        if miss:
            raise RuntimeError(f'{self.joint}: encoder not ready - missing: {", ".join(miss)}')

    # ---------------- conversions ----------------
    def delta_counts(self, counts: float, reference: float) -> float:
        """counts - reference, wrapped to half a turn for absolute encoders."""
        d = counts - reference
        if self.kind == 'absolute':
            cpr = self.counts_per_rev
            d = math.fmod(d + cpr / 2.0, cpr)
            if d < 0:
                d += cpr
            d -= cpr / 2.0
        return d

    def counts_to_angle(self, counts: float) -> float:
        """URDF joint angle (rad) from raw counts."""
        self._require()
        a = self.direction * self.delta_counts(counts, self.zero_offset_counts) * TWO_PI / self.counts_per_rev
        return wrap_angle(a)

    def angle_to_counts(self, angle: float) -> float:
        """Raw count the encoder should show at a URDF joint angle (rad). Absolute: 0 .. cpr-1."""
        self._require()
        c = self.zero_offset_counts + self.direction * angle * self.counts_per_rev / TWO_PI
        return c % self.counts_per_rev if self.kind == 'absolute' else c

    @property
    def resolution_deg(self) -> Optional[float]:
        return None if self.counts_per_rev is None else 360.0 / self.counts_per_rev


def model_from_entry(joint: str, e: dict) -> EncoderModel:
    kind = e.get('type', 'absolute')
    cpr = None
    if kind == 'absolute':
        if _is_set(e.get('resolution_bits')):
            cpr = float(2 ** int(e['resolution_bits']))
    elif kind == 'incremental':
        ppr, quad, gear = e.get('pulses_per_rev'), e.get('quadrature_multiplier', 4), e.get('gear_ratio', 1)
        if _is_set(ppr) and _is_set(quad) and _is_set(gear):
            cpr = float(ppr) * float(quad) * float(gear)
    else:
        raise ValueError(f'{joint}: unknown encoder type {kind!r}')
    d = e.get('direction')
    z = e.get('zero_offset_counts')
    return EncoderModel(joint=joint, kind=kind, counts_per_rev=cpr,
                        direction=int(d) if _is_set(d) else None,
                        zero_offset_counts=float(z) if _is_set(z) else None,
                        calibrated=bool(e.get('calibrated', False)))


def load_encoders(cfg: dict = None) -> Dict[str, EncoderModel]:
    cfg = cfg or load_yaml('encoders.yaml')
    return {j: model_from_entry(j, e) for j, e in cfg['encoders'].items()}


def counts_to_joint_angles(counts: Dict[str, float], encoders: Dict[str, EncoderModel] = None) -> Dict[str, float]:
    encoders = encoders or load_encoders()
    return {j: encoders[j].counts_to_angle(c) for j, c in counts.items()}


def status_report(encoders: Dict[str, EncoderModel] = None) -> str:
    encoders = encoders or load_encoders()
    lines = []
    for j, m in encoders.items():
        res = f'{m.resolution_deg:.4f} deg/count' if m.resolution_deg else 'resolution TBD'
        state = 'READY' if m.ready else 'NOT READY: ' + '; '.join(m.missing())
        lines.append(f'{j:15s} {m.kind:11s} {res:22s} {state}')
    return '\n'.join(lines)
