"""Bearing degradation over a ~200 h horizon.

Injected into latent state: wear raises vibration RMS and joint torque the way
real wear does, rather than being drawn onto the telemetry. This is the fault
the repo's thesis rests on, because it is the one where detectability lags
injection by a long and interesting interval.
"""

from __future__ import annotations


def wear_fraction(hours_since_onset: float, total_hours: float) -> float:
    """Fraction of end-of-life bearing wear at a given time since onset.

    Returns 0.0 at onset, 1.0 at `total_hours`. Monotonic non-decreasing.

    Drives `vibration_rms_g` and `joint_torque_nm` in the physics-lite backend,
    which means this curve sets what recall and latency even mean:

      - Linear: measurable signal from roughly hour 5, recall looks strong, and
        the harness never exercises the case it was built for.
      - Late knee (flat, then steep): nothing observable above noise for the
        first ~150 h, then rapid progression. Total latency looks awful,
        algorithm latency is small, and the decomposition is what carries the
        result. This is the honest hard case.

    TODO(subha): implement. 5-10 lines.
    """
    raise NotImplementedError
