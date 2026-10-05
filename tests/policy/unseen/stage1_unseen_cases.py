"""Stage 1 UNSEEN cases (Card 2.1): only for the final MPC vs RL comparison (milestone M1).

Do not open, run or tune against these while designing; a design tuned on them can no longer be fairly evaluated.
pytest skips them unless run with --run-unseen. Pass thresholds are the same as for the tuning cases.

Signs: left of the lane centre and pointing left are positive. Cone index 0 is the pair on the start line.
"""
from stage1_scenarios import Disturbances, Stage1Case
from policy.input_output import ConeColour

UNSEEN_CASES = (
    Stage1Case("U1", "unseen", "0.2 m right, 5 degrees off to the right, slower, noise and delay", -0.2, -5.0, 0.7,
               Disturbances(noise_m=0.03, delay_s=0.1), seed=11),
    Stage1Case("U2", "unseen", "0.1 m left, 15 degrees off to the left, two missed cones and a wrong colour",
               0.1, 15.0, 1.0,
               Disturbances(missed_cones=((ConeColour.BLUE, 4), (ConeColour.BLUE, 5)),
                            wrong_colour_cones=((ConeColour.YELLOW, 8),)), seed=12),
    Stage1Case("U3", "unseen", "0.1 m right, 0.8 m/s, noise, delay and a missed cone near the end", -0.1, 0.0, 0.8,
               Disturbances(noise_m=0.02, delay_s=0.2, missed_cones=((ConeColour.YELLOW, 11),)), seed=13),
)
