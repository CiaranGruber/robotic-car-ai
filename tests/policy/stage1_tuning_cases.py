"""Stage 1 tuning cases (Card 2.1): use these, and only these, to design and tune MPC and RL.

The unseen cases for the final comparison are kept separately in unseen/stage1_unseen_cases.py; do not open or run
them while tuning. The cases and their pass thresholds are explained in docs/stage1-test-cases.md.

Signs: left of the lane centre and pointing left are positive. Cone index 0 is the pair on the start line.
"""
from stage1_scenarios import Disturbances, Stage1Case
from policy.input_output import ConeColour

TUNING_CASES = (
    Stage1Case("T1", "tuning", "Centred, straight", 0.0, 0.0, 1.0),
    Stage1Case("T2", "tuning", "0.15 m to the left", 0.15, 0.0, 1.0),
    Stage1Case("T3", "tuning", "10 degrees off to the left", 0.0, 10.0, 1.0),
    Stage1Case("T4", "tuning", "Centred, detector noise and one missed cone", 0.0, 0.0, 1.0,
               Disturbances(noise_m=0.03, missed_cones=((ConeColour.YELLOW, 6),)), seed=1),
    Stage1Case("T5", "tuning", "0.15 m right, 10 degrees off to the right", -0.15, -10.0, 1.0),
    Stage1Case("T6", "tuning", "Centred, 0.2 s detection delay", 0.0, 0.0, 1.0, Disturbances(delay_s=0.2)),
)
