"""Stage 1 tuning cases (Card 2.1): use these, and only these, to design and tune MPC and RL.

The unseen cases for the final comparison are kept separately in unseen/stage1_unseen_cases.py; do not open or run
them while tuning. The cases and their pass thresholds are explained in docs/stage1-test-cases.md.

Signs: left of the lane centre and pointing left are positive. Cone index 0 is the pair on the start line.
"""
from stage1_scenarios import Disturbances, Lane, Stage1Case
from policy.input_output import ConeColour

NARROW_LANE = Lane(length_m=6.0, width_m=0.8, cone_spacing_m=0.5)
"""A narrower lane: 0.275 m clearance each side of the 0.25 m wide car when centred."""
WIDE_SPARSE_LANE = Lane(length_m=6.0, width_m=1.2, cone_spacing_m=0.75)
"""A wider lane with fewer cones, so fewer detections per row."""
LONG_LANE = Lane(length_m=8.0, width_m=1.0, cone_spacing_m=0.5)
"""A longer lane, which needs 10.5 m of floor and 17 cones per colour."""

TUNING_CASES = (
    # The card's starting cases, on the main 6 m x 1.0 m lane
    Stage1Case("T1", "tuning", "Centred, straight", 0.0, 0.0, 1.0),
    Stage1Case("T2", "tuning", "0.15 m to the left", 0.15, 0.0, 1.0),
    Stage1Case("T3", "tuning", "10 degrees off to the left", 0.0, 10.0, 1.0),
    Stage1Case("T4", "tuning", "Centred, detector noise and one missed cone", 0.0, 0.0, 1.0,
               Disturbances(noise_m=0.03, missed_cones=((ConeColour.YELLOW, 6),)), seed=1),
    Stage1Case("T5", "tuning", "0.15 m right, 10 degrees off to the right", -0.15, -10.0, 1.0),
    Stage1Case("T6", "tuning", "Centred, 0.2 s detection delay", 0.0, 0.0, 1.0, Disturbances(delay_s=0.2)),
    # Roads of different sizes
    Stage1Case("T7", "tuning", "Narrow 0.8 m lane, 0.1 m to the left", 0.1, 0.0, 1.0, lane=NARROW_LANE),
    Stage1Case("T8", "tuning", "Wide 1.2 m lane, cones every 0.75 m, 0.15 m to the right", -0.15, 0.0, 1.0,
               lane=WIDE_SPARSE_LANE),
    Stage1Case("T9", "tuning", "Long 8 m lane, 5 degrees off to the left", 0.0, 5.0, 1.0, lane=LONG_LANE),
    # Other Card 2.4 disturbances
    Stage1Case("T10", "tuning", "Extra object beside the lane, glare limits the camera to 3 m", 0.0, 0.0, 1.0,
               Disturbances(extra_objects=((3.0, 0.85, ConeColour.YELLOW),), detector_range_m=3.0)),
    Stage1Case("T11", "tuning", "Wet floor (dynamic car), 20% heavier car, 0.1 m to the left", 0.1, 0.0, 1.0,
               Disturbances(floor="wet", car_mass_kg=3.6)),
)
