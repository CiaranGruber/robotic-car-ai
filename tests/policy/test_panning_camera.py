"""Check moving field of view, acquisition-time coordinates, servo limits and unchanged default physics."""
from dataclasses import replace
import math

import numpy as np
import pytest

from policy.control.actions import DriveCommand
from policy.input_output import ConeColour
from tests.policy.car21_uturn_simulation import case, controller
from tests.policy.curved_lanes import run_curve
from tests.policy.panning_camera import PanningCamera


@pytest.mark.parametrize("sign", [-1, 1])
def test_pan_sees_side_cone_and_returns_its_body_position(sign):
    car = replace(case().car, detector_noise_m=0.)
    cones = [(ConeColour.BLUE, 0., 1., sign*math.sqrt(3))]
    fixed = PanningCamera(max_pan_deg=0.)
    assert not fixed.capture(cones, (0, 0, 0), car, np.random.default_rng(0), 0.).cones
    camera = PanningCamera(max_pan_deg=30.)
    camera.angle_deg = sign*30.
    frame = camera.capture(cones, (0, 0, 0), car, np.random.default_rng(0), 0.)
    assert len(frame.cones) == 1
    camera.angle_deg = -20.  # The camera moved while this frame was waiting in the latency queue.
    observed = camera.to_body(frame)[0]
    assert (observed.pos.x, observed.pos.y) == pytest.approx((1., sign*math.sqrt(3)))


def test_uncorrected_pan_produces_the_wrong_lane_heading():
    car = replace(case().car, detector_noise_m=0.)
    camera = PanningCamera(compensate=False)
    camera.angle_deg = 20.
    frame = camera.capture([(ConeColour.BLUE, 0., 1., .5)], (0, 0, 0), car, np.random.default_rng(0), 0.)
    observed = camera.to_body(frame)[0]
    assert math.atan2(observed.pos.y, observed.pos.x) == pytest.approx(math.atan2(.5, 1)-math.radians(20))


def test_pan_slew_angle_limits_straight_recentering_and_stop_hold():
    camera = PanningCamera(max_pan_deg=25., slew_deg_per_s=60.)
    for _ in range(100):
        before = camera.angle_deg
        camera.advance(DriveCommand(.4, 1.), .01)
        assert 0 <= camera.angle_deg-before <= .6+1e-12
        assert camera.angle_deg <= 25.
    assert camera.angle_deg > 24.
    held = camera.target_deg
    before = camera.angle_deg
    camera.advance(DriveCommand(0., 0.), 1.)
    assert camera.target_deg == held
    assert before <= camera.angle_deg <= held
    before = camera.angle_deg
    camera.advance(DriveCommand(.4, 0.), .1)
    assert camera.angle_deg < before and camera.target_deg == 0.


def test_zero_pan_retains_original_trajectory_and_scoring():
    c = replace(case(seed=701), max_duration_s=3.)
    baseline = run_curve(c, controller())
    with_camera = run_curve(c, controller(), camera=PanningCamera(max_pan_deg=0.))
    np.testing.assert_array_equal(baseline.x, with_camera.x)
    np.testing.assert_array_equal(baseline.y, with_camera.y)
    assert baseline.metrics == with_camera.metrics


def test_camera_previews_observed_bend_before_steering_and_forgets_a_missing_path():
    camera = PanningCamera(path_lookahead_m=2.)
    camera.aim_path(((.4, 0.), (1., 0.), (1., 2.)))
    camera.advance(DriveCommand(.4, 0.), .1)
    assert camera.target_deg > 0 and camera.angle_deg > 0
    camera.aim_path(())
    camera.advance(DriveCommand(.4, 0.), .1)
    assert camera.target_deg == 0
