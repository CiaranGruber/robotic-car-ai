"""RC observations remain compatible with recorded scenario data."""
import dataclasses
import json

from policy.input_output import CarObservations, RcInput, SensorAge
from tests.policy.test_mpc import observe


def test_rc_observations_round_trip_through_json():
    original = dataclasses.replace(observe([], first_step=True),
                                   rc=RcInput(0.5, -0.2, SensorAge(0.1, None)))
    restored = CarObservations.deserialise(json.loads(json.dumps(original.serialise())))
    assert restored.rc == original.rc
    assert restored.rc.sensor_age.stamp_ns is None
    assert restored.policy == original.policy


def test_recordings_before_rc_support_load_with_missing_rc():
    data = observe([], first_step=True).serialise()
    del data["rc"]
    restored = CarObservations.deserialise(data)
    assert restored.rc is None
    assert restored.cones == []
