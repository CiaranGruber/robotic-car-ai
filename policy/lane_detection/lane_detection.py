from policy.input_output import CarObservations
from policy.lane_detection.lanes import LaneDetection


def detect_lanes(observations: CarObservations) -> LaneDetection:
    """Detects the lanes from a set of observations.

    This function should receive the observations, which include cone data, and return the detected lanes holding a
    DreamGym road

    :param observations: The observations taken by the car at the current policy step
    :return: The detected lanes, holding the road configuration as a DreamGym road
    """
    # Output is Dream gym roads
    pass
