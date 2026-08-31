from dexfull.bridge.controller import METHOD_ALIASES


def test_existing_unity_method_aliases_are_preserved():
    assert METHOD_ALIASES["startXRTeleop"] == "start_xr_teleop"
    assert METHOD_ALIASES["stopXRTeleop"] == "stop_xr_teleop"
    assert METHOD_ALIASES["startRobot"] == "start_teleop"
    assert METHOD_ALIASES["stopRobot"] == "stop_teleop"

