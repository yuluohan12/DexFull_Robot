from dexfull.control.robots import get_robot_adapter, list_robot_types
from dexfull.hand_drivers import get_hand_plugin, list_hand_types


def test_builtin_robot_adapters_are_registered():
    assert set(list_robot_types()) >= {"G1_29", "G1_23", "H1_2", "H1"}
    assert len(get_robot_adapter("G1_29").joint_names) == 29
    assert len(get_robot_adapter("G1_23").joint_names) == 23


def test_brainco_is_an_external_expandable_hand_plugin():
    assert set(list_hand_types()) >= {"brainco", "dex1", "dex3", "inspire_dfx", "inspire_ftp"}
    plugin = get_hand_plugin("brainco")
    assert plugin.isolation == "external_process"
    assert plugin.service_name == "brainco"
    assert plugin.collector_factory is not None

