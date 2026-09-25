"""Smoke tests for ethercat_sdk Python bindings (no hardware required)."""

import ethercat_sdk


def test_module_has_expected_types():
    for name in [
        "EtherCATMaster",
        "OperationMode",
        "DriveState",
        "ActuatorState",
        "ActuatorType",
        "ActuatorConfig",
        "BusConfig",
        "CycleTimingSnapshot",
        "InvalidActuatorTypeError",
    ]:
        assert hasattr(ethercat_sdk, name)


def test_operation_mode_values():
    assert ethercat_sdk.OperationMode.CSP.value == 8
    assert ethercat_sdk.OperationMode.CSV.value == 9
    assert ethercat_sdk.OperationMode.CST.value == 10
    assert ethercat_sdk.OperationMode.PVT.value == 5


def test_drive_state_names():
    assert ethercat_sdk.DriveState.Fault.name == "Fault"
    assert ethercat_sdk.DriveState.OperationEnabled.name == "OperationEnabled"


def test_actuator_state_defaults():
    state = ethercat_sdk.ActuatorState()
    assert state.position == 0.0
    assert state.velocity == 0.0
    assert state.torque == 0.0
    assert state.enabled is False
    assert state.fault is False
    assert state.lost is False


def test_actuator_state_repr():
    state = ethercat_sdk.ActuatorState()
    r = repr(state)
    assert "ActuatorState" in r
    assert "pos=" in r


def test_cycle_timing_snapshot_defaults():
    snap = ethercat_sdk.CycleTimingSnapshot()
    assert snap.cycle_count == 0
    assert snap.wkc == 0


def test_master_construction():
    master = ethercat_sdk.EtherCATMaster("nonexistent_iface")
    assert master.is_running() is False
    assert master.get_slave_count() == 0


def test_master_operation_mode():
    master = ethercat_sdk.EtherCATMaster("nonexistent_iface")
    master.set_operation_mode(ethercat_sdk.OperationMode.PVT)
    assert master.get_operation_mode() == ethercat_sdk.OperationMode.PVT


def test_exception_is_runtime_error():
    assert issubclass(ethercat_sdk.InvalidActuatorTypeError, RuntimeError)
