import io
import json
import threading

from uav_formation.sitl_vehicle import SitlVehicle


class FlushCountingStream(io.StringIO):
    def __init__(self):
        super().__init__()
        self.flushes = 0

    def flush(self):
        self.flushes += 1
        super().flush()


def test_phase_transition_is_buffered_without_synchronous_flush():
    vehicle = SitlVehicle.__new__(SitlVehicle)
    vehicle.phase = 'path_entry'
    vehicle.events = FlushCountingStream()
    vehicle.messages = {}
    vehicle.start_wall = 0.0

    vehicle.set_phase('turn_coupled_settle')

    assert vehicle.events.flushes == 0
    event = json.loads(vehicle.events.getvalue())
    assert event['previous'] == 'path_entry'
    assert event['phase'] == 'turn_coupled_settle'
    assert event['boot_s'] is None


def test_phase_transition_is_safe_before_event_stream_exists():
    vehicle = SitlVehicle.__new__(SitlVehicle)
    vehicle.phase = 'connect'
    vehicle.events = None

    vehicle.set_phase('exit')

    assert vehicle.phase == 'exit'


def test_copter_reference_serializes_heartbeat_and_setpoint_writes():
    vehicle = SitlVehicle.__new__(SitlVehicle)
    vehicle.command_lock = threading.RLock()
    vehicle.target = {}
    vehicle.home_offset = (0., 0.)
    vehicle.altitude = 60.
    calls = []
    vehicle.link = type('Link', (), {'mav': type('Mav', (), {
        'set_position_target_local_ned_send': lambda self,*args: calls.append(args)})()})()

    vehicle.copter_reference(1., 2., 3., 4., accel=(.1, .2), yaw_rate=.3)

    assert len(calls) == 1
    assert vehicle.target['sent_yaw_rate'] == .3
