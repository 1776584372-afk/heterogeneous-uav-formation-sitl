import math

import pytest
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.summarize_onboard_probe import summarize, yaw_coordinate_decomposition


def test_native_yaw_unwrap_and_degree_units():
    rows = [dict(boot_s=i, DesYaw=yaw, Yaw=yaw, DesRoll=0, Roll=0, DesPitch=0, Pitch=0)
            for i, yaw in enumerate((179., -180., -179.))]
    stats = summarize('ANG', rows)['signals']['estimated_euler_yaw_rate_rad_s']
    assert stats['minimum'] == pytest.approx(math.pi/180)
    assert stats['maximum'] == pytest.approx(math.pi/180)


def test_body_rate_is_separate_from_heading_derivative():
    rows = [dict(boot_s=i, YDes=180., Y=90., YOut=.1) for i in range(3)]
    stats = summarize('RATE', rows)['signals']
    assert stats['YDes_body_yaw_rate_rad_s']['maximum'] == pytest.approx(math.pi)
    assert stats['Y_body_yaw_rate_rad_s']['maximum'] == pytest.approx(math.pi/2)
    assert not any('euler' in key for key in stats)


def test_tecs_native_target_and_response_remain_distinct():
    rows = [dict(boot_s=i, spdem=14.+.02*i, sp=14.+.3*i, dsp=.3, dspdem=.02,
                 h=60., hin=60., hdem=60., th=.2, ph=.1) for i in range(3)]
    stats = summarize('TECS', rows)['signals']
    assert stats['spdem_derivative_m_s2']['maximum'] == pytest.approx(.02)
    assert stats['sp_derivative_m_s2']['maximum'] == pytest.approx(.3)
    with pytest.raises(ValueError):
        summarize('TECS', rows[::-1])


def test_plane_received_target_and_conversion_ratio_keep_native_units():
    rows = [dict(boot_s=i, Arsp=14.+.02*i, Alt=60.) for i in range(3)]
    stats = summarize('OFG', rows)['signals']
    assert stats['received_equivalent_airspeed_m_s']['minimum'] == 14.
    assert stats['received_equivalent_airspeed_derivative_m_s2']['maximum'] == pytest.approx(.02)
    assert 'received_altitude_m' not in stats
    assert stats['guided_slew_location_alt_field_m']['maximum'] == 60.
    ratio = summarize('CTUN', [dict(boot_s=i, E2T=1.003) for i in range(3)])
    assert ratio['signals']['eas_to_tas_ratio']['maximum'] == 1.003


def test_yaw_twist_decomposition_matches_independent_quaternion_algebra():
    angles = np.radians([[12., -31., 179.], [-8., -30., -179.], [15., -32., -177.]])
    twist, offset = yaw_coordinate_decomposition(*angles.T)
    quat = Rotation.from_euler('xyz', angles).as_quat()
    independent = np.unwrap(2*np.arctan2(quat[:, 2], quat[:, 3]))
    np.testing.assert_allclose(np.angle(np.exp(1j*(twist-independent))), 0., atol=1e-12)
    np.testing.assert_allclose(twist+offset, np.unwrap(angles[:, 2]), atol=1e-12)
    twist, offset = yaw_coordinate_decomposition(np.zeros(3), angles[:, 1], angles[:, 2])
    np.testing.assert_allclose(offset, 0.)
    with pytest.raises(ValueError):
        yaw_coordinate_decomposition(np.zeros(3), np.full(3, np.pi/2), np.zeros(3))


def test_tilt_coordinate_rate_does_not_replace_euler_yaw_metric():
    roll = np.linspace(-.2, .2, 5)
    pitch = np.full(5, -.5)
    offset = 2*np.arctan(np.tan(roll/2)*np.tan(pitch/2))
    yaw = .1*np.arange(5)+offset
    rows = [dict(boot_s=i, DesYaw=np.degrees(yaw[i]), Yaw=np.degrees(yaw[i]),
        DesRoll=np.degrees(roll[i]), Roll=np.degrees(roll[i]),
        DesPitch=np.degrees(pitch[i]), Pitch=np.degrees(pitch[i])) for i in range(5)]
    result = summarize('ANG', rows)['signals']
    assert result['desired_z_twist_rate_rad_s']['maximum'] == pytest.approx(.1)
    assert result['desired_euler_yaw_rate_rad_s']['maximum'] < .1
