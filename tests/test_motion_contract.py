import numpy as np
import pytest

from scripts.audit_motion_contract import motion_series, ppva_residual, unique_samples


def test_course_can_turn_while_nose_stays_fixed():
    t = np.arange(31)*.05
    v = 12*np.column_stack([np.cos(.1*t), np.sin(.1*t)])
    result = motion_series(t, v, t, np.zeros(len(t)))
    np.testing.assert_allclose(result['course_rate_rad_s'], .1, atol=1e-12)
    np.testing.assert_allclose(result['yaw_rate_rad_s'], 0.)
    np.testing.assert_allclose(result['horizontal_heading_left_m_s'], v[:, 1])


def test_yaw_crossing_pi_keeps_course_and_drift_separate():
    t = np.arange(10)*.05
    yaw = np.pi-.02+.2*t
    course = yaw+.1
    wrapped = np.angle(np.exp(1j*yaw))
    result = motion_series(t, 12*np.column_stack([np.cos(course), np.sin(course)]), t, wrapped)
    np.testing.assert_allclose(result['yaw_rate_rad_s'], .2, atol=1e-12)
    np.testing.assert_allclose(result['beta_rad'], .1, atol=1e-12)


def test_alignment_uses_each_source_clock_and_never_extrapolates():
    ta = np.arange(20)*.05
    tv = np.arange(19)*.05+.025
    yaw = .2*ta
    v = 12*np.column_stack([np.cos(.2*tv), np.sin(.2*tv)])
    result = motion_series(tv, v, ta, yaw)
    np.testing.assert_allclose(result['beta_rad'], 0., atol=1e-12)
    with pytest.raises(ValueError, match='course undefined'):
        motion_series(tv, v*0., ta, yaw)
    with pytest.raises(ValueError, match='gaps'):
        motion_series(tv*10, v, ta*10, yaw)


def test_ppva_consistency_and_measurement_anchor_reset():
    t = np.arange(10)*.05
    a = np.tile([.1, 0.], (len(t), 1))
    v = np.column_stack([12+.1*t, np.zeros(len(t))])
    p = np.column_stack([12*t+.05*t*t, np.zeros(len(t))])
    good = ppva_residual(t, p, v, a)
    assert max(good['position_defect_m']) < 1e-12
    assert max(good['velocity_defect_m_s']) < 1e-12
    p[5:, 1] += .2
    bad = ppva_residual(t, p, v, a)
    assert bad['position_defect_m'][4] == pytest.approx(.2)
    assert max(bad['velocity_defect_m_s']) < 1e-12


def test_unique_samples_keep_last_and_reject_clock_reversal():
    t, v = unique_samples([0., 0., 1., 2.], [1., 2., 3., 4.])
    np.testing.assert_array_equal(v, [2., 3., 4.])
    with pytest.raises(ValueError):
        unique_samples([0., 2., 1.], [1., 2., 3.])
