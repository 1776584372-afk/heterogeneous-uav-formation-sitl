import math

import pytest

from uav_formation.paper_tracking import CopterReferenceCorrection


def test_zero_offset_is_exact_geometric_ppva():
    adapter=CopterReferenceCorrection()
    reference=(10.,20.,math.pi/2,14.,1/160.,.2)
    position,velocity,accel,yaw_rate=adapter.update(reference,reference[:2],.05)
    assert position==pytest.approx((10.,20.,math.pi/2))
    assert velocity==pytest.approx((0.,14.))
    assert accel==pytest.approx((-14*14/160.,.2))
    assert yaw_rate==pytest.approx(-14/160.)


def test_discontinuous_nmpc_request_produces_bounded_continuous_correction():
    adapter=CopterReferenceCorrection(max_offset=1.,max_rate=.2,max_accel=.1,response=.5)
    previous=None
    for j in range(100):
        t=j*.05
        reference=(14*t,0.,0.,14.,0.,0.)
        target=(reference[0],1. if j<50 else -1.)
        position,velocity,accel,yaw_rate=adapter.update(reference,target,.05)
        assert abs(position[1])<=1.+1e-12
        assert abs(velocity[1])<=.2+1e-12
        assert abs(accel[1])<=.1+1e-12
        assert yaw_rate==0.
        if previous is not None:
            p0,v0=previous
            assert position[1]-p0==pytest.approx(.5*(v0+velocity[1])*.05,abs=1e-10)
        previous=(position[1],velocity[1])
    assert abs(adapter.offset[1])>.01


def test_invalid_time_and_reference_fail_closed():
    adapter=CopterReferenceCorrection()
    with pytest.raises(ValueError):
        adapter.update((0.,0.,0.,14.,0.,0.),(0.,0.),0.)
    with pytest.raises(ValueError):
        adapter.update((0.,0.,0.,14.,0.,0.),(math.nan,0.),.05)
    with pytest.raises(ValueError, match='stopping distance'):
        CopterReferenceCorrection(max_offset=.1).update((0.,0.,0.,14.,0.,0.),(2.,0.),.05)


def test_long_saturated_request_stays_inside_offset_envelope():
    adapter=CopterReferenceCorrection()
    for j in range(4000):
        x=14*j*.05
        adapter.update((x,0.,0.,14.,0.,0.),(x,100.),.05)
    assert math.hypot(*adapter.offset)<adapter.max_offset
