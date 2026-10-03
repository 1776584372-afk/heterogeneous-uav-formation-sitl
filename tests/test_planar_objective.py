import unittest
import numpy as np
from scipy.optimize import minimize
from uav_formation.planar_objective import cost_gradient, expand_held_controls, held_cost_gradient


class ExactGradientTests(unittest.TestCase):
    def setUp(self):
        self.n=12
        rng=np.random.default_rng(42)
        self.controls=np.r_[rng.uniform(10,15,12),rng.uniform(-.4,.4,12)]
        self.state=np.array([2.,3.,3.1])
        self.reference=np.vstack([2.-14.*np.arange(13)*.05,np.full(13,3.),np.full(13,-3.12)])
        self.speeds=np.linspace(13,14,12)
    def objective(self,u):
        return cost_gradient(u,self.state,self.reference,self.speeds,.05,(10.,7.,2.,.02))
    def test_gradient_matches_central_difference_across_wrapped_yaw(self):
        _,gradient=self.objective(self.controls)
        epsilon=1e-5
        numeric=[]
        for j in range(24):
            delta=np.zeros(24);delta[j]=epsilon
            numeric.append((self.objective(self.controls+delta)[0]-self.objective(self.controls-delta)[0])/(2*epsilon))
        np.testing.assert_allclose(gradient,numeric,rtol=1e-6,atol=1e-6)
    def test_cost_matches_original_scalar_model_and_weights(self):
        x=self.state.copy();value=0.
        for k in range(13):
            d=x-self.reference[:,k];d[2]=(d[2]+np.pi)%(2*np.pi)-np.pi
            value+=10.*sum(d[:2]**2)+7.*d[2]**2
            if k<12:
                v,w=self.controls[k],self.controls[12+k]
                value+=.02*(v*v+w*w)+2.*(v-self.speeds[k])**2
                x+=.05*np.array([v*np.cos(x[2]),v*np.sin(x[2]),w])
        self.assertAlmostEqual(self.objective(self.controls)[0],value,places=10)
    def test_same_bounded_optimum_as_numerical_gradient(self):
        bounds=[(0.,20.)]*12+[(-1.2,1.2)]*12
        exact=minimize(self.objective,self.controls,jac=True,method='SLSQP',bounds=bounds,options={'ftol':1e-7,'maxiter':120})
        numeric=minimize(lambda u:self.objective(u)[0],self.controls,method='SLSQP',bounds=bounds,options={'ftol':1e-7,'maxiter':120})
        self.assertTrue(exact.success and numeric.success)
        np.testing.assert_allclose(exact.x,numeric.x,atol=1e-4)
        self.assertLess(exact.nfev,numeric.nfev)

    def test_held_block_has_exact_reduced_gradient_and_constant_first_seven_steps(self):
        u=np.r_[np.linspace(12.,14.,6),np.linspace(-.1,.2,6)]
        expanded=expand_held_controls(u,12,7)
        np.testing.assert_array_equal(expanded[:7],np.full(7,u[0]))
        np.testing.assert_array_equal(expanded[12:19],np.full(7,u[6]))
        fn=lambda x:held_cost_gradient(x,self.state,self.reference,self.speeds,.05,(10.,7.,2.,.02),7)
        value,grad=fn(u)
        self.assertAlmostEqual(value,self.objective(expanded)[0])
        numerical=[]
        for k in range(len(u)):
            delta=np.zeros_like(u);delta[k]=1e-5
            numerical.append((fn(u+delta)[0]-fn(u-delta)[0])/2e-5)
        np.testing.assert_allclose(grad,numerical,rtol=1e-6,atol=1e-6)


if __name__=='__main__':unittest.main()
