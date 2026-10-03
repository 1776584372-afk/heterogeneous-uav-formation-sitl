"""Exact discrete adjoint for the existing Euler unicycle NMPC objective."""
import numpy as np


def expand_held_controls(reduced, n, block):
    """Repeat the first decision for the externally held control interval."""
    if not 1<=block<=n or np.shape(reduced)!=(2*(n-block+1),):
        raise ValueError('invalid blocked input dimensions')
    half=n-block+1
    return np.r_[np.repeat(reduced[0],block),reduced[1:half],
                 np.repeat(reduced[half],block),reduced[half+1:]]


def held_cost_gradient(reduced, state, reference, speeds, dt, weights, block):
    n=len(speeds)
    value,gradient=cost_gradient(expand_held_controls(reduced,n,block),state,reference,speeds,dt,weights)
    return value,np.r_[sum(gradient[:block]),gradient[block:n],
                       sum(gradient[n:n+block]),gradient[n+block:]]


def cost_gradient(controls, state, reference, speeds, dt, weights):
    position_weight, heading_weight, speed_weight, input_weight=weights
    n=len(speeds)
    velocity,rate=np.asarray(controls[:n]),np.asarray(controls[n:])
    x=np.empty((3,n+1)); x[:,0]=state
    for k in range(n):
        x[:,k+1]=x[:,k]+dt*np.array([velocity[k]*np.cos(x[2,k]),
                                    velocity[k]*np.sin(x[2,k]),rate[k]])
    error=x-reference
    error[2]=(error[2]+np.pi)%(2*np.pi)-np.pi
    value=(position_weight*np.sum(error[:2]**2)+heading_weight*np.sum(error[2]**2)
           +speed_weight*np.sum((velocity-speeds)**2)
           +input_weight*np.sum(velocity**2+rate**2))
    state_gradient=2*np.array([position_weight,position_weight,heading_weight])[:,None]*error
    gradient=np.empty(2*n)
    adjoint=state_gradient[:,n].copy()
    for k in range(n-1,-1,-1):
        c,s=np.cos(x[2,k]),np.sin(x[2,k])
        gradient[k]=2*input_weight*velocity[k]+2*speed_weight*(velocity[k]-speeds[k])+dt*(c*adjoint[0]+s*adjoint[1])
        gradient[n+k]=2*input_weight*rate[k]+dt*adjoint[2]
        adjoint=adjoint+state_gradient[:,k]+np.array([0.,0.,dt*velocity[k]*(-s*adjoint[0]+c*adjoint[1])])
    return float(value),gradient
