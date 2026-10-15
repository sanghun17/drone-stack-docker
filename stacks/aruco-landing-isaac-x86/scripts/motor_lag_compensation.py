"""Optional stack-owned one-step compensation of the existing native RPM motor lag.
No native dynamics/gains are edited. Inversion is exact only without active limits.
"""
import numpy as np

def _api(x):
    if hasattr(x,'detach'):
        import torch
        return torch
    return np

def _clip(x, lo, hi, xp):
    return xp.clamp(x,min=lo,max=hi) if xp is not np else np.clip(x,lo,hi)

def predict_native_motor_step(command,current,k,tau_inc,tau_dec,dt,max_rate,low,high,scheme='rk4',discrete=True):
    xp=_api(current)
    command=_clip(command,low,high,xp)
    tau=xp.where(command<current,tau_dec,tau_inc)
    f=1/(dt+tau) if discrete else 1/tau
    omega=xp.sqrt(_clip(current/k,0,None,xp));desired=xp.sqrt(_clip(command/k,0,None,xp))
    error=desired-omega
    def rate(e):return _clip(f*e,-max_rate,max_rate,xp)
    k1=rate(error)
    if scheme=='euler':delta=dt*k1
    elif scheme=='rk4':
        k2=rate(error-.5*dt*k1);k3=rate(error-.5*dt*k2);k4=rate(error-dt*k3)
        delta=dt/6*(k1+2*k2+2*k3+k4)
    else:raise ValueError('Unsupported native motor integration scheme')
    return _clip(k*(omega+delta)**2,low,high,xp)

def compensate_motor_thrust(target,current,k,tau_inc,tau_dec,dt,max_rate,low,high,scheme='rk4',discrete=True):
    if not discrete:raise ValueError('Opt-in compensation requires the inspected native discrete mixing model')
    if scheme not in ('euler','rk4'):raise ValueError('Unsupported motor integrator')
    xp=_api(current)
    feasible_target=_clip(target,low,high,xp)
    tau=xp.where(feasible_target<current,tau_dec,tau_inc)
    q=dt/(dt+tau)
    beta=q if scheme=='euler' else q-q*q/2+q**3/6-q**4/24
    omega=xp.sqrt(_clip(current/k,0,None,xp));omega_target=xp.sqrt(feasible_target/k)
    omega_raw=omega+(omega_target-omega)/beta
    raw=k*_clip(omega_raw,0,None,xp)**2
    clipped=_clip(raw,low,high,xp)
    applied_tau=xp.where(clipped<current,tau_dec,tau_inc)
    rate_requested=(xp.sqrt(clipped/k)-omega)/(dt+applied_tau)
    return dict(command_raw=raw,command_clamped=clipped,
        predicted_next=predict_native_motor_step(raw,current,k,tau_inc,tau_dec,dt,max_rate,low,high,scheme,discrete),
        input_saturated=(raw<low)|(raw>high),target_saturated=(target<low)|(target>high),
        negative_inverse_speed=omega_raw<0,rate_limited=xp.abs(rate_requested)>max_rate,
        inverse_beta=beta)


def validate_native_actuator(actuator, expected_source_sha256):
    """Read-only guard on the actual actuator source and supported integration."""
    import inspect, hashlib
    from pathlib import Path
    actual = hashlib.sha256(Path(inspect.getfile(type(actuator))).read_bytes()).hexdigest()
    if actual != expected_source_sha256:
        raise ValueError('Native thruster source differs from the inspected compensation model')
    if not actuator.cfg.use_discrete_approximation:
        raise ValueError('Compensation opt-in supports the inspected discrete mixing model only')
    if actuator.cfg.integration_scheme not in ('rk4','euler'):
        raise ValueError('Unsupported native integration scheme')
    return dict(revision='rpm_one_step_inverse_v1', native_thruster_source_sha256=actual,
        integration_scheme=actuator.cfg.integration_scheme, use_discrete_approximation=True,
        dt_s=actuator.cfg.dt, native_force_bounds_N=[actuator.min_thrust,actuator.max_thrust],
        negative_inverse_speed_fallback='zero inverse RPM followed by unchanged native lower-force clamp',
        limits='Force/rate limits remain native; predicted and actual tracking error and all flags are logged')
