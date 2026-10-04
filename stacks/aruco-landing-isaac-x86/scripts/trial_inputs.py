"""Explicit paper grid inputs, expressed at the camera in the pad frame."""


def grid_shape(config):
    protocol = config.get('initial_protocol')
    if protocol is None:
        return None
    if protocol['kind'] != 'funnel_grid':
        raise ValueError('unsupported initial protocol')
    n, repeats = protocol['points_per_axis'], protocol['repeats']
    if not isinstance(n, int) or not isinstance(repeats, int) or n < 2 or repeats < 1:
        raise ValueError('grid needs at least two points per axis and positive repeats')
    if protocol['pad_side_m'] <= 0 or protocol['lateral_uncertainty_m'] < 0 or protocol['h_max_m'] <= 0:
        raise ValueError('invalid funnel dimensions')
    return n, repeats


def trial_initial_condition(config, trial_id, marker_plane_z_m, random_sampler):
    shape = grid_shape(config)
    if shape is None:
        return random_sampler(config['seed'], trial_id, config['initial_bounds'])
    n, repeats = shape
    if not 0 <= trial_id < n*n*repeats:
        raise ValueError('trial ID outside prescribed grid')
    protocol = config['initial_protocol']
    repeat, cell = divmod(trial_id, n*n)
    iy, ix = divmod(cell, n)
    # The user selected the revised funnel: fw(hmax) = L/2 + epsilon.
    half = protocol['pad_side_m']/2 + protocol['lateral_uncertainty_m']
    x, y = (-half + 2*half*i/(n-1) for i in (ix, iy))
    offset = config['camera']['body_position_m']
    # Initial body attitude is zero, so the mount translation needs no rotation.
    height = protocol['h_max_m']
    return dict(x=x-offset[0], y=y-offset[1],
                z=marker_plane_z_m+height-offset[2], yaw_deg=0.,
                grid_x_index=ix, grid_y_index=iy, repeat_index=repeat,
                camera_initial_position_pad_m=[x, y, height])
