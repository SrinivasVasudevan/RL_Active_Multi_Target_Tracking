import math
from typing import Tuple

import torch
from torch import tensor


def se2_kinematics(x: tensor, action: tensor, tau: float) -> tensor:
    wt_2 = action[1] * tau / 2
    t_v_sinc_term = tau * action[0] * torch.sinc(wt_2 / torch.pi)
    ret_x = torch.empty(3, dtype=x.dtype, device=x.device)
    ret_x[0] = x[0] + t_v_sinc_term * torch.cos(x[2] + wt_2)
    ret_x[1] = x[1] + t_v_sinc_term * torch.sin(x[2] + wt_2)
    ret_x[2] = wrap_to_pi(x[2] + 2 * wt_2)
    return ret_x


def landmark_motion(mu: tensor, v: tensor, a: tensor, b: tensor) -> tensor:
    return mu @ a.T + v @ b.T


def landmark_motion_real(mu: tensor, v: tensor, a: tensor, b: tensor, w: tensor) -> tensor:
    noise = torch.normal(mean=torch.zeros_like(mu), std=torch.sqrt(w))
    return mu @ a.T + v @ b.T + noise


def triangle_sdf(q: tensor, psi: float, r: float) -> tensor:
    original_shape = q.shape[:-1]
    q_flat = q.reshape(-1, 2)
    x, y = q_flat[:, 0], q_flat[:, 1]

    psi_val = torch.as_tensor(psi, device=q_flat.device, dtype=q_flat.dtype).reshape(())
    r_val = torch.as_tensor(r, device=q_flat.device, dtype=q_flat.dtype).reshape(())
    tan_psi = torch.tan(psi_val)
    p_x = r_val / (1 + torch.sin(psi_val))

    a_1 = torch.stack((-torch.ones_like(tan_psi), 1 / tan_psi))
    a_2 = torch.stack((-torch.ones_like(tan_psi), -1 / tan_psi))
    a_3 = q_flat.new_tensor([1.0, 0.0])
    b_1, b_2, b_3 = 0.0, 0.0, -r_val

    q_1 = torch.stack((r_val, r_val * tan_psi))
    q_2 = torch.stack((r_val, -r_val * tan_psi))
    q_3 = q_flat.new_tensor([0.0, 0.0])

    l_1_low, l_1_up, l_2_low, l_2_up = _l_function(x, psi_val, r_val, p_x)

    sdf = torch.empty_like(x)

    cond_1 = y >= l_1_up
    sdf[cond_1] = torch.linalg.norm(q_flat[cond_1] - q_1, dim=1)

    cond_2 = (~cond_1) & (y >= l_1_low) & (y < l_1_up)
    sdf[cond_2] = (q_flat[cond_2] @ a_1 + b_1) / torch.linalg.norm(a_1)

    cond_3 = (~cond_1) & (~cond_2) & (x < 0) & (y >= l_2_up) & (y < l_1_low)
    sdf[cond_3] = torch.linalg.norm(q_flat[cond_3] - q_3, dim=1)

    cond_4 = (~cond_1) & (~cond_2) & (~cond_3) & (x > p_x) & (y >= l_2_up) & (y < l_1_low)
    sdf[cond_4] = (q_flat[cond_4] @ a_3 + b_3) / torch.linalg.norm(a_3)

    cond_5 = (~cond_1) & (~cond_2) & (~cond_3) & (~cond_4) & (y > l_2_low) & (y < l_2_up)
    sdf[cond_5] = (q_flat[cond_5] @ a_2 + b_2) / torch.linalg.norm(a_2)

    cond_6 = ~(cond_1 | cond_2 | cond_3 | cond_4 | cond_5)
    sdf[cond_6] = torch.linalg.norm(q_flat[cond_6] - q_2, dim=1)

    return sdf.reshape(original_shape)


def _l_function(x: tensor, psi: tensor, r: tensor, p_x: tensor) -> Tuple[tensor, tensor, tensor, tensor]:
    ones = torch.ones_like(x)
    l_1_low = r * torch.tan(psi) * ones
    l_2_up = -r * torch.tan(psi) * ones

    inds_1 = torch.nonzero(x < 0, as_tuple=False)
    l_1_low[inds_1] = -x[inds_1] / torch.tan(psi)
    l_2_up[inds_1] = x[inds_1] / torch.tan(psi)

    inds_2 = torch.nonzero(torch.logical_and(0 <= x, x < p_x), as_tuple=False)
    l_1_low[inds_2] = 0
    l_2_up[inds_2] = 0

    inds_3 = torch.nonzero(torch.logical_and(p_x <= x, x < r), as_tuple=False)
    l_1_low[inds_3] = torch.tan(torch.pi / 4 + psi / 2) * x[inds_3] - r / torch.cos(psi)
    l_2_up[inds_3] = -torch.tan(torch.pi / 4 + psi / 2) * x[inds_3] + r / torch.cos(psi)

    l_1_up = r * torch.tan(psi) * ones
    l_2_low = -r * torch.tan(psi) * ones

    inds_4 = torch.nonzero(x < r, as_tuple=False)
    l_1_up[inds_4] = -(x[inds_4] - r) / torch.tan(psi) + r * torch.tan(psi)
    l_2_low[inds_4] = (x[inds_4] - r) / torch.tan(psi) - r * torch.tan(psi)

    return l_1_low, l_1_up, l_2_low, l_2_up


def phi(sdf: tensor, kappa: float) -> tensor:
    return 0.5 * (1 + torch.erf(sdf / (2 ** 0.5 * kappa) - 2))


def wrap_to_pi(theta: tensor) -> tensor:
    return (theta + math.pi) % (2 * math.pi) - math.pi
