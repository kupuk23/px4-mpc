############################################################################
#
#   Copyright (C) 2024 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

from acados_template import AcadosModel
import casadi as cs
import numpy as np

class SpacecraftVisualServoModel():
    def __init__(self):
        self.name = 'spacecraft_visual_servo_model'

        # constants
        self.mass = 16.8
        self.inertia = np.diag((0.1454, 0.1366, 0.1594))
        self.max_thrust = 1.5
        self.max_rate = 0.5
        self.torque_arm_length = 0.12

        self.theta_max_deg = 20

    def get_acados_model(self) -> AcadosModel:
        def skew_symmetric(v):
            return cs.vertcat(cs.horzcat(0, -v[0], -v[1], -v[2]),
                cs.horzcat(v[0], 0, v[2], -v[1]),
                cs.horzcat(v[1], -v[2], 0, v[0]),
                cs.horzcat(v[2], v[1], -v[0], 0))

        def q_to_rot_mat(q):
            qw, qx, qy, qz = q[0], q[1], q[2], q[3]

            rot_mat = cs.vertcat(
                cs.horzcat(1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
                cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)),
                cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)))

            return rot_mat

        def v_dot_q(v, q):
            rot_mat = q_to_rot_mat(q)

            return cs.mtimes(rot_mat, v)

        model = AcadosModel()


        
        # set up states & controls
        p      = cs.MX.sym('p', 3)
        v      = cs.MX.sym('v', 3)
        q      = cs.MX.sym('q', 4)
        w      = cs.MX.sym('w', 3)
        p_obj = cs.MX.sym('p_obj', 3)  # Object position in inertial frame

        x = cs.vertcat(p, v, q, w)

        # compute bearing inequality

        r_I = p_obj - p  # Vector from robot to object in inertial frame
        # Transform to body frame using the rotation matrix
        r_B = cs.mtimes(cs.transpose(q_to_rot_mat(q)), r_I)
        # Compute vector norm
        r_B_norm = cs.sqrt(r_B[0]**2 + r_B[1]**2 + r_B[2]**2)  # More explicit form
        cos_theta_max = cs.cos(np.deg2rad(self.theta_max_deg))
        g_x = cos_theta_max * r_B_norm - r_B[0]
        

        u = cs.MX.sym('u', 4)
        D_mat = cs.MX.zeros(2, 4)
        D_mat[0, 0] = 1
        D_mat[0, 1] = 1
        D_mat[1, 2] = -1
        D_mat[1, 3] = -1

        # L mat
        L_mat = cs.MX.zeros(1, 4)
        L_mat[0, 0] = -1
        L_mat[0, 1] = 1
        L_mat[0, 2] = -1
        L_mat[0, 3] = 1
        L_mat = L_mat * self.torque_arm_length

        F_2d = cs.mtimes(D_mat, u)
        tau_1d = cs.mtimes(L_mat, u)

        F = cs.vertcat(F_2d[0, 0], F_2d[1, 0], 0.0)
        tau = cs.vertcat(0.0, 0.0, tau_1d)

        # xdot
        p_dot      = cs.MX.sym('p_dot', 3)
        v_dot      = cs.MX.sym('v_dot', 3)
        q_dot      = cs.MX.sym('q_dot', 4)
        w_dot      = cs.MX.sym('w_dot', 3)

        xdot = cs.vertcat(p_dot, v_dot, q_dot, w_dot)

        a_thrust = v_dot_q(F, q)/self.mass

        # dynamics
        f_expl = cs.vertcat(v,
                            a_thrust,
                            1 / 2 * cs.mtimes(skew_symmetric(w), q),
                            np.linalg.inv(self.inertia) @ (tau - cs.cross(w, self.inertia @ w))
                            )

        f_impl = xdot - f_expl

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.name = self.name

        # Add model parameters
        model.p = p_obj  # Use object position as parameter

         # Define nonlinear constraint
        model.con_h_expr = g_x
        model.con_h_expr_e = g_x
        
        return model
