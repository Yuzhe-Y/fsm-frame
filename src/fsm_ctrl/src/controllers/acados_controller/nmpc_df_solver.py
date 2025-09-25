#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Acados Solver definition."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import casadi as ca
# 修改导入语句以匹配模型文件
from nmpc_df_model import CaControl, CaState, get_acados_model
import numpy as np
import scipy.linalg
import time


@dataclass
class AcadosMPCParams:
    """
    Model parameters.

    :param Q (np.ndarray): State weight matrix.
    [x, y, z, vx, vy, vz, qx, qy, qz, wx, wy, wz, f0, f1, f2, f3]
    :param Qe (np.ndarray): Terminal state weight matrix.
    [x, y, z, vx, vy, vz, qx, qy, qz, wx, wy, wz, f0, f1, f2, f3]
    :param R (np.ndarray): Control weight matrix.
    [f0_dot, f1_dot, f2_dot, f3_dot]
    :param lbu (np.ndarray): Lower bounds on control input.
    [f0_dot, f1_dot, f2_dot, f3_dot]_min
    :param ubu (np.ndarray): Upper bounds on control input.
    [f0_dot, f1_dot, f2_dot, f3_dot]_max
    :param lbx (np.ndarray): Lower bounds on state.
    [wx, wy, wz, f0, f1, f2, f3]_min
    :param ubx (np.ndarray): Upper bounds on state.
    [wx, wy, wz, f0, f1, f2, f3]_max
    :param p (np.ndarray): Parameter vector.
    [qw_ref, qx_ref, qy_ref, qz_ref]
    """

    Q: np.ndarray = np.zeros((16, 16))  # State cost + Control cost for intermediate nodes
    Qe: np.ndarray = np.zeros((16, 16))  # Terminal cost for states only
    R: np.ndarray = np.zeros((4, 4))
    lbu: np.ndarray = np.zeros(4)
    ubu: np.ndarray = np.zeros(4)
    lbx: np.ndarray = np.zeros(7)
    ubx: np.ndarray = np.zeros(7)
    p: np.ndarray = np.zeros(4)  # 参考四元数 [qw_ref, qx_ref, qy_ref, qz_ref]

    def __str__(self):
        return (f'Q: \n{self.Q}\n'
                f'Qe: \n{self.Qe}\n'
                f'R: \n{self.R}\n'
                f'lbu: {self.lbu}\n'
                f'ubu: {self.ubu}\n'
                f'lbx: {self.lbx}\n'
                f'ubx: {self.ubx}\n'
                f'p: {self.p}')


class AcadosMPCSolver:
    """Acados Solver for Model Predictive Controller."""

    def __init__(
            self,
            prediction_steps: int,
            prediction_horizon: float,
            mpc_params: AcadosMPCParams,
            export_dir: str = 'mpc_generated_code') -> None:
        """
        Initialize the Acados MPC controller.

        :param prediction_steps(int): Prediction steps.
        :param prediction_horizon(float): Prediction horizon (seconds).
        :param mpc_params(MPCParams): MPC parameters.
        :param export_dir(str): Export directory for the generated code.
        """
        self.mpc_params = mpc_params
        self.export_directory = export_dir

        # Acados model
        self.acados_model = get_acados_model()

        # Define acados solver
        ocp = AcadosOcp()
        ocp.model = self.acados_model

        # initial values for parameter vector - can be updated stagewise
        ocp.parameter_values = self.mpc_params.p

        # Initial state and control
        x0 = CaState.get_state(
            position=np.array([0.0, 0.0, 0.0]),
            linear_velocity=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            angular_velocity=np.array([0.0, 0.0, 0.0]),
            motor_thrusts=np.array([2.450375, 2.450375, 2.450375, 2.450375])  # 重力补偿，假设质量1kg
        )
        x0_position = x0[:3]
        x0_linear_velocity = x0[3:6]
        x0_orientation = x0[6:10]
        x0_angular_velocity = x0[10:13]
        x0_motor_thrusts = x0[13:17]
        
        # 控制输入：四个电机推力导数，初始为0
        u0 = CaControl.get_control(
            f0_dot=0.0,
            f1_dot=0.0,
            f2_dot=0.0,
            f3_dot=0.0
        )

        # Cost
        cost = ocp.cost

        # weight matrix at intermediate shooting nodes (1 to N-1)
        cost.W = scipy.linalg.block_diag(self.mpc_params.Q, self.mpc_params.R)
        # weight matrix at terminal shooting node (N)
        cost.W_e = self.mpc_params.Qe

        # reference at intermediate shooting nodes (1 to N-1)
        # cost_y_expr = [position, linear_velocity, attitude_error, angular_velocity, motor_thrusts, control]
        cost.yref = np.concatenate([
            x0_position,          # Position reference
            x0_linear_velocity,   # Linear velocity reference
            np.zeros(3),          # Attitude error reference (always zero)
            x0_angular_velocity,  # Angular velocity reference
            x0_motor_thrusts,     # Motor thrusts reference
            u0                    # Control reference
        ])
        # reference at terminal shooting node (N)
        # cost_y_expr_e = [position, linear_velocity, attitude_error, angular_velocity, motor_thrusts]
        cost.yref_e = np.concatenate([
            x0_position,          # Position reference
            x0_linear_velocity,   # Linear velocity reference
            np.zeros(3),          # Attitude error reference (always zero)
            x0_angular_velocity,  # Angular velocity reference
            x0_motor_thrusts      # Motor thrusts reference
        ])

        # For nonlinear least squares cost
        # Set up the cost type
        cost.cost_type = 'NONLINEAR_LS'
        cost.cost_type_e = 'NONLINEAR_LS'

        # Constraints
        constraints = ocp.constraints
        # initial state
        constraints.x0 = x0
        # lower bounds on u at shooting nodes (0 to N-1)
        constraints.lbu = self.mpc_params.lbu
        # upper bounds on u at shooting nodes (0 to N-1)
        constraints.ubu = self.mpc_params.ubu
        # matrix coefficient for bounds on u at shooting nodes
        # constraints.Jbu = np.identity(4)
        # id of the bounds on u at shooting nodes (0 to N-1)
        constraints.idxbu = np.array([0, 1, 2, 3]) 
        # lower bounds on x at shooting nodes (0 to N-1)
        constraints.lbx = self.mpc_params.lbx
        # upper bounds on x at shooting nodes (0 to N-1)
        constraints.ubx = self.mpc_params.ubx
        # matrix coefficient for bounds on u at shooting nodes
        # constraints.Jbx = np.identity(3)
        # id of the bounds on u at shooting nodes (0 to N-1)
        constraints.idxbx = np.array([10, 11, 12, 13, 14, 15, 16])

        # Solver options
        solver_options = ocp.solver_options
        # number of shooting intervals
        solver_options.N_horizon = prediction_steps
        # prediction horizon
        solver_options.tf = prediction_horizon
        # QP solver to be used in the NLP solver. String in (
        # 'PARTIAL_CONDENSING_HPIPM', 'FULL_CONDENSING_QPOASES', 'FULL_CONDENSING_HPIPM',
        # 'PARTIAL_CONDENSING_QPDUNES', 'PARTIAL_CONDENSING_OSQP', 'FULL_CONDENSING_DAQP').
        # Default: 'PARTIAL_CONDENSING_HPIPM'.
        solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        # NLP solver. String in ('SQP', 'SQP_RTI', 'DDP'). Default: 'SQP_RTI'.
        solver_options.nlp_solver_type = 'SQP_RTI'
        # Hessian approximation. String in ('GAUSS_NEWTON', 'EXACT'). Default: 'GAUSS_NEWTON'.
        solver_options.hessian_approx = 'GAUSS_NEWTON'
        # Integrator type. String in ('ERK', 'IRK', 'GNSF', 'DISCRETE', 'LIFTED_IRK').
        # Default: 'ERK'.
        solver_options.integrator_type = 'ERK'

        # Create solver
        ocp.code_export_directory = self.export_directory + '/nmpc_f_generated_code'
        ocp.json_file = self.export_directory + '/nmpc_f_acados_ocp.json'

        self._solver = AcadosOcpSolver(
            ocp,
            # json_file = 'mpc_controller.json',
            generate=True,
            verbose=False)

        # Internal variables
        self.x_dim = x0.shape[0]
        self.u_dim = u0.shape[0]
        self.u0 = u0

    def update_mpc_params(self, mpc_params: AcadosMPCParams) -> None:
        """
        Update the MPC parameters.

        :param mpc_params(MPCParams): MPC parameters.
        """
        # Update Solver constraints:
        self.mpc_params = mpc_params

        # lower bounds on u at shooting nodes (0 to N-1)
        # upper bounds on u at shooting nodes (0 to N-1)
        for node in range(self.N):
            self.solver.constraints_set(node, 'lbu', self.mpc_params.lbu)
            self.solver.constraints_set(node, 'ubu', self.mpc_params.ubu)

        # lower bounds on x at shooting nodes (1 to N)
        # upper bounds on x at shooting nodes (1 to N)
        for node in range(self.N-1):
            self.solver.constraints_set(node + 1, 'lbx', self.mpc_params.lbx)
            self.solver.constraints_set(node + 1, 'ubx', self.mpc_params.ubx)
        # self.solver.constraints_set(20, 'lbx', self.mpc_params.lbx)
        # self.solver.constraints_set(20, 'ubx', self.mpc_params.ubx)

        # initial values for parameter vector - can be updated stagewise
        for i in range(self.N+1):
            self.solver.set(i, 'p', self.mpc_params.p)

        # weight matrix at intermediate shooting nodes (1 to N-1)
        for node in range(0, self.N):
            self.solver.cost_set(node, 'W', scipy.linalg.block_diag(
                self.mpc_params.Q, self.mpc_params.R))

        # weight matrix at terminal shooting node (N)
        self.solver.cost_set(self.N, 'W', self.mpc_params.Qe)

    def export_integrador(self, simulation_time) -> AcadosSimSolver:
        """Export integrator for simulation."""
        # Acados Sim
        acados_sim = AcadosSim()
        acados_sim.model = self.acados_model
        acados_sim.parameter_values = self.mpc_params.p

        # Solver options
        # integrator type. String in ('ERK', 'IRK', 'GNSF', 'DISCRETE', 'LIFTED_IRK').
        acados_sim.solver_options.integrator_type = 'IRK'
        # number of stages in the integrator
        acados_sim.solver_options.num_stages = 4
        # number of steps in the integrator
        acados_sim.solver_options.num_steps = 3
        # time horizon
        acados_sim.solver_options.T = simulation_time

        # Create integrator
        acados_sim.code_export_directory = self.export_directory + '/mpc_generated_code'
        json_file = self.export_directory + '/acados_sim.json'
        self.acados_integrator = AcadosSimSolver(
            acados_sim,
            json_file=json_file,
            generate=True,
            verbose=True)

        return self.acados_integrator

    @property
    def states(self):
        """
        Get the predicted state of the system.

        :return: The predicted state of the system.
        """
        states = np.zeros((
            self.solver.acados_ocp.solver_options.N_horizon+1, self.solver.get(0, 'x').shape[0]))
        for i in range(self.solver.acados_ocp.solver_options.N_horizon+1):
            states[i, :] = self.solver.get(i, 'x')
        return states

    @property
    def controls(self):
        """
        Get the predicted control input of the system.

        :return: The predicted control input of the system.
        """
        controls = np.zeros((
            self.solver.acados_ocp.solver_options.N_horizon, self.solver.get(0, 'u').shape[0]))
        for i in range(self.solver.acados_ocp.solver_options.N_horizon):
            controls[i, :] = self.solver.get(i, 'u')
        return controls

    def _set_state(self, state: np.ndarray) -> None:
        """
        Set the current state of the system.

        :param state: The current state of the system.
        """
        self.solver.set(0, 'lbx', state)
        self.solver.set(0, 'ubx', state)

    def _set_references(self, yref: np.ndarray, yref_e: np.ndarray) -> None:
        """
        Set the reference trajectory for the system.

        :param yref: The intermediate reference trajectory for the system
        (matrix of size [N, state_dim]).
        :param yref_e: The final reference trajectory for the system
        (matrix of size [state_dim]).
        """
        # 确保参考轨迹包含控制输入
        if yref.shape[1] == self.x_dim:
            # 如果只有状态，添加控制参考
            yref = np.concatenate([yref, np.tile(self.u0, (yref.shape[0], 1))], axis=1)

        for i in range(self.N):
            position = yref[i, :3]
            linear_velocity = yref[i, 3:6]
            orientation = yref[i, 6:10]
            angular_velocity = yref[i, 10:13]
            motor_thrusts = yref[i, 13:17]
            actuation = yref[i, 17:21]
            
            # if yref.shape[1] > self.x_dim:
            #     actuation = yref[i, self.x_dim:]
            # else:
            #     actuation = self.u0

            # 构建cost_y_expr对应的参考
            # cost_y_expr = [position, linear_velocity, attitude_error, angular_velocity, motor_thrusts, control]
            y_ref = np.concatenate([
                position,           # position
                linear_velocity,    # linear_velocity
                np.zeros(3),        # attitude error (always zero for reference)
                angular_velocity,   # angular_velocity
                motor_thrusts,      # motor_thrusts
                actuation          # control
            ])

            self.solver.set(i, 'yref', y_ref)
            
            # 设置参数（参考四元数）
            self.solver.set(i, 'p', orientation)

        # 设置终端参考
        position = yref_e[:3]
        linear_velocity = yref_e[3:6]
        orientation = yref_e[6:10]
        angular_velocity = yref_e[10:13]
        motor_thrusts = yref_e[13:17]

        # cost_y_expr_e = [position, linear_velocity, attitude_error, angular_velocity, motor_thrusts]
        y_ref_e = np.concatenate([
            position,           # position
            linear_velocity,    # linear_velocity
            np.zeros(3),        # attitude error (always zero for reference)
            angular_velocity,   # angular_velocity
            motor_thrusts       # motor_thrusts
        ])
        
        self.solver.set(self.N, 'yref', y_ref_e)
        self.solver.set(self.N, 'p', orientation)

    def evaluate(
            self,
            state: np.ndarray,
            reference_trajectory_intermediate: np.ndarray,
            reference_trajectory_final: np.ndarray) -> np.ndarray:
        """
        Simulate the system using MPC with a given state and reference trajectory.

        :param state: The current state of the system.
        :param reference_trajectory_intermediate: The intermediate reference trajectory
        for the system (matrix of size [N, state_dim]).
        :param reference_trajectory_final: The final reference trajectory for the system
        (matrix of size [state_dim]).

        :return: The control actions for all prediction steps.
        """
        # Set the reference trajectory
        self._set_references(reference_trajectory_intermediate, reference_trajectory_final)

        # Set current state
        self._set_state(state)

        # Solve the MPC problem
        status = self.solver.solve()
        if status != 0:
            print(f'MPC Solver failed with status {status}')
            return None

        # Get control actions and predicted states
        controls = self.controls  # Predicted control inputs
        states = self.states      # Predicted states

        return controls, states

    def compute_control_action(
            self,
            state: np.ndarray,
            reference_trajectory_intermediate: np.ndarray,
            reference_trajectory_final: np.ndarray) -> np.ndarray:
        """
        Compute the first control action using MPC.

        :param state: The current state of the system.
        :param reference_trajectory_intermediate: The intermediate reference trajectory
        for the system (matrix of size [N, state_dim]).
        :param reference_trajectory_final: The final reference trajectory for the system
        (matrix of size [state_dim]).

        :return: The first control action u0.
        """
        # Set the reference trajectory
        self._set_references(reference_trajectory_intermediate, reference_trajectory_final)

        # Solve the MPC problem
        status = self.solver.solve_for_x0(state)
        if status != 0:
            print(f'MPC Solver failed with status {status}')
            return None
            
        return self.solver.get(0, 'u')

    @property
    def solver(self):
        return self._solver

    @property
    def prediction_horizon(self):
        return self.solver.acados_ocp.solver_options.tf

    @property
    def prediction_steps(self):
        return self.solver.acados_ocp.solver_options.N_horizon

    @property
    def evaluation_time(self):
        return self.solver.acados_ocp.solver_options.shooting_nodes

    @property
    def N(self):
        """Index of maximum shooting node."""
        return self.solver.acados_ocp.solver_options.N_horizon

    def get_empty_reference(self):
        """
        Get an empty reference trajectory.

        :return: An empty reference trajectory.
        """
        return np.zeros((self.N, self.x_dim))

    def get_empty_end_reference(self):
        """
        Get an empty final reference trajectory.

        :return: An empty final reference trajectory.
        """
        return np.zeros(self.x_dim)

    def get_empty_state(self):
        """
        Get an empty state.

        :return: An empty state.
        """
        return np.zeros(self.x_dim)


if __name__ == '__main__':

    mpc_params = AcadosMPCParams(
        Q=CaState.get_cost_matrix(
            position_weight=40*np.ones(3),
            linear_velocity_weight=1.0*np.ones(3),
            orientation_weight=8.0*np.ones(3),
            angular_velocity_weight=1.0*np.ones(3),
            motor_thrust_weight=0.0*np.ones(4)
        ),
        Qe=CaState.get_cost_matrix(
            position_weight=40*np.ones(3),
            linear_velocity_weight=1.0*np.ones(3),
            orientation_weight=8.0*np.ones(3),
            angular_velocity_weight=1.0*np.ones(3),
            motor_thrust_weight=0.0*np.ones(4)
        ),
        R=CaControl.get_cost_matrix(
            motor_thrust_deriv_weight=0.1*np.ones(4)
        ),
        lbu=np.array([-50.0, -50.0, -50.0, -50.0]),  # 电机推力导数下界
        ubu=np.array([50.0, 50.0, 50.0, 50.0]),       # 电机推力导数上界
        lbx=np.array([-3.14, -3.14, -3.14, 0.0, 0.0, 0.0, 0.0]),  # 角速度，推力下界
        ubx=np.array([3.14, 3.14, 3.14, 50.0, 50.0, 50.0, 50.0]),  # 角速度，推力上界
        p=np.array([1.0, 0.0, 0.0, 0.0])  # 参考四元数 [qw, qx, qy, qz]
    )

    mpc = AcadosMPCSolver(
        prediction_steps=20,
        prediction_horizon=1,
        mpc_params=mpc_params
    )

    mpc.update_mpc_params(mpc_params)

    # 设置状态：[position(3), linear_velocity(3), orientation(4), angular_velocity(3), motor_thrusts(4)]
    state = np.array([
        0.0, 0.0, 0.0,        # Position
        0.0, 0.0, 0.0,        # Linear velocity
        1.0, 0.0, 0.0, 0.0,   # Orientation (quaternion)
        0.0, 0.0, 0.0,        # Angular velocity
        2.450375, 2.450375, 2.450375, 2.450375  # Motor thrusts (重力补偿)
    ])
    
    # 设置参考轨迹
    reference_intermediate = np.zeros((mpc.prediction_steps, mpc.x_dim + mpc.u_dim))
    reference_final = np.zeros(mpc.x_dim)
    
    # 设置位置参考为 [0, 0, 1]
    reference_intermediate[:, :3] = np.array([0.0, 0.0, 1.0])         # Position reference
    reference_intermediate[:, 3:6] = np.array([0.0, 0.0, 0.0])       # Linear velocity reference
    reference_intermediate[:, 6:10] = np.array([1.0, 0.0, 0.0, 0.0]) # Orientation reference
    reference_intermediate[:, 10:13] = np.array([0.0, 0.0, 0.0])     # Angular velocity reference
    reference_intermediate[:, 13:17] = np.array([2.450375, 2.450375, 2.450375, 2.450375])  # Motor thrusts reference
    reference_intermediate[:, 17:21] = np.array([0.0, 0.0, 0.0, 0.0])  # Motor thrusts reference
    
    reference_final[:3] = np.array([0.0, 0.0, 1.0])                  # Position reference
    reference_final[3:6] = np.array([0.0, 0.0, 0.0])                 # Linear velocity reference
    reference_final[6:10] = np.array([1.0, 0.0, 0.0, 0.0])           # Orientation reference
    reference_final[10:13] = np.array([0.0, 0.0, 0.0])               # Angular velocity reference
    reference_final[13:17] = np.array([2.450375, 2.450375, 2.450375, 2.450375])      # Motor thrusts reference

    start_time = time.time()

    u, opt_x = mpc.evaluate(
        state,
        reference_intermediate,
        reference_final
    )

    end_time = time.time()
    
    # 计算执行时间（以毫秒为单位）
    execution_time_ms = (end_time - start_time) * 1000
    
    # 打印结果和执行时间
    if u is not None:
        print("Computed control actions:")
        # print(f"First control action u0: {u[0]}")
        print(f"Motor thrust derivatives [f0_dot, f1_dot, f2_dot, f3_dot]: {u[0]}")
        # print(f"All control actions shape: {u.shape}")
    else:
        print("Failed to compute control actions")
    print(f"Execution time: {execution_time_ms:.2f} ms")