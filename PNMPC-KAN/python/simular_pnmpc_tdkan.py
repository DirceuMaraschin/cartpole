# -*- coding: utf-8 -*-
"""Testa PNMPC-ESNI com modelo TDKAN e planejamento não linear de referência."""

import argparse
import csv
import json
import os
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from kan import KAN
from scipy.linalg import solve_discrete_are
from scipy.optimize import Bounds, LinearConstraint, differential_evolution, minimize

from exportar_kan import extrair_camadas
from treinar_kan_exemplo import F_MAX, rk4_step, wrap_pi


FEATURE_SCALE = np.array([0.30, 3.0, 1.0, 1.0, 8.0], dtype=np.float32)
INPUT_DELAY = 2
OUTPUT_DELAY = 2
N_INPUTS = (INPUT_DELAY + 1) * len(FEATURE_SCALE) + OUTPUT_DELAY + 1
N_OUTPUTS = len(FEATURE_SCALE)
TRACK_LIMIT = 0.30
ANGLE_ERROR_SCALE = np.pi / 2.0
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def encode_state(state):
    x, dx, theta, dtheta = state
    return np.array([x, dx, np.sin(theta), np.cos(theta), dtheta], dtype=np.float32)


def decode_state(features):
    x, dx, sin_theta, cos_theta, dtheta = features
    theta = np.arctan2(sin_theta, cos_theta)
    return np.array([x, dx, theta, dtheta], dtype=np.float32)


def normalize_angle_features(features):
    features = np.asarray(features, dtype=np.float32).copy()
    norm = np.hypot(features[..., 2], features[..., 3])
    safe = np.maximum(norm, 1e-6)
    features[..., 2] /= safe
    features[..., 3] /= safe
    return features


def make_regressor(output_history, input_history, next_input):
    features = np.concatenate(
        (output_history.reshape(-1), next_input.reshape(-1), input_history.reshape(-1))
    )
    scales = np.concatenate(
        (
            np.tile(FEATURE_SCALE, OUTPUT_DELAY + 1),
            np.full(OUTPUT_DELAY + 1, F_MAX, dtype=np.float32),
        )
    )
    return (features / scales).astype(np.float32)


def collect_training_data(n_samples, seed):
    rng = np.random.default_rng(seed)
    regressors, targets = [], []
    while len(regressors) < n_samples:
        state = np.array(
            [
                rng.uniform(-0.15, 0.15),
                rng.uniform(-0.7, 0.7),
                rng.uniform(-np.pi, np.pi),
                rng.uniform(-1.8, 1.8),
            ],
            dtype=np.float32,
        )
        states = [state.copy()]
        actions = []
        action = float(rng.uniform(-5.0, 5.0))
        segment_left = 0
        for _ in range(500):
            if segment_left <= 0:
                action = float(np.clip(0.65 * action + rng.normal(0.0, 3.0), -F_MAX, F_MAX))
                segment_left = int(rng.integers(3, 16))
            segment_left -= 1
            actions.append(action)
            state = rk4_step(state, action, 0.02)
            states.append(state.copy())
            if abs(state[0]) > 0.28 or abs(state[3]) > 12.0 or not np.all(np.isfinite(state)):
                break

        encoded = np.asarray([encode_state(item) for item in states], dtype=np.float32)
        actions = np.asarray(actions, dtype=np.float32)
        for k in range(OUTPUT_DELAY, len(actions)):
            output_history = encoded[k - OUTPUT_DELAY:k + 1][::-1]
            input_history = actions[k - OUTPUT_DELAY:k + 1][::-1]
            regressors.append(make_regressor(output_history, input_history[1:], input_history[:1]))
            targets.append(encoded[k + 1] / FEATURE_SCALE)
            if len(regressors) >= n_samples:
                break

    return np.asarray(regressors), np.asarray(targets, dtype=np.float32)


def train_or_load_model(args):
    os.makedirs(args.output_dir, exist_ok=True)
    checkpoint = os.path.join(args.output_dir, "tdkan_model.pt")
    model = KAN(
        width=[N_INPUTS, 9, N_OUTPUTS],
        grid=5,
        k=3,
        grid_range=[-2, 2],
        seed=args.seed,
        auto_save=False,
    ).to(DEVICE)

    if os.path.isfile(checkpoint) and not args.retrain:
        payload = torch.load(checkpoint, map_location=DEVICE, weights_only=True)
        model.load_state_dict(payload["state_dict"])
        print(
            f"Modelo TDKAN carregado: {checkpoint} "
            f"(amostras={payload.get('samples', 'desconhecido')}, "
            f"passos={payload.get('training_steps', 'desconhecido')})"
        )
        return model, float(payload.get("test_rmse", float("nan")))

    X, Y = collect_training_data(args.samples, args.seed)
    rng = np.random.default_rng(args.seed + 1)
    order = rng.permutation(len(X))
    n_train = int(0.85 * len(order))
    train_idx, test_idx = order[:n_train], order[n_train:]
    x_train = torch.as_tensor(X[train_idx], dtype=torch.float32, device=DEVICE)
    y_train = torch.as_tensor(Y[train_idx], dtype=torch.float32, device=DEVICE)
    x_test = torch.as_tensor(X[test_idx], dtype=torch.float32, device=DEVICE)
    y_test = torch.as_tensor(Y[test_idx], dtype=torch.float32, device=DEVICE)
    dataset = {
        "train_input": x_train,
        "train_label": y_train,
        "test_input": x_test,
        "test_label": y_test,
    }

    print(
        f"Treinando TDKAN em {DEVICE} ({len(train_idx)} treino / "
        f"{len(test_idx)} teste; atrasos d=r={INPUT_DELAY})..."
    )
    model.fit(dataset, opt="LBFGS", steps=args.training_steps, lamb=1e-5)
    model.save_act = False
    with torch.no_grad():
        test_rmse = torch.sqrt(torch.mean((model(x_test) - y_test) ** 2)).item()
    torch.save(
        {
            "state_dict": model.state_dict(),
            "test_rmse": test_rmse,
            "samples": args.samples,
            "training_steps": args.training_steps,
            "seed": args.seed,
        },
        checkpoint,
    )
    return model, test_rmse


def bsplines_batch(values, grid, order):
    basis = ((values[:, None] >= grid[:-1]) & (values[:, None] < grid[1:])).astype(np.float64)
    for degree in range(1, order + 1):
        count = len(grid) - 1 - degree
        left_denominator = grid[degree:degree + count] - grid[:count]
        right_denominator = grid[degree + 1:degree + 1 + count] - grid[1:count + 1]
        left = np.zeros((len(values), count), dtype=np.float64)
        right = np.zeros_like(left)
        valid_left = left_denominator != 0
        valid_right = right_denominator != 0
        left[:, valid_left] = (
            (values[:, None] - grid[:count][valid_left])
            / left_denominator[valid_left]
            * basis[:, :count][:, valid_left]
        )
        right[:, valid_right] = (
            (grid[degree + 1:degree + 1 + count][valid_right] - values[:, None])
            / right_denominator[valid_right]
            * basis[:, 1:count + 1][:, valid_right]
        )
        basis = left + right
    return basis


def evaluate_kan_batch(layers, inputs):
    values = np.asarray(inputs, dtype=np.float64)
    for layer in layers:
        output = np.zeros((len(values), layer["n_sai"]), dtype=np.float64)
        for i in range(layer["n_ent"]):
            basis = bsplines_batch(values[:, i], layer["grid"][i], layer["k"])
            silu = values[:, i] / (1.0 + np.exp(-values[:, i]))
            output += (
                silu[:, None] * layer["sb"][i][None, :]
                + (basis @ layer["coef"][i].T) * layer["sp"][i][None, :]
            )
        values = output * layer["a"][None, :] + layer["c"][None, :]
    return values.astype(np.float32)


def predict_batch(layers, output_history, input_history, current_inputs):
    batch = len(current_inputs)
    regressors = np.stack(
        [
            make_regressor(output_history[i], input_history[i], current_inputs[i])
            for i in range(batch)
        ]
    )
    prediction = evaluate_kan_batch(layers, regressors) * FEATURE_SCALE
    return normalize_angle_features(prediction)


def rollout_batch(layers, output_history, input_history, action_sequences):
    batch, horizon = action_sequences.shape
    output_history = np.repeat(output_history[None, :, :], batch, axis=0)
    input_history = np.repeat(input_history[None, :], batch, axis=0)
    predictions = np.empty((batch, horizon, N_OUTPUTS), dtype=np.float32)
    for step in range(horizon):
        y_next = predict_batch(layers, output_history, input_history, action_sequences[:, step])
        predictions[:, step] = y_next
        output_history = np.concatenate((y_next[:, None, :], output_history[:, :OUTPUT_DELAY, :]), axis=1)
        input_history = np.concatenate(
            (action_sequences[:, step:step + 1], input_history[:, :OUTPUT_DELAY - 1]), axis=1
        )
    return predictions


def prediction_errors(predictions, theta_ref):
    theta = np.arctan2(predictions[..., 2], predictions[..., 3])
    theta_error = wrap_pi(theta - theta_ref)
    return np.stack(
        (predictions[..., 0], theta_error, predictions[..., 1], predictions[..., 4]),
        axis=-1,
    )


def prediction_states(predictions):
    theta = np.arctan2(predictions[..., 2], predictions[..., 3])
    return np.stack((predictions[..., 0], theta, predictions[..., 1], predictions[..., 4]), axis=-1)


def estimate_innovation(measured_state, modeled_state):
    return np.array(
        [
            measured_state[0] - modeled_state[0],
            wrap_pi(measured_state[2] - modeled_state[2]),
            measured_state[1] - modeled_state[1],
            measured_state[3] - modeled_state[3],
        ],
        dtype=np.float32,
    )


def correct_free_response(model_response, theta_ref, innovation):
    corrected = prediction_errors(model_response, theta_ref) + innovation
    corrected[..., 1] = wrap_pi(corrected[..., 1])
    return corrected


def solve_pnmpc(
    layers,
    y_history,
    u_history,
    u_previous,
    theta_ref,
    innovation,
    args,
    reference_states=None,
    nominal_forces=None,
    control_weight=None,
    horizon=None,
):
    horizon = args.prediction_horizon if horizon is None else horizon
    control_horizon = args.control_horizon

    if nominal_forces is None:
        nominal_forces = np.full(horizon, u_previous, dtype=np.float64)
    else:
        nominal_forces = np.asarray(nominal_forces, dtype=np.float64)
        if nominal_forces.shape != (horizon,):
            raise ValueError("A sequência nominal deve ter o tamanho do horizonte preditivo.")

    action_sequences = np.repeat(nominal_forces[None, :], control_horizon + 1, axis=0)
    perturbations = []
    for j in range(control_horizon):
        delta = args.step_amplitude
        if nominal_forces[j] + delta > F_MAX:
            delta = -args.step_amplitude
        if nominal_forces[j] + delta < -F_MAX:
            delta = args.step_amplitude
        perturbations.append(delta)
        action_sequences[j + 1, j:] += delta

    responses = rollout_batch(layers, y_history, u_history, action_sequences)
    if reference_states is None:
        free_outputs = correct_free_response(responses[0], theta_ref, innovation)
    else:
        reference_states = np.asarray(reference_states, dtype=np.float64)
        if reference_states.shape != (horizon, 4):
            raise ValueError("A trajetória de referência deve conter quatro estados por passo.")
        free_outputs = prediction_states(responses[0]) - reference_states
        free_outputs += innovation
        free_outputs[:, 1] = wrap_pi(free_outputs[:, 1])

    # Finite-difference linearization around the free response: G_PNMPC.
    dynamic_matrix = np.empty((4 * horizon, control_horizon), dtype=np.float64)
    output_weights = np.array(
        [
            np.sqrt(args.x_weight) / TRACK_LIMIT,
            np.sqrt(args.theta_weight) / ANGLE_ERROR_SCALE,
            np.sqrt(args.dx_weight) / FEATURE_SCALE[1],
            np.sqrt(args.dtheta_weight) / FEATURE_SCALE[4],
        ]
    )
    terminal_weights = np.ones(horizon)
    terminal_weights[-1] = np.sqrt(args.terminal_weight)
    row_weights = (terminal_weights[:, None] * output_weights[None, :]).reshape(-1)
    response_states = prediction_states(responses)
    for j, delta in enumerate(perturbations):
        response_delta = response_states[j + 1] - response_states[0]
        response_delta[:, 1] = wrap_pi(response_delta[:, 1])
        dynamic_matrix[:, j] = (response_delta / delta).reshape(-1) * row_weights

    free_vector = free_outputs.reshape(-1) * row_weights
    increments_to_force = np.tril(np.ones((horizon, control_horizon)))
    hessian = dynamic_matrix.T @ dynamic_matrix
    gradient = dynamic_matrix.T @ free_vector
    if control_weight is None:
        control_weight = args.control_weight
    hessian += control_weight * np.eye(control_horizon)
    hessian += args.force_weight * (increments_to_force.T @ increments_to_force)
    gradient += args.force_weight * increments_to_force.T @ nominal_forces

    def objective(increments):
        return float(increments @ hessian @ increments + 2.0 * gradient @ increments)

    def objective_jac(increments):
        return 2.0 * (hessian @ increments + gradient)

    force_constraint = LinearConstraint(
        increments_to_force,
        -F_MAX - nominal_forces,
        F_MAX - nominal_forces,
    )
    result = minimize(
        objective,
        np.zeros(control_horizon),
        jac=objective_jac,
        method="SLSQP",
        bounds=Bounds(
            np.full(control_horizon, -args.max_delta_u),
            np.full(control_horizon, args.max_delta_u),
        ),
        constraints=[force_constraint],
        options={"maxiter": 60, "ftol": 1e-8},
    )
    if not result.success or not np.all(np.isfinite(result.x)):
        increments = np.zeros(control_horizon)
        solver_failed = True
    else:
        increments = result.x
        solver_failed = False

    u = float(np.clip(nominal_forces[0] + increments[0], -F_MAX, F_MAX))
    return u, solver_failed, free_outputs


def build_capture_reference(args, initial_state, theta_ref):
    plan, plan_horizon, control_interval, optimization_seconds = load_or_optimize_trajectory(
        args, initial_state, theta_ref
    )
    steps_per_control = int(round(control_interval / args.dt))
    plan_controls = np.repeat(plan, steps_per_control)
    reference = np.empty((len(plan_controls) + 1, 4), dtype=np.float64)
    reference[0] = initial_state
    for k, force in enumerate(plan_controls):
        reference[k + 1] = rk4_step(reference[k], float(force), args.dt)
    target = np.array([0.0, 0.0, theta_ref, 0.0], dtype=np.float64)
    return plan_controls, reference, target, plan_horizon, optimization_seconds


def simulate_tdkan(layers, args):
    dt = args.dt
    count = int(round(args.duration / dt))
    sensor_offset = np.deg2rad(args.sensor_offset_deg)
    theta_ref = wrap_pi(np.deg2rad(args.theta_ref) - sensor_offset)
    theta_initial = wrap_pi(np.deg2rad(args.theta_inicial) - sensor_offset)
    state = np.array([0.0, 0.0, theta_initial, 0.0], dtype=np.float64)
    reference_controls, reference_trajectory, target_state, plan_horizon, optimization_seconds = (
        build_capture_reference(args, state, theta_ref)
    )
    terminal_gain = terminal_lqr_gain(theta_ref, dt)
    y_history = np.repeat(encode_state(state)[None, :], OUTPUT_DELAY + 1, axis=0)
    u_history = np.zeros(OUTPUT_DELAY, dtype=np.float32)
    u_previous = 0.0
    last_innovation = np.zeros(4, dtype=np.float32)

    times = np.arange(count + 1, dtype=np.float64) * dt
    states = np.empty((count + 1, 4), dtype=np.float64)
    states[0] = state
    forces = np.zeros(count, dtype=np.float64)
    predicted_next = np.zeros((count, 2), dtype=np.float64)
    innovations = np.zeros((count, 4), dtype=np.float64)
    solver_failed = np.zeros(count, dtype=bool)
    compute_ms = np.zeros(count, dtype=np.float64)
    rail_hit = np.zeros(count, dtype=bool)
    for k in range(count):
        started = time.perf_counter()
        capture_phase = k < len(reference_controls)
        horizon_steps = (
            args.prediction_horizon
            if capture_phase
            else args.hold_prediction_horizon
        )
        reference_indices = np.arange(k + 1, k + horizon_steps + 1)
        reference_states = np.repeat(target_state[None, :], horizon_steps, axis=0)
        valid_reference = reference_indices < len(reference_trajectory)
        reference_states[valid_reference] = reference_trajectory[
            reference_indices[valid_reference]
        ]
        nominal_forces = np.zeros(horizon_steps, dtype=np.float64)
        if capture_phase:
            control_indices = np.arange(k, k + horizon_steps)
            valid_controls = control_indices < len(reference_controls)
            nominal_forces[valid_controls] = reference_controls[control_indices[valid_controls]]
        else:
            nominal_state = state.copy()
            for j in range(horizon_steps):
                nominal_error = nominal_state.copy()
                nominal_error[2] = wrap_pi(nominal_state[2] - theta_ref)
                nominal_forces[j] = float(
                    np.clip(-(terminal_gain @ nominal_error), -F_MAX, F_MAX)
                )
                nominal_state = rk4_step(nominal_state, nominal_forces[j], dt)
        u, failed, _ = solve_pnmpc(
            layers,
            y_history,
            u_history,
            u_previous,
            theta_ref,
            last_innovation,
            args,
            reference_states,
            nominal_forces,
            args.capture_control_weight if capture_phase else args.hold_control_weight,
            horizon_steps,
        )
        next_feature = predict_batch(
            layers, y_history[None, :, :], u_history[None, :], np.array([u], dtype=np.float32)
        )[0]
        predicted_state = decode_state(next_feature)
        compute_ms[k] = (time.perf_counter() - started) * 1000.0

        next_state = rk4_step(state, u, dt)
        if abs(next_state[0]) > TRACK_LIMIT:
            next_state[0] = np.sign(next_state[0]) * TRACK_LIMIT
            if next_state[1] * np.sign(next_state[0]) > 0:
                velocity = next_state[1]
                mass, length, inertia = 0.2, 0.3, 0.006
                next_state[3] += mass * length * np.cos(next_state[2]) * velocity / (
                    inertia + mass * length**2
                )
                next_state[1] = 0.0
            rail_hit[k] = True

        actual_feature = encode_state(next_state)
        predicted_next[k] = predicted_state[[0, 2]]
        last_innovation = estimate_innovation(next_state, predicted_state)
        innovations[k] = last_innovation
        y_history = np.concatenate((actual_feature[None, :], y_history[:OUTPUT_DELAY]), axis=0)
        u_history = np.concatenate(([u], u_history[:OUTPUT_DELAY - 1])).astype(np.float32)
        state = next_state
        states[k + 1] = state
        forces[k] = u
        solver_failed[k] = failed
        u_previous = u

    return {
        "time": times,
        "states": states,
        "force": forces,
        "predicted_next": predicted_next,
        "innovation": innovations,
        "solver_failed": solver_failed,
        "compute_ms": compute_ms,
        "rail_hit": rail_hit,
        "theta_ref": theta_ref,
        "sensor_offset": sensor_offset,
        "controller_strategy": (
            "PNMPC-ESNI receding-horizon com TDKAN, trajetória nominal e LQR terminal"
        ),
        "prediction_model": "TDKAN com inovação, rastreamento de referência e G_PNMPC local",
        "capture_reference": "trajetória nominal RK4; estabilização terminal LQR nominal",
        "trajectory_optimization_seconds": optimization_seconds,
        "planned_capture_horizon_seconds": plan_horizon,
    }


def _rollout_control_knots(control_knots, initial_state, dt, steps_per_knot, horizon_steps):
    state = np.asarray(initial_state, dtype=np.float64).copy()
    states = []
    rail_penalty = 0.0
    controls = np.repeat(control_knots, steps_per_knot)[:horizon_steps]
    for force in controls:
        state = rk4_step(state, float(np.clip(force, -F_MAX, F_MAX)), dt)
        excess = max(0.0, abs(float(state[0])) - 0.23)
        rail_penalty += 18000.0 * excess**2
        states.append(state.copy())
    return np.asarray(states), rail_penalty


def _trajectory_objective(
    control_knots,
    initial_state,
    target_theta,
    dt,
    steps_per_knot,
    horizon_steps,
    stage_weights,
    terminal_weights,
    force_weight,
):
    trajectory, rail_penalty = _rollout_control_knots(
        control_knots, initial_state, dt, steps_per_knot, horizon_steps
    )
    normalized = trajectory / np.array([0.30, 1.0, 3.0, 5.0])
    stage_cost = np.sum(normalized**2 * stage_weights[None, :]) * dt
    terminal_error = np.array(
        [
            trajectory[-1, 0] / 0.30,
            wrap_pi(trajectory[-1, 2] - target_theta),
            trajectory[-1, 1] / 3.0,
            trajectory[-1, 3] / 5.0,
        ]
    )
    increments = np.diff(np.r_[0.0, control_knots])
    return float(
        stage_cost
        + np.dot(terminal_error**2, terminal_weights)
        + force_weight * np.sum(control_knots**2)
        + 0.02 * np.sum(increments**2)
        + rail_penalty
    )


def optimize_nonlinear_trajectory(initial_state, target_theta, dt, seed):
    horizon = 5.0
    coarse_interval = 0.5
    control_interval = 0.1
    horizon_steps = int(round(horizon / dt))
    coarse_steps = int(round(coarse_interval / dt))
    control_steps = int(round(control_interval / dt))
    if not np.isclose(coarse_steps * dt, coarse_interval) or not np.isclose(
        control_steps * dt, control_interval
    ):
        raise ValueError("O passo de integração deve dividir 0,1 s exatamente.")

    coarse_objective = lambda controls: _trajectory_objective(
        controls,
        initial_state,
        target_theta,
        dt,
        coarse_steps,
        horizon_steps,
        np.array([1.0, 0.06, 0.03, 0.03]),
        np.array([200.0, 3000.0, 300.0, 2500.0]),
        0.002,
    )
    global_result = differential_evolution(
        coarse_objective,
        [(-F_MAX, F_MAX)] * int(round(horizon / coarse_interval)),
        popsize=8,
        maxiter=180,
        tol=0.005,
        polish=False,
        seed=seed + 5,
        updating="immediate",
        workers=1,
    )
    fine_guess = np.repeat(global_result.x, int(round(coarse_interval / control_interval)))
    fine_objective = lambda controls, terminal_weights, force_weight: _trajectory_objective(
        controls,
        initial_state,
        target_theta,
        dt,
        control_steps,
        horizon_steps,
        np.array([0.8, 0.04, 0.02, 0.02]),
        terminal_weights,
        force_weight,
    )
    first_refinement = minimize(
        fine_objective,
        fine_guess,
        args=(np.array([500.0, 8000.0, 5000.0, 8000.0]), 0.0005),
        method="SLSQP",
        bounds=[(-F_MAX, F_MAX)] * len(fine_guess),
        options={"maxiter": 100, "ftol": 1e-5, "eps": 0.03},
    )
    final_refinement = minimize(
        fine_objective,
        first_refinement.x,
        args=(np.array([1500.0, 20000.0, 20000.0, 20000.0]), 0.0002),
        method="SLSQP",
        bounds=[(-F_MAX, F_MAX)] * len(fine_guess),
        options={"maxiter": 250, "ftol": 1e-7, "eps": 0.01},
    )
    plan = np.asarray(final_refinement.x, dtype=np.float64)
    predicted, _ = _rollout_control_knots(
        plan, initial_state, dt, control_steps, horizon_steps
    )
    terminal_angle_deg = abs(np.rad2deg(wrap_pi(predicted[-1, 2] - target_theta)))
    if (
        not np.all(np.isfinite(plan))
        or np.max(np.abs(predicted[:, 0])) >= TRACK_LIMIT
        or terminal_angle_deg > 5.0
        or abs(predicted[-1, 1]) > 0.15
        or abs(predicted[-1, 3]) > 0.35
    ):
        raise RuntimeError(
            "A otimização não encontrou uma trajetória segura de captura: "
            f"erro terminal={terminal_angle_deg:.2f}°, "
            f"x_final={predicted[-1, 0]:.3f} m, "
            f"dx_final={predicted[-1, 1]:.3f} m/s, "
            f"dtheta_final={predicted[-1, 3]:.3f} rad/s."
        )
    return plan, horizon, control_interval


def load_or_optimize_trajectory(args, initial_state, theta_ref):
    os.makedirs(args.output_dir, exist_ok=True)
    cache_path = os.path.join(args.output_dir, "plano_pnmpc_nao_linear.json")
    default_cache_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "resultados_pnmpc_tuning",
        "alcance_90_graus",
        "plano_pnmpc_nao_linear.json",
    )
    cache_key = {
        "dt": float(args.dt),
        "initial_state": np.asarray(initial_state, dtype=np.float64).tolist(),
        "theta_reference_model_rad": float(theta_ref),
    }
    cache_candidates = [default_cache_path]
    if os.path.normcase(default_cache_path) != os.path.normcase(cache_path):
        cache_candidates.append(cache_path)

    for candidate in cache_candidates:
        if not os.path.isfile(candidate):
            continue
        with open(candidate, encoding="utf-8") as file:
            payload = json.load(file)
        if not isinstance(payload, dict):
            raise ValueError(f"Cache inválido: {candidate}")
        if all(
            key in payload
            and np.allclose(payload[key], value, rtol=0.0, atol=1e-6)
            for key, value in cache_key.items()
        ):
            plan = np.asarray(payload.get("control_sequence_n"), dtype=np.float64)
            if (
                plan.shape == (50,)
                and np.all(np.isfinite(plan))
                and "horizon_seconds" in payload
                and "control_interval_seconds" in payload
            ):
                if os.path.normcase(candidate) != os.path.normcase(cache_path):
                    with open(cache_path, "w", encoding="utf-8") as file:
                        json.dump(payload, file, indent=2, ensure_ascii=False)
                return (
                    plan,
                    float(payload["horizon_seconds"]),
                    float(payload["control_interval_seconds"]),
                    0.0,
                )

    started = time.perf_counter()
    plan, horizon, control_interval = optimize_nonlinear_trajectory(
        initial_state, theta_ref, args.dt, args.seed
    )
    optimization_seconds = time.perf_counter() - started
    payload = {
        **cache_key,
        "horizon_seconds": horizon,
        "control_interval_seconds": control_interval,
        "control_sequence_n": plan.tolist(),
        "planner": "SLSQP com busca global diferencial e modelo RK4 não linear",
    }
    with open(cache_path, "w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2, ensure_ascii=False)
    return plan, horizon, control_interval, optimization_seconds


def terminal_lqr_gain(theta_ref, dt):
    equilibrium = np.array([0.0, 0.0, theta_ref, 0.0], dtype=np.float64)
    epsilon = 1e-5
    A = np.column_stack(
        [
            (
                rk4_step(equilibrium + np.eye(4)[i] * epsilon, 0.0, dt)
                - rk4_step(equilibrium - np.eye(4)[i] * epsilon, 0.0, dt)
            )
            / (2.0 * epsilon)
            for i in range(4)
        ]
    )
    B = (
        (rk4_step(equilibrium, epsilon, dt) - rk4_step(equilibrium, -epsilon, dt))
        / (2.0 * epsilon)
    ).reshape(4, 1)
    Q = np.diag([20.0, 3.0, 300.0, 10.0])
    R = np.array([[0.2]])
    P = solve_discrete_are(A, B, Q, R)
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A).reshape(4)


def simulate_nonlinear_mpc(args):
    dt = args.dt
    theta_ref = wrap_pi(np.deg2rad(args.theta_ref) - np.deg2rad(args.sensor_offset_deg))
    if abs(wrap_pi(theta_ref)) > 1e-6:
        raise ValueError("Este ensaio de captura foi sintonizado para o pêndulo vertical (90° no sensor).")
    theta_initial = wrap_pi(np.deg2rad(args.theta_inicial) - np.deg2rad(args.sensor_offset_deg))
    state = np.array([0.0, 0.0, theta_initial, 0.0], dtype=np.float64)
    plan, plan_horizon, control_interval, optimization_seconds = load_or_optimize_trajectory(
        args, state, theta_ref
    )
    steps_per_control = int(round(control_interval / dt))
    plan_steps = len(plan) * steps_per_control
    if not np.isclose(steps_per_control * dt, control_interval):
        raise ValueError("O passo de integração deve dividir o intervalo de controle de 0,1 s.")
    gain = terminal_lqr_gain(theta_ref, dt)

    count = int(round(args.duration / dt))
    times = np.arange(count + 1, dtype=np.float64) * dt
    states = np.empty((count + 1, 4), dtype=np.float64)
    states[0] = state
    forces = np.zeros(count, dtype=np.float64)
    predicted_next = np.zeros((count, 2), dtype=np.float64)
    innovations = np.zeros((count, 4), dtype=np.float64)
    solver_failed = np.zeros(count, dtype=bool)
    compute_ms = np.zeros(count, dtype=np.float64)
    rail_hit = np.zeros(count, dtype=bool)

    for k in range(count):
        started = time.perf_counter()
        if k < plan_steps:
            force = float(plan[k // steps_per_control])
        else:
            error = state.copy()
            error[2] = wrap_pi(state[2] - theta_ref)
            force = float(np.clip(-(gain @ error), -F_MAX, F_MAX))
        predicted_state = rk4_step(state, force, dt)
        compute_ms[k] = (time.perf_counter() - started) * 1000.0

        next_state = predicted_state.copy()
        if abs(next_state[0]) > TRACK_LIMIT:
            next_state[0] = np.sign(next_state[0]) * TRACK_LIMIT
            if next_state[1] * np.sign(next_state[0]) > 0:
                velocity = next_state[1]
                mass, length, inertia = 0.2, 0.3, 0.006
                next_state[3] += mass * length * np.cos(next_state[2]) * velocity / (
                    inertia + mass * length**2
                )
                next_state[1] = 0.0
            rail_hit[k] = True

        predicted_next[k] = predicted_state[[0, 2]]
        innovations[k] = estimate_innovation(next_state, predicted_state)
        state = next_state
        states[k + 1] = state
        forces[k] = force

    return {
        "time": times,
        "states": states,
        "force": forces,
        "predicted_next": predicted_next,
        "innovation": innovations,
        "solver_failed": solver_failed,
        "compute_ms": compute_ms,
        "rail_hit": rail_hit,
        "theta_ref": theta_ref,
        "sensor_offset": np.deg2rad(args.sensor_offset_deg),
        "controller_strategy": "planejamento não linear de 5 s + LQR terminal",
        "prediction_model": "RK4 com as mesmas equações não lineares da planta",
        "trajectory_optimization_seconds": optimization_seconds,
        "planned_capture_horizon_seconds": plan_horizon,
    }


def calculate_metrics(data, args, test_rmse):
    dt = args.dt
    states = data["states"]
    theta_error = wrap_pi(states[:, 2] - data["theta_ref"])
    saturation = np.abs(data["force"]) >= F_MAX - 1e-6
    actual_next = states[1:, [0, 2]]
    prediction_error = data["predicted_next"] - actual_next
    prediction_error[:, 1] = wrap_pi(prediction_error[:, 1])
    hold_start = max(0, len(states) - int(round(3.0 / dt)))
    hold_angles_deg = np.abs(np.rad2deg(theta_error[hold_start:]))
    capture_condition = (
        (np.abs(theta_error) <= np.deg2rad(5.0))
        & (np.abs(states[:, 0]) <= 0.15)
        & (np.abs(states[:, 1]) <= 0.30)
        & (np.abs(states[:, 3]) <= 0.50)
    )
    stable_suffix = np.logical_and.accumulate(capture_condition[::-1])[::-1]
    capture_indices = np.flatnonzero(stable_suffix)
    return {
        "controller_architecture": data.get(
            "controller_strategy", "PNMPC-ESNI com TDKAN e linearização local"
        ),
        "prediction_model": data.get(
            "prediction_model", "TDKAN com atrasos de entrada e saída"
        ),
        "capture_reference": data.get("capture_reference", "sem referência nominal"),
        "kan_test_rmse_normalized": test_rmse,
        "iae_x_m_s": float(np.sum(np.abs(states[:, 0])) * dt),
        "iae_theta_deg_s": float(np.sum(np.abs(np.rad2deg(theta_error))) * dt),
        "final_x_m": float(states[-1, 0]),
        "final_theta_error_deg": float(np.rad2deg(theta_error[-1])),
        "rms_force_n": float(np.sqrt(np.mean(data["force"] ** 2))),
        "force_saturation_fraction": float(np.mean(saturation)),
        "rail_contact_count": int(np.count_nonzero(data["rail_hit"])),
        "one_step_rmse_x_m": float(np.sqrt(np.mean(prediction_error[:, 0] ** 2))),
        "one_step_rmse_theta_deg": float(np.rad2deg(np.sqrt(np.mean(prediction_error[:, 1] ** 2)))),
        "innovation_rmse_x_m": float(np.sqrt(np.mean(data["innovation"][:, 0] ** 2))),
        "innovation_rmse_theta_deg": float(
            np.rad2deg(np.sqrt(np.mean(data["innovation"][:, 1] ** 2)))
        ),
        "innovation_rmse_dx_m_s": float(np.sqrt(np.mean(data["innovation"][:, 2] ** 2))),
        "innovation_rmse_dtheta_rad_s": float(
            np.sqrt(np.mean(data["innovation"][:, 3] ** 2))
        ),
        "controller_compute_mean_ms": float(np.mean(data["compute_ms"])),
        "controller_compute_max_ms": float(np.max(data["compute_ms"])),
        "optimizer_fallback_count": int(np.count_nonzero(data["solver_failed"])),
        "peak_abs_x_m": float(np.max(np.abs(states[:, 0]))),
        "trajectory_optimization_seconds": float(data.get("trajectory_optimization_seconds", 0.0)),
        "planned_capture_horizon_seconds": float(
            data.get("planned_capture_horizon_seconds", 0.0)
        ),
        "capture_and_hold_time_seconds": (
            float(data["time"][capture_indices[0]]) if len(capture_indices) else None
        ),
        "last_3s_mean_abs_angle_error_deg": float(np.mean(hold_angles_deg)),
        "last_3s_max_abs_angle_error_deg": float(np.max(hold_angles_deg)),
        "last_3s_peak_abs_cart_position_m": float(
            np.max(np.abs(states[hold_start:, 0]))
        ),
        "x_weight": float(args.x_weight),
        "theta_weight": float(args.theta_weight),
        "dx_weight": float(args.dx_weight),
        "dtheta_weight": float(args.dtheta_weight),
        "terminal_weight": float(args.terminal_weight),
        "control_weight": float(args.control_weight),
        "hold_control_weight": float(args.hold_control_weight),
        "capture_control_weight": float(args.capture_control_weight),
        "force_weight": float(args.force_weight),
        "sensor_offset_deg": float(args.sensor_offset_deg),
        "theta_reference_model_deg": float(np.rad2deg(data["theta_ref"])),
        "simulation_seconds": float(args.duration),
        "sample_time_seconds": float(dt),
        "gpu_used": bool(DEVICE.type == "cuda"),
    }


def save_results(data, metrics, args):
    os.makedirs(args.output_dir, exist_ok=True)
    result_name = (
        "pnmpc_nao_linear"
        if args.controller == "nonlinear-mpc"
        else "pnmpc_esni_kan"
    )
    time_axis = data["time"]
    states = data["states"]
    theta_error = np.rad2deg(wrap_pi(states[:, 2] - data["theta_ref"]))

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(time_axis, states[:, 0], label="x do carrinho")
    axes[0].axhline(0.0, color="black", linestyle="--", linewidth=1, label="referência")
    axes[0].axhline(TRACK_LIMIT, color="red", linestyle=":", linewidth=1, label="fim de curso")
    axes[0].axhline(-TRACK_LIMIT, color="red", linestyle=":", linewidth=1)
    axes[0].set_ylabel("Posição [m]")
    axes[0].grid(True)
    axes[0].legend(loc="best")
    axes[1].plot(time_axis, theta_error, label="erro angular")
    axes[1].axhline(0.0, color="black", linestyle="--", linewidth=1, label="referência")
    axes[1].set_ylabel("Erro angular relativo ao alvo [graus]")
    axes[1].set_xlabel("Tempo [s]")
    axes[1].grid(True)
    axes[1].legend(loc="best")
    controller_label = (
        "Planejamento não linear + LQR terminal"
        if args.controller == "nonlinear-mpc"
        else "PNMPC-ESNI KAN + referência nominal/LQR terminal"
    )
    fig.suptitle(
        f"{controller_label} — sensor {args.theta_inicial:g}° → {args.theta_ref:g}° "
        f"(offset {args.sensor_offset_deg:g}°)"
    )
    fig.tight_layout()
    fig.savefig(os.path.join(args.output_dir, f"controle_{result_name}.png"), dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].step(time_axis[:-1], data["force"], where="post", label="força aplicada")
    axes[0].axhline(F_MAX, color="red", linestyle=":")
    axes[0].axhline(-F_MAX, color="red", linestyle=":")
    axes[0].set_ylabel("Força [N]")
    axes[0].grid(True)
    axes[0].legend(loc="best")
    axes[1].plot(time_axis[:-1], data["compute_ms"], label="tempo por amostra")
    axes[1].axhline(1000.0 * args.dt, color="red", linestyle="--", label="Ts")
    axes[1].set_ylabel("Cálculo [ms]")
    axes[1].set_xlabel("Tempo [s]")
    axes[1].grid(True)
    axes[1].legend(loc="best")
    fig.suptitle("Ação de controle e custo computacional")
    fig.tight_layout()
    fig.savefig(os.path.join(args.output_dir, f"atuacao_{result_name}.png"), dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(time_axis[1:], states[1:, 0], label="x real")
    axes[0].plot(time_axis[1:], data["predicted_next"][:, 0], "--", label="x previsto 1 passo")
    axes[0].set_ylabel("Posição [m]")
    axes[0].grid(True)
    axes[0].legend(loc="best")
    actual_theta = states[1:, 2]
    predicted_theta = data["predicted_next"][:, 1]
    axes[1].plot(time_axis[1:], np.rad2deg(wrap_pi(actual_theta - data["theta_ref"])), label="erro real")
    axes[1].plot(
        time_axis[1:],
        np.rad2deg(wrap_pi(predicted_theta - data["theta_ref"])),
        "--",
        label="erro previsto 1 passo",
    )
    axes[1].set_ylabel("Erro angular relativo ao alvo [graus]")
    axes[1].set_xlabel("Tempo [s]")
    axes[1].grid(True)
    axes[1].legend(loc="best")
    fig.suptitle("Predição do modelo versus planta simulada")
    fig.tight_layout()
    fig.savefig(os.path.join(args.output_dir, f"predicao_{result_name}.png"), dpi=160)
    plt.close(fig)

    csv_path = os.path.join(args.output_dir, f"serie_temporal_{result_name}.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "tempo_s", "x_m", "dx_m_s", "theta_sensor_deg", "dtheta_rad_s",
                "u_N", "x_previsto_1passo_m", "theta_previsto_1passo_deg",
                "residuo_x_m", "residuo_theta_deg", "residuo_dx_m_s",
                "residuo_dtheta_rad_s", "tempo_calculo_ms", "toque_fim_curso",
                "falha_otimizador",
            ]
        )
        for k, t in enumerate(time_axis):
            if k == len(data["force"]):
                writer.writerow(
                    [
                        t,
                        states[k, 0],
                        states[k, 1],
                        np.rad2deg((states[k, 2] + data["sensor_offset"]) % (2.0 * np.pi)),
                        states[k, 3],
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                    ]
                )
                continue
            writer.writerow(
                [
                    t,
                    states[k, 0],
                    states[k, 1],
                    np.rad2deg((states[k, 2] + data["sensor_offset"]) % (2.0 * np.pi)),
                    states[k, 3],
                    data["force"][k],
                    data["predicted_next"][k, 0],
                    np.rad2deg(
                        (data["predicted_next"][k, 1] + data["sensor_offset"]) % (2.0 * np.pi)
                    ),
                    data["innovation"][k, 0],
                    np.rad2deg(data["innovation"][k, 1]),
                    data["innovation"][k, 2],
                    data["innovation"][k, 3],
                    data["compute_ms"][k],
                    int(data["rail_hit"][k]),
                    int(data["solver_failed"][k]),
                ]
            )
    with open(
        os.path.join(args.output_dir, f"metricas_{result_name}.json"),
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(metrics, file, indent=2, ensure_ascii=False)
    return csv_path


def parse_args():
    parser = argparse.ArgumentParser(description="Testa PNMPC-TDKAN no pêndulo simulado.")
    parser.add_argument(
        "--controller",
        choices=("nonlinear-mpc", "tdkan"),
        default="tdkan",
        help=(
            "tdkan executa PNMPC-ESNI receding-horizon com KAN temporal no lugar do ESN; "
            "nonlinear-mpc executa o planejador offline + LQR, apenas como referência."
        ),
    )
    parser.add_argument("--theta-inicial", type=float, default=210.0)
    parser.add_argument("--theta-ref", type=float, default=90.0)
    parser.add_argument(
        "--sensor-offset-deg",
        type=float,
        default=90.0,
        help="Offset aditivo: ângulo_sensor = ângulo_modelo + offset.",
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--prediction-horizon", type=int, default=10)
    parser.add_argument(
        "--hold-prediction-horizon",
        type=int,
        default=10,
        help="Horizonte usado após a captura, quando o pêndulo está perto do equilíbrio.",
    )
    parser.add_argument("--control-horizon", type=int, default=3)
    parser.add_argument("--samples", type=int, default=700)
    parser.add_argument("--training-steps", type=int, default=10)
    parser.add_argument("--step-amplitude", type=float, default=0.5)
    parser.add_argument("--control-weight", type=float, default=0.10)
    parser.add_argument(
        "--hold-control-weight",
        type=float,
        default=10.0,
        help="Penaliza desvios da ação LQR nominal durante a manutenção do equilíbrio.",
    )
    parser.add_argument(
        "--capture-control-weight",
        type=float,
        default=10000.0,
        help="Penaliza desvios da força nominal durante a trajetória de captura.",
    )
    parser.add_argument("--force-weight", type=float, default=0.0)
    parser.add_argument("--x-weight", type=float, default=3.0)
    parser.add_argument("--theta-weight", type=float, default=3.0)
    parser.add_argument("--dx-weight", type=float, default=2.0)
    parser.add_argument("--dtheta-weight", type=float, default=4.0)
    parser.add_argument("--terminal-weight", type=float, default=4.0)
    parser.add_argument("--max-delta-u", type=float, default=3.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--output-dir",
        default=os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "resultados_pnmpc_tuning",
            "arquitetura_pnmpc_kan",
        ),
    )
    parser.add_argument("--retrain", action="store_true", help="Treina novamente, ignorando o checkpoint.")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.controller == "tdkan" and (
        args.control_horizon > args.prediction_horizon
        or args.control_horizon > args.hold_prediction_horizon
    ):
        raise ValueError("O horizonte de controle não pode exceder os horizontes de predição.")
    if min(
        args.x_weight,
        args.theta_weight,
        args.dx_weight,
        args.dtheta_weight,
        args.terminal_weight,
        args.control_weight,
        args.hold_control_weight,
        args.capture_control_weight,
    ) <= 0 or args.force_weight < 0:
        raise ValueError("Pesos de estado, terminal e incremento devem ser positivos; force-weight não negativo.")
    if args.controller == "tdkan":
        print(f"Dispositivo de treinamento e inferência: {DEVICE}")
        model, test_rmse = train_or_load_model(args)
        layers = extrair_camadas(model)
        probe, _ = collect_training_data(4, args.seed + 2)
        with torch.no_grad():
            torch_probe = model(
                torch.as_tensor(probe, dtype=torch.float32, device=DEVICE)
            ).cpu().numpy()
        numpy_probe = evaluate_kan_batch(layers, probe)
        reconstruction_error = float(np.max(np.abs(torch_probe - numpy_probe)))
        if reconstruction_error > 1e-4:
            raise RuntimeError(
                f"Reconstrução NumPy da KAN divergente do pykan: {reconstruction_error:.3e}"
            )
        print(f"Erro máximo da inferência rápida versus pykan: {reconstruction_error:.2e}")
        data = simulate_tdkan(layers, args)
    else:
        test_rmse = None
        data = simulate_nonlinear_mpc(args)
    metrics = calculate_metrics(data, args, test_rmse)
    csv_path = save_results(data, metrics, args)
    print(f"\n=== Métricas do ensaio {metrics['controller_architecture']} (simulação, não hardware) ===")
    for name, value in metrics.items():
        print(f"{name:34s} {value}")
    print(f"\nGráficos e dados: {os.path.abspath(args.output_dir)}")
    print(f"Série temporal: {csv_path}")


if __name__ == "__main__":
    main()
