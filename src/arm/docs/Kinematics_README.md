# Cartesian Kinematics Module

This directory contains the firmware-side kinematics implementation used by Cartesian XYZ control mode.

The module is intentionally small and self-contained. It does not know about BLE, UART, servo IDs, controller-board packets, or real servo pulse positions. Its job is only to convert between the simulation's 4-DOF logical positioning chain and a Cartesian tip position in millimeters.

## Coordinate Frame

The coordinate system matches `sim/README.md` and `sim/model/arm_model.py`:

- Origin: ground reference at `(0, 0, 0)` mm.
- XY plane: floor plane.
- Z axis: positive upward.
- Shoulder/base joint position: `(0, 0, 65)` mm.
- Home pose `[0, 0, 0, 0]` points straight up and places the tip at `(0, 0, 429)` mm.

The 4-DOF chain used for position control is:

1. Sim Joint 1: base yaw about world +Z.
2. Sim Joint 2: shoulder pitch.
3. Sim Joint 3: first elbow pitch.
4. Sim Joint 4: second elbow pitch.

The pitch joints all live in the yaw-rotated vertical plane. Positive pitch follows the simulation convention: shoulder positive moves from vertical toward forward +X, and positive elbow angles bend forward in the same plane.

## Link Model

The link dimensions are the simulation defaults:

| Parameter | Value |
| --- | ---: |
| Shoulder height | `65 mm` |
| Link 1 | `101 mm` |
| Link 2 | `95 mm` |
| Link 3 | `168 mm` |
| Total reach | `364 mm` |

The wrist and gripper are excluded from XYZ math. Link 3 is treated as the rigid final positioning link, as in the simulation.

## Real Joint Mapping

The IK solution is returned in simulation joint order, then `main.c` maps it into the real arm's logical target registers:

| Simulation joint | Meaning | Real joint | Target register |
| --- | --- | ---: | --- |
| Sim J1 | Base yaw | Joint 6 | `g_joint_target_deci_deg[5]` |
| Sim J2 | Shoulder pitch | Joint 5 | `g_joint_target_deci_deg[4]` |
| Sim J3 | Elbow 1 | Joint 4 | `g_joint_target_deci_deg[3]` |
| Sim J4 | Elbow 2 | Joint 3 | `g_joint_target_deci_deg[2]` |

Real Joints 1 and 2 are wrist/gripper joints. Cartesian mode leaves them manually controlled through the existing joint-target packet.

The kinematics module outputs logical degrees. Hardware-specific polarity and offset remain in the existing joint transform table in `main.c`, so the same clamp/direction/deadband/UART pipeline is used in both control modes.

## Forward Kinematics

The module uses the same equations as `ThreeLinkArm.get_all_joint_positions()` in the Python simulation.

Let:

- `theta0` = base yaw.
- `theta1` = shoulder pitch.
- `theta2` = elbow 1 pitch.
- `theta3` = elbow 2 pitch.
- `L1`, `L2`, `L3` = link lengths.

The pitch angles of each link relative to the horizontal plane are:

```text
pitch1 = pi/2 - theta1
pitch2 = pitch1 - theta2
pitch3 = pitch2 - theta3
```

The yaw-plane radial distance and vertical offset from the shoulder are:

```text
r = L1*cos(pitch1) + L2*cos(pitch2) + L3*cos(pitch3)
z = L1*sin(pitch1) + L2*sin(pitch2) + L3*sin(pitch3)
```

World coordinates are then:

```text
x = r*cos(theta0)
y = r*sin(theta0)
z_world = shoulder_z + z
```

This keeps the home pose vertical and makes increasing X/Y/Z match the documented simulation frame.

## Inverse Kinematics

IK is split into yaw and planar pitch solve:

1. Compute desired yaw with `atan2(y, x)` and clamp it to +/-120 degrees.
2. Reduce the XYZ target to the yaw-aligned pitch plane using radial distance `sqrt(x^2 + y^2)` and `z - shoulder_z`.
3. Solve the three pitch joints iteratively with damped least squares.

The planar error vector is:

```text
error = [target_r - current_r,
         target_z - current_z]
```

The 2x3 Jacobian maps small shoulder/elbow changes to radial and vertical tip motion:

```text
d(r)/d(theta1) = L1*sin(pitch1) + L2*sin(pitch2) + L3*sin(pitch3)
d(r)/d(theta2) =                 L2*sin(pitch2) + L3*sin(pitch3)
d(r)/d(theta3) =                                  L3*sin(pitch3)

d(z)/d(theta1) = -L1*cos(pitch1) - L2*cos(pitch2) - L3*cos(pitch3)
d(z)/d(theta2) =                   -L2*cos(pitch2) - L3*cos(pitch3)
d(z)/d(theta3) =                                      -L3*cos(pitch3)
```

The damped least-squares step is:

```text
delta = J^T * inverse(J*J^T + lambda^2*I) * error
```

The implementation uses a 2x2 inverse for `J*J^T + lambda^2*I`, which avoids a general matrix library. Each per-iteration joint step is capped before applying joint clamps. This limits sudden changes when the target is moved quickly or when the arm is near a singular posture.

When solving from the vertical home pose, the module uses a closed-form two-link seed that treats Links 2 and 3 as one temporary combined forearm. This gives the iterative solver a useful starting bend for forward/downward or vertical-down targets instead of starting exactly on the singular straight-up posture.

## Joint Limits

The firmware IK uses simulation limits:

| Joint | Limit |
| --- | ---: |
| Base yaw | `-120 deg` to `+120 deg` |
| Shoulder pitch | `-90 deg` to `+90 deg` |
| Elbow 1 | `-120 deg` to `+120 deg` |
| Elbow 2 | `-120 deg` to `+120 deg` |

The downstream real-joint target pipeline still applies the firmware's per-joint logical limits before converting to raw servo positions.

## Unreachable Targets

Before solving, the target is clamped against simple workspace bounds:

- `Z` is clamped to the ground plane (`Z >= 0`).
- Targets farther than the full 364 mm reach from the shoulder are scaled back to the reach sphere.
- Yaw is clamped to the +/-120 degree base-yaw limit.

After those clamps, the solver attempts to converge from the last valid Cartesian solution. If the final error remains above tolerance, `arm_kinematics_inverse()` returns `false`. `main.c` then holds the last valid IK pose instead of applying the failed solution. Wrist/gripper target updates still apply while the IK pose is held.

For targets very close to the vertical Z axis, exact radial convergence can be unnecessarily strict because many practical folded poses lower Z with a small radial offset. The solver accepts a near-axis result when Z is within tolerance and radial error remains within a small 25 mm band. This keeps the Z slider responsive from home while still preserving hold-last-valid behavior for broader unreachable targets.

This behavior favors predictable motion over chasing every unreachable slider value. It also prevents jumps at workspace boundaries.

## Runtime State

`main.c` owns the stateful parts:

- Current mode: joint or Cartesian.
- Last valid Cartesian IK solution.
- Last accepted/clamped Cartesian target.
- Whether IK is currently holding after a failed solve.

The kinematics module itself is stateless: each FK/IK call receives all required inputs and writes outputs through caller-provided structs.

## Calibration Notes

The constants in this module match the simulation, not a measured physical calibration. If later testing shows a fixed offset or sign mismatch on real hardware, prefer adjusting the real-joint mapping or joint transform table in `main.c` first. The BLE packet format and the macOS UI should not need to change for calibration.
