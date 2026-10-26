#include "kinematics/arm_kinematics.h"

#include <math.h>
#include <stddef.h>

#define ARM_KINEMATICS_PI 3.14159265358979323846f
#define ARM_KINEMATICS_DEG_TO_RAD (ARM_KINEMATICS_PI / 180.0f)
#define ARM_KINEMATICS_RAD_TO_DEG (180.0f / ARM_KINEMATICS_PI)

#define ARM_KINEMATICS_BASE_Z_MM 65.0f
#define ARM_KINEMATICS_LINK_1_MM 101.0f
#define ARM_KINEMATICS_LINK_2_MM 95.0f
#define ARM_KINEMATICS_LINK_3_MM 168.0f
#define ARM_KINEMATICS_TOTAL_REACH_MM \
	(ARM_KINEMATICS_LINK_1_MM + ARM_KINEMATICS_LINK_2_MM + ARM_KINEMATICS_LINK_3_MM)
#define ARM_KINEMATICS_GROUND_Z_MM 0.0f

#define ARM_KINEMATICS_YAW_MIN_RAD (-120.0f * ARM_KINEMATICS_DEG_TO_RAD)
#define ARM_KINEMATICS_YAW_MAX_RAD (120.0f * ARM_KINEMATICS_DEG_TO_RAD)
#define ARM_KINEMATICS_SHOULDER_MIN_RAD (-90.0f * ARM_KINEMATICS_DEG_TO_RAD)
#define ARM_KINEMATICS_SHOULDER_MAX_RAD (90.0f * ARM_KINEMATICS_DEG_TO_RAD)
#define ARM_KINEMATICS_ELBOW_MIN_RAD (-120.0f * ARM_KINEMATICS_DEG_TO_RAD)
#define ARM_KINEMATICS_ELBOW_MAX_RAD (120.0f * ARM_KINEMATICS_DEG_TO_RAD)

#define ARM_KINEMATICS_IK_MAX_ITERATIONS 36u
#define ARM_KINEMATICS_IK_TOLERANCE_MM 3.0f
#define ARM_KINEMATICS_IK_NEAR_AXIS_TOLERANCE_MM 25.0f
#define ARM_KINEMATICS_IK_DAMPING 18.0f
#define ARM_KINEMATICS_IK_MAX_STEP_RAD 0.080f
#define ARM_KINEMATICS_IK_DET_EPSILON 0.000001f

static float clamp_f32(float value, float minimum, float maximum) {
	if (value < minimum) {
		return minimum;
	}
	if (value > maximum) {
		return maximum;
	}
	return value;
}

static float square_f32(float value) {
	return value * value;
}

static float normalize_small(float value) {
	return (fabsf(value) < 0.000001f) ? 0.0f : value;
}

static void clamp_pitch_joints(float q_rad[3]) {
	q_rad[0] = clamp_f32(q_rad[0], ARM_KINEMATICS_SHOULDER_MIN_RAD, ARM_KINEMATICS_SHOULDER_MAX_RAD);
	q_rad[1] = clamp_f32(q_rad[1], ARM_KINEMATICS_ELBOW_MIN_RAD, ARM_KINEMATICS_ELBOW_MAX_RAD);
	q_rad[2] = clamp_f32(q_rad[2], ARM_KINEMATICS_ELBOW_MIN_RAD, ARM_KINEMATICS_ELBOW_MAX_RAD);
}

static void seed_pitch_joints_closed_form(float target_radial_mm,
                                          float target_z_relative_mm,
                                          float q_rad[3]) {
	const float combined_forearm_mm = ARM_KINEMATICS_LINK_2_MM + ARM_KINEMATICS_LINK_3_MM;
	const float distance_squared = square_f32(target_radial_mm) + square_f32(target_z_relative_mm);
	const float cosine_elbow = clamp_f32(
		(distance_squared - square_f32(ARM_KINEMATICS_LINK_1_MM) - square_f32(combined_forearm_mm)) /
			(2.0f * ARM_KINEMATICS_LINK_1_MM * combined_forearm_mm),
		-1.0f,
		1.0f);
	const float elbow_rad = acosf(cosine_elbow);
	const float target_angle_rad = atan2f(target_radial_mm, target_z_relative_mm);
	const float shoulder_offset_rad = atan2f(combined_forearm_mm * sinf(elbow_rad),
	                                      ARM_KINEMATICS_LINK_1_MM + combined_forearm_mm * cosf(elbow_rad));

	q_rad[0] = target_angle_rad - shoulder_offset_rad;
	q_rad[1] = elbow_rad;
	q_rad[2] = 0.0f;
	clamp_pitch_joints(q_rad);
}

static float solution_joint_rad(const arm_kinematics_solution_t* solution, size_t index) {
	if (solution == NULL || index >= ARM_KINEMATICS_DOF) {
		return 0.0f;
	}
	return solution->joint_deg[index] * ARM_KINEMATICS_DEG_TO_RAD;
}

static void pitch_forward(float shoulder_rad,
                          float elbow1_rad,
                          float elbow2_rad,
                          float* radial_mm,
                          float* z_relative_mm) {
	const float pitch1 = (ARM_KINEMATICS_PI * 0.5f) - shoulder_rad;
	const float pitch2 = pitch1 - elbow1_rad;
	const float pitch3 = pitch2 - elbow2_rad;

	if (radial_mm != NULL) {
		*radial_mm =
			ARM_KINEMATICS_LINK_1_MM * cosf(pitch1) +
			ARM_KINEMATICS_LINK_2_MM * cosf(pitch2) +
			ARM_KINEMATICS_LINK_3_MM * cosf(pitch3);
	}

	if (z_relative_mm != NULL) {
		*z_relative_mm =
			ARM_KINEMATICS_LINK_1_MM * sinf(pitch1) +
			ARM_KINEMATICS_LINK_2_MM * sinf(pitch2) +
			ARM_KINEMATICS_LINK_3_MM * sinf(pitch3);
	}
}

static void pitch_jacobian(float shoulder_rad,
                           float elbow1_rad,
                           float elbow2_rad,
                           float jacobian[2][3]) {
	const float pitch1 = (ARM_KINEMATICS_PI * 0.5f) - shoulder_rad;
	const float pitch2 = pitch1 - elbow1_rad;
	const float pitch3 = pitch2 - elbow2_rad;

	jacobian[0][0] =
		ARM_KINEMATICS_LINK_1_MM * sinf(pitch1) +
		ARM_KINEMATICS_LINK_2_MM * sinf(pitch2) +
		ARM_KINEMATICS_LINK_3_MM * sinf(pitch3);
	jacobian[0][1] =
		ARM_KINEMATICS_LINK_2_MM * sinf(pitch2) +
		ARM_KINEMATICS_LINK_3_MM * sinf(pitch3);
	jacobian[0][2] = ARM_KINEMATICS_LINK_3_MM * sinf(pitch3);

	jacobian[1][0] =
		-ARM_KINEMATICS_LINK_1_MM * cosf(pitch1) -
		ARM_KINEMATICS_LINK_2_MM * cosf(pitch2) -
		ARM_KINEMATICS_LINK_3_MM * cosf(pitch3);
	jacobian[1][1] =
		-ARM_KINEMATICS_LINK_2_MM * cosf(pitch2) -
		ARM_KINEMATICS_LINK_3_MM * cosf(pitch3);
	jacobian[1][2] = -ARM_KINEMATICS_LINK_3_MM * cosf(pitch3);
}

static void clamp_cartesian_target(const arm_kinematics_cartesian_t* target,
                                   arm_kinematics_cartesian_t* clamped_target) {
	float x_mm = 0.0f;
	float y_mm = 0.0f;
	float z_mm = ARM_KINEMATICS_BASE_Z_MM + ARM_KINEMATICS_TOTAL_REACH_MM;

	if (target != NULL) {
		x_mm = target->x_mm;
		y_mm = target->y_mm;
		z_mm = target->z_mm;
	}

	if (z_mm < ARM_KINEMATICS_GROUND_Z_MM) {
		z_mm = ARM_KINEMATICS_GROUND_Z_MM;
	}

	float radial_mm = sqrtf(square_f32(x_mm) + square_f32(y_mm));
	float z_relative_mm = z_mm - ARM_KINEMATICS_BASE_Z_MM;
	const float shoulder_distance_mm = sqrtf(square_f32(radial_mm) + square_f32(z_relative_mm));

	if (shoulder_distance_mm > ARM_KINEMATICS_TOTAL_REACH_MM && shoulder_distance_mm > 0.0f) {
		const float scale = ARM_KINEMATICS_TOTAL_REACH_MM / shoulder_distance_mm;
		x_mm *= scale;
		y_mm *= scale;
		z_relative_mm *= scale;
		z_mm = ARM_KINEMATICS_BASE_Z_MM + z_relative_mm;
		radial_mm *= scale;
	}

	if (clamped_target != NULL) {
		clamped_target->x_mm = normalize_small(x_mm);
		clamped_target->y_mm = normalize_small(y_mm);
		clamped_target->z_mm = normalize_small(z_mm);
	}
}

void arm_kinematics_home_solution(arm_kinematics_solution_t* solution) {
	if (solution == NULL) {
		return;
	}

	for (size_t index = 0; index < ARM_KINEMATICS_DOF; ++index) {
		solution->joint_deg[index] = 0.0f;
	}
}

void arm_kinematics_forward(const arm_kinematics_solution_t* solution,
                            arm_kinematics_cartesian_t* tip_position) {
	if (tip_position == NULL) {
		return;
	}

	const float yaw_rad = solution_joint_rad(solution, 0u);
	const float shoulder_rad = solution_joint_rad(solution, 1u);
	const float elbow1_rad = solution_joint_rad(solution, 2u);
	const float elbow2_rad = solution_joint_rad(solution, 3u);
	float radial_mm = 0.0f;
	float z_relative_mm = 0.0f;

	pitch_forward(shoulder_rad, elbow1_rad, elbow2_rad, &radial_mm, &z_relative_mm);

	tip_position->x_mm = normalize_small(radial_mm * cosf(yaw_rad));
	tip_position->y_mm = normalize_small(radial_mm * sinf(yaw_rad));
	tip_position->z_mm = normalize_small(ARM_KINEMATICS_BASE_Z_MM + z_relative_mm);
}

bool arm_kinematics_inverse(const arm_kinematics_cartesian_t* target,
                            const arm_kinematics_solution_t* seed,
                            arm_kinematics_solution_t* solution,
                            arm_kinematics_cartesian_t* clamped_target) {
	if (solution == NULL) {
		return false;
	}

	arm_kinematics_cartesian_t target_mm;
	clamp_cartesian_target(target, &target_mm);
	if (clamped_target != NULL) {
		*clamped_target = target_mm;
	}

	float yaw_rad = atan2f(target_mm.y_mm, target_mm.x_mm);
	yaw_rad = clamp_f32(yaw_rad, ARM_KINEMATICS_YAW_MIN_RAD, ARM_KINEMATICS_YAW_MAX_RAD);

	const float target_radial_mm = sqrtf(square_f32(target_mm.x_mm) + square_f32(target_mm.y_mm));
	const float target_z_relative_mm = target_mm.z_mm - ARM_KINEMATICS_BASE_Z_MM;

	float q_rad[3] = {
		solution_joint_rad(seed, 1u),
		solution_joint_rad(seed, 2u),
		solution_joint_rad(seed, 3u),
	};
	clamp_pitch_joints(q_rad);

	if (fabsf(q_rad[0]) < 0.001f &&
	    fabsf(q_rad[1]) < 0.001f &&
	    fabsf(q_rad[2]) < 0.001f &&
	    target_z_relative_mm < (ARM_KINEMATICS_TOTAL_REACH_MM - ARM_KINEMATICS_IK_TOLERANCE_MM)) {
		seed_pitch_joints_closed_form(target_radial_mm, target_z_relative_mm, q_rad);
	}

	float final_error_mm = 0.0f;

	for (size_t iteration = 0; iteration < ARM_KINEMATICS_IK_MAX_ITERATIONS; ++iteration) {
		float current_radial_mm = 0.0f;
		float current_z_relative_mm = 0.0f;
		float jacobian[2][3];

		pitch_forward(q_rad[0], q_rad[1], q_rad[2], &current_radial_mm, &current_z_relative_mm);

		const float error_radial_mm = target_radial_mm - current_radial_mm;
		const float error_z_mm = target_z_relative_mm - current_z_relative_mm;
		final_error_mm = sqrtf(square_f32(error_radial_mm) + square_f32(error_z_mm));
		if (final_error_mm <= ARM_KINEMATICS_IK_TOLERANCE_MM) {
			break;
		}

		pitch_jacobian(q_rad[0], q_rad[1], q_rad[2], jacobian);

		const float damping_squared = square_f32(ARM_KINEMATICS_IK_DAMPING);
		const float a00 =
			square_f32(jacobian[0][0]) + square_f32(jacobian[0][1]) + square_f32(jacobian[0][2]) +
			damping_squared;
		const float a01 =
			jacobian[0][0] * jacobian[1][0] +
			jacobian[0][1] * jacobian[1][1] +
			jacobian[0][2] * jacobian[1][2];
		const float a11 =
			square_f32(jacobian[1][0]) + square_f32(jacobian[1][1]) + square_f32(jacobian[1][2]) +
			damping_squared;
		const float determinant = (a00 * a11) - (a01 * a01);

		if (fabsf(determinant) < ARM_KINEMATICS_IK_DET_EPSILON) {
			break;
		}

		const float inv00 = a11 / determinant;
		const float inv01 = -a01 / determinant;
		const float inv11 = a00 / determinant;
		const float resolved_error0 = (inv00 * error_radial_mm) + (inv01 * error_z_mm);
		const float resolved_error1 = (inv01 * error_radial_mm) + (inv11 * error_z_mm);

		for (size_t joint_index = 0; joint_index < 3u; ++joint_index) {
			float delta_rad =
				(jacobian[0][joint_index] * resolved_error0) +
				(jacobian[1][joint_index] * resolved_error1);
			delta_rad = clamp_f32(delta_rad, -ARM_KINEMATICS_IK_MAX_STEP_RAD, ARM_KINEMATICS_IK_MAX_STEP_RAD);
			q_rad[joint_index] += delta_rad;
		}

		clamp_pitch_joints(q_rad);
	}

	solution->joint_deg[0] = yaw_rad * ARM_KINEMATICS_RAD_TO_DEG;
	solution->joint_deg[1] = q_rad[0] * ARM_KINEMATICS_RAD_TO_DEG;
	solution->joint_deg[2] = q_rad[1] * ARM_KINEMATICS_RAD_TO_DEG;
	solution->joint_deg[3] = q_rad[2] * ARM_KINEMATICS_RAD_TO_DEG;

	float final_radial_mm = 0.0f;
	float final_z_relative_mm = 0.0f;
	pitch_forward(q_rad[0], q_rad[1], q_rad[2], &final_radial_mm, &final_z_relative_mm);
	const float final_radial_error_mm = target_radial_mm - final_radial_mm;
	const float final_z_error_mm = target_z_relative_mm - final_z_relative_mm;
	final_error_mm = sqrtf(square_f32(final_radial_error_mm) + square_f32(final_z_error_mm));

	if (target_radial_mm <= ARM_KINEMATICS_IK_NEAR_AXIS_TOLERANCE_MM &&
	    fabsf(final_radial_error_mm) <= ARM_KINEMATICS_IK_NEAR_AXIS_TOLERANCE_MM &&
	    fabsf(final_z_error_mm) <= ARM_KINEMATICS_IK_TOLERANCE_MM) {
		return true;
	}

	return final_error_mm <= ARM_KINEMATICS_IK_TOLERANCE_MM;
}
