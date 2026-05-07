#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_KINEMATICS_DOF 4u

typedef struct arm_kinematics_cartesian {
	float x_mm;
	float y_mm;
	float z_mm;
} arm_kinematics_cartesian_t;

typedef struct arm_kinematics_solution {
	float joint_deg[ARM_KINEMATICS_DOF];
} arm_kinematics_solution_t;

void arm_kinematics_home_solution(arm_kinematics_solution_t* solution);
void arm_kinematics_forward(const arm_kinematics_solution_t* solution,
                            arm_kinematics_cartesian_t* tip_position);
bool arm_kinematics_inverse(const arm_kinematics_cartesian_t* target,
                            const arm_kinematics_solution_t* seed,
                            arm_kinematics_solution_t* solution,
                            arm_kinematics_cartesian_t* clamped_target);

#ifdef __cplusplus
}
#endif

#endif
