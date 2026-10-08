#ifndef THRUST_ALLOCATOR_C_H
#define THRUST_ALLOCATOR_C_H

#include <stddef.h>

/*
 * Static-library API.
 *
 * No __declspec(dllimport/dllexport):
 * the implementation is linked directly from libthrust_allocator.a.
 */
#define THRUST_ALLOCATOR_API

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TA_Handle TA_Handle;

THRUST_ALLOCATOR_API TA_Handle* ta_create(void);
THRUST_ALLOCATOR_API void ta_destroy(TA_Handle* handle);

THRUST_ALLOCATOR_API int ta_add_thruster(
    TA_Handle* handle,
    double px, double py, double pz,
    double ax, double ay, double az,
    double max_thrust,
    double torque_direction,
    double reaction_torque);

THRUST_ALLOCATOR_API int ta_set_wrench_weights(
    TA_Handle* handle,
    const double weights6[6]);

THRUST_ALLOCATOR_API int ta_set_effort_weight(
    TA_Handle* handle,
    double weight);

THRUST_ALLOCATOR_API int ta_initialize(TA_Handle* handle);

THRUST_ALLOCATOR_API int ta_solve_normalized(
    TA_Handle* handle,
    const double desired6[6],
    double* normalized_out,
    size_t out_capacity);

THRUST_ALLOCATOR_API int ta_solve_thrust(
    TA_Handle* handle,
    const double desired6[6],
    double* thrust_out,
    size_t out_capacity);

THRUST_ALLOCATOR_API size_t ta_get_thruster_count(
    const TA_Handle* handle);

THRUST_ALLOCATOR_API int ta_is_initialized(
    const TA_Handle* handle);

#ifdef __cplusplus
}
#endif

#endif