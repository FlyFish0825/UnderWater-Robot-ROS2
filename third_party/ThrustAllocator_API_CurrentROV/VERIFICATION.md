# Verification

Date: 2026-09-08
Platform: Windows 10 x64
Toolchain used for packaged Windows binary:
- GCC/G++ 15.2.0 (D:/w64devkit)
- CMake 4.1.1
- Ninja 1.13.2

Verified from a clean build directory:
1. Configure bundled Eigen/OSQP/QDLDL/OsqpEigen offline: PASS
2. Build libthrust_allocator.a: PASS
3. Build current-ROV example: PASS
4. Run current-ROV example: PASS
5. Install SDK and CMake package files: PASS
6. Configure independent consumer using find_package: PASS
7. Build/link independent consumer against installed SDK: PASS
8. Run independent consumer, thruster count = 8: PASS

URDF geometry review:
- Horizontal thrusters 1-4: original overlay geometry matches URDF.
- Vertical axes 5-8: original overlay directions match URDF (+Z in base_link).
- Vertical positions 5-8: original overlay positions DO NOT match URDF and were corrected.
- Corrected 6x8 geometry allocation matrix rank = 6.

Note: max thrust and propeller reaction torque are not defined by URDF and require real propulsion calibration.
