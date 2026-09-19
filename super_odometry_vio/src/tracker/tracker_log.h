#pragma once
//
// Logging shim for the VINS-Mono core imported into super_odometry_vio.
// Upstream feature_tracker.cpp calls the ROS1 ROS_DEBUG/INFO/WARN/ERROR macros;
// the core must stay ROS-independent (docs/THIRD_PARTY.md), so the macros are
// remapped here instead of touching the imported algorithm code.
//
#include <cstdio>
#include <iostream>

#define ROS_DEBUG(...) do {} while (0)
#define ROS_DEBUG_STREAM(x) do {} while (0)

#define ROS_INFO(...) do { fprintf(stdout, __VA_ARGS__); } while (0)
#define ROS_INFO_STREAM(x) do { std::cout << "[vio] " << x << std::endl; } while (0)

#define ROS_WARN(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define ROS_WARN_STREAM(x) do { std::cerr << "[vio warn] " << x << std::endl; } while (0)

#define ROS_ERROR(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define ROS_ERROR_STREAM(x) do { std::cerr << "[vio error] " << x << std::endl; } while (0)
