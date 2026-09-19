//
// Minimal ROS1 shim for the VINS-Mono estimator core vendored into
// super_odometry_vio (upstream HKUST-Aerial-Robotics/VINS-Mono @ 90dabb5e,
// GPLv3). The vendored algorithm files include <ros/ros.h> only for logging
// macros and ROS_ASSERT; those are remapped here so the core stays
// ROS-independent (docs/THIRD_PARTY.md, docs/VINS_PORTING_NOTES.md).
//
#pragma once

#include <cassert>
#include <cstdio>
#include <iostream>

#define ROS_DEBUG(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define ROS_DEBUG_STREAM(x) do {} while (0)

#define ROS_INFO(...) do { fprintf(stdout, __VA_ARGS__); } while (0)
#define ROS_INFO_STREAM(x) do { std::cout << "[vio] " << x << std::endl; } while (0)

#define ROS_WARN(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define ROS_WARN_STREAM(x) do { std::cerr << "[vio warn] " << x << std::endl; } while (0)

#define ROS_ERROR(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define ROS_ERROR_STREAM(x) do { std::cerr << "[vio error] " << x << std::endl; } while (0)

#define ROS_FATAL(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define ROS_FATAL_STREAM(x) do { std::cerr << "[vio fatal] " << x << std::endl; } while (0)

#define ROS_ASSERT(cond) assert(cond)
#define ROS_ASSERT_STREAM(cond, x) assert(cond)
#define ROS_BREAK() assert(false)
#define ROS_COND_BREAK(cond) do { if (!(cond)) assert(false); } while (0)
#define ROS_CONDBREAK(cond) ROS_COND_BREAK(cond)
