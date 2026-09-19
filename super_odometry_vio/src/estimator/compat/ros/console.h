// Compat shim: vendored files include <ros/console.h> for the same logging
// macros provided in ros/ros.h; route to that shim.
#pragma once
#include "ros/ros.h"
