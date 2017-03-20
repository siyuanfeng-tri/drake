# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_binary", "drake_cc_googletest", "drake_cc_library")

package(
    default_visibility = ["//drake/examples/QPInverseDynamicsForHumanoids:__subpackages__"],
)

drake_cc_library(
    name = "generic_plan",
    srcs = [
        "generic_plan.cc",
        "plan_eval_utils.cc",
    ],
    hdrs = [
        "generic_plan.h",
        "plan_eval_utils.h",
        "clone_only_value.h"
    ],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller_common",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",
        "//drake/multibody:rigid_body_tree",
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/common/trajectories:piecewise_polynomial",
    ],
)

drake_cc_library(
    name = "manipulator_streaming_plan",
    srcs = ["manipulator_streaming_plan.cc"],
    hdrs = ["manipulator_streaming_plan.h"],
    deps = [
        ":generic_plan",
    ],
)

drake_cc_library(
    name = "manipulator_plans",
    srcs = [
        "manipulator_move_end_effector_plan.cc",
        "manipulator_move_joints_plan.cc",
    ],
    hdrs = [
        "manipulator_move_end_effector_plan.h",
        "manipulator_move_joints_plan.h",
    ],
    deps = [
        ":generic_plan",
        "//drake/lcmtypes:manipulator_plan_move_end_effector",
        "//drake/util:lcm_util",
        "@bot_core_lcmtypes//:lib",
        "@robotlocomotion_lcmtypes//:lib",
    ],
)

drake_cc_library(
    name = "humanoid_plan",
    srcs = [
        "humanoid_manipulation_plan.cc",
        "humanoid_plan.cc",
        "humanoid_walking_plan.cc",
    ],
    hdrs = [
        "humanoid_manipulation_plan.h",
        "humanoid_plan.h",
        "humanoid_walking_plan.h",
    ],
    deps = [
        ":generic_plan",
        "//drake/examples/QPInverseDynamicsForHumanoids:lcm_utils",
        "//drake/systems/controllers:zmp_planner",
        "//drake/util:util",
        "@robotlocomotion_lcmtypes//:lib",
    ],
)
