# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

package(
    default_visibility = [":__subpackages__"],
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
    visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "iiwa_ik_planner",
    srcs = ["iiwa_ik_planner.cc"],
    hdrs = ["iiwa_ik_planner.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_library(
    name = "iiwa_common",
    srcs = [
        "iiwa_common.cc",
    ],
    hdrs = [
        "iiwa_common.h",
    ],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "@bot_core_lcmtypes//:lib",
        "@robotlocomotion_lcmtypes//:lib",
    ],
)

drake_cc_library(
    name = "iiwa_ik_planner",
    srcs = ["iiwa_ik_planner.cc"],
    hdrs = ["iiwa_ik_planner.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_library(
    name = "iiwa_lcm",
    srcs = ["iiwa_lcm.cc"],
    hdrs = ["iiwa_lcm.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/lcmtypes:iiwa",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "oracular_state_estimator",
    hdrs = ["oracular_state_estimator.h"],
    deps = [
        "//drake/systems/framework:leaf_system",
        "@bot_core_lcmtypes//:lib",
    ],
)

drake_cc_binary(
    name = "iiwa_wsg_simulation",
    srcs = ["iiwa_wsg_simulation.cc"],
    data = [
        ":models",
        "//drake/examples/schunk_wsg:models",
    ],
    deps = [
        ":iiwa_lcm",
        ":oracular_state_estimator",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/controllers:pid_with_gravity_compensator",
        "//drake/systems/primitives:matrix_gain",
        "//drake/util:lcm_util",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "kuka_simulation",
    srcs = ["kuka_simulation.cc"],
    data = [":models"],
    deps = [
        ":iiwa_common",
        ":iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:pid_with_gravity_compensator",
        "//drake/systems/primitives:constant_vector_source",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "kuka_plan_runner",
    srcs = ["kuka_plan_runner.cc"],
    data = [":models"],
    deps = [
        ":iiwa_common",
        ":iiwa_lcm",
    ],
)

cc_binary(
    name = "kuka_ik_demo",
    srcs = ["kuka_ik_demo.cc"],
    data = [":models"],
    deps = [
        ":iiwa_common",
        "//drake/lcm",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/solvers:mathematical_program",
    ],
)

drake_cc_library(
    name = "kuka_pick_and_place",
    srcs = ["pick_and_place_state.cc"],
    hdrs = ["pick_and_place_state.h"],
    deps = [
        ":iiwa_common",
        ":iiwa_lcm",
        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/util:lcm_util",
        "//drake/lcm",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/solvers:mathematical_program",
    ],
)

drake_cc_binary(
    name = "pick_and_place_state_machine",
    srcs = [
        "pick_and_place_state_machine.cc",
    ],
    deps = [
        ":iiwa_common",
        ":kuka_pick_and_place",
        ":iiwa_ik_planner",
        ":iiwa_lcm",
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/util:lcm_util",
    ],
)

cpplint()
