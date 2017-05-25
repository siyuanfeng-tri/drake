# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)

drake_cc_binary(
    name = "controlled_kuka_demo",
    srcs = ["controlled_kuka_demo.cc"],
    add_test_rule = 1,
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    tags = ["snopt"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids/system:iiwa_controller",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:sim_diagram_builder",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:trajectory_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "sfeng_iiwa",
    srcs = ["sfeng_iiwa.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/lcmtypes:contact_info_for_viz",
        "//drake/lcmtypes:contact_results_for_viz",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/examples/QPInverseDynamicsForHumanoids/system:iiwa_controller",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:oracular_state_estimator",
        "//drake/examples/kuka_iiwa_arm:sim_diagram_builder",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "//drake/systems/primitives:trajectory_source",
    ],
)

drake_cc_binary(
    name = "sfeng_command",
    srcs = ["sfeng_command.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids/system:iiwa_controller",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:oracular_state_estimator",
        "//drake/examples/kuka_iiwa_arm:sim_diagram_builder",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "//drake/systems/primitives:trajectory_source",
    ],
)
