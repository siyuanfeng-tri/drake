# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.urdf",
        "**/*.STL",
        "**/*.dae",
        "**/*.mtl",
        "**/*.png",
    ]),
    visibility = ["//visibility:public"],
)

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "fetch_common",
    srcs = [
        "fetch_common.cc",
    ],
    hdrs = [
        "fetch_common.h",
    ],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_binary(
    name = "run_passive_fetch_demo",
    srcs = ["run_passive_fetch_demo.cc"],
    add_test_rule = 1,
    data = [
        "//drake/examples/Fetch:models",
    ],
    test_rule_args = ["--simulation_sec=0.1"],
    deps = [
        ":fetch_common",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "run_fetch_demo",
    srcs = ["run_fetch_demo.cc"],
    add_test_rule = 1,
    data = [
        "//drake/examples/Fetch:models",
    ],
    test_rule_args = ["--simulation_sec=10"],
    deps = [
        ":fetch_common",
        "//drake/common:text_logging_gflags",
        "//drake/examples/Fetch/controller:fetch_controller",
        "//drake/lcm",
        "//drake/lcmtypes:contact_results_for_viz",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
