# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "fetch_controller",
    srcs = [
        "fetch_controller.cc",
    ],
    hdrs = [
        "fetch_controller.h",
    ],
    deps = [
        "//drake/examples/Fetch:fetch_common",
        "//drake/systems/framework:framework"
    ],
)

cpplint()
