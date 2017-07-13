# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "planar_pushing",
    srcs = [
        "dubins_interface.cc",
        "dubins.cc",
    ],
    hdrs = [
        "dubins_interface.h",
        "dubins.h",
    ],
    deps = [
        "//drake/common:common",
    ],
)

