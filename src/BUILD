cc_library(
    name = "gsl",
    srcs = [],
    hdrs = [],
    includes = ["/usr/include"],  # Change if necessary
    linkopts = ["-lgsl", "-lgslcblas"],  # Link GSL and CBLAS
    visibility = ["//visibility:public"],
)

cc_library(
    name="spherical_harmonic_models",
    srcs=["spherical_harmonic_models.cc"],
    hdrs=["spherical_harmonic_models.h"],
    deps=[":gsl", ":math"],
    data=["//coeffs:coeffs"],
)

cc_library(
    name="wgs84",
    hdrs=["wgs84.h"],
)

cc_library(
    name="math",
    srcs=["math.cc"],
    hdrs=["math.h"],
    deps=[":gsl", ":time", ":wgs84"]
)

cc_library(
    name="conversions",
    srcs=["conversions.cc"],
    hdrs=["conversions.h"],
)

cc_test(
    name="conversions_test",
    srcs=["conversions_test.cc"],
    deps=[
        ":conversions",
        "@googletest//:gtest",
        "@googletest//:gtest_main"
    ]
)

cc_library(
    name="time",
    srcs=["time.cc"],
    hdrs=["time.h"],
)

cc_binary(
    name="main",
    srcs=["main.cc"],
    deps=[":spherical_harmonic_models", ":time"],
)
