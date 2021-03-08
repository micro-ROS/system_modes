This document is a declaration of software quality for the `system_modes` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# System Modes Quality Declaration

The package `system_modes` claims to be in the **Quality Level 3** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-quality-categories) of the ROS 2 developer guide.

## Version Policy [1]

### Version Scheme [1.i]

`system_modes` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`system_modes` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API.

Except for the exclusions listed below, all installed headers are in the `include` directory of the package, headers in any other folders are not installed and considered private.

### API Stability Policy [1.iv]

`system_modes` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Policy [1.v]

`system_modes` contains C++ code and therefore must be concerned with ABI stability, and will maintain ABI stability within a ROS distribution.

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

`system_modes` will not break API nor ABI within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

## Change Control Process [2]

`system_modes` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process).

### Change Requests [2.i]

All changes will occur through a pull request, check [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers) (currently not true for tier 1 platform _Windows_).
Latest build results can be seen [here](https://github.com/micro-ROS/system_modes/actions).

###  Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`system_modes` has a [feature list](http://docs.ros2.org/latest/api/system_modes/) and each item in the list links to the corresponding feature documentation. There is documentation for all of the features, and new features require documentation before being added.

### License [3.iii]

The license for `system_modes` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](./package.xml) manifest file, and a full copy of the license is in the [`LICENSE`](../LICENSE) file.

There is an automated test which runs a linter that ensures each file has a license statement. [Here](http://build.ros2.org/view/Rpr/job/Rpr__system_modes__ubuntu_focal_amd64/lastCompletedBuild/testReport/system_modes/)**TODO** can be found a list with the latest results of the various linters being run on the package.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `system_modes`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen [here](http://build.ros2.org/view/Rpr/job/Rpr__system_modes__ubuntu_focal_amd64/lastCompletedBuild/testReport/system_modes/copyright/). **TODO**

## Testing [4]

### Feature Testing [4.i]

Each feature in `system_modes` has corresponding tests which simulate typical usage, located in the [`test`](https://github.com/micro-ROS/system_modes/tree/master/system_modes/test) directory.
New features are required to have tests before being added.
Latest test results can be seen [here](https://github.com/micro-ROS/system_modes/actions).

### Coverage [4.iii]

`system_modes` follows the recommendations for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#code-coverage), and opts to use line coverage instead of branch coverage.

This includes:

- tracking and reporting line coverage statistics
- achieving and maintaining a reasonable branch line coverage (90-100%)
- no lines are manually skipped in coverage calculations

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

Current coverage statistics can be viewed [here](https://codecov.io/gh/micro-ROS/system_modes).

`system_modes` has a line coverage `>= 20%`, which is calculated over all directories within `system_modes`.

### Linters and Static Analysis [4.v]

`system_modes` uses and passes all the ROS2 standard linters and static analysis tools for a C++ package as described in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

Latest linting results can be seen [here](https://github.com/micro-ROS/system_modes/actions).

## Dependencies [5]

Below are evaluations of each of `system_modes`'s run-time and build-time dependencies that have been determined to influence the quality.

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

It also has several test dependencies, which do not affect the resulting quality of the package, because they are only used to build and run the test code.

### Direct and Optional Runtime ROS Dependencies [5.i]/[5.ii]

`system_modes` has the following runtime ROS dependencies:

#### `rclcpp_lifecycle`

The `rclcpp` package provides theROS client library in C++. It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rclcpp/blob/master/rclcpp/QUALITY_DECLARATION.md). The same is true for its transient dependencies, see its [Quality Declaration document](https://github.com/ros2/rclcpp/blob/master/rclcpp/QUALITY_DECLARATION.md).

#### `rclcpp_lifecycle`

The `rclcpp_lifecycle` package provides the ROS 2 standard lifecycle in C++. It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/QUALITY_DECLARATION.md). The same is true for its transient dependencies, see its [Quality Declaration document](https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/QUALITY_DECLARATION.md).

### Direct Runtime non-ROS Dependency [5.iii]

`system_modes` has no run-time or build-time dependencies that need to be considered for this declaration.

## Platform Support [6]

`system_modes` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers) (currently not true for tier 1 platform _Windows_), and tests each change against all of them.

## Security

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
