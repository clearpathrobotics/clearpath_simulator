# Contributing to clearpath_simulator

Thanks for your interest in improving `clearpath_simulator`! This is the **simulation** half of the
stack: it mirrors `clearpath_robot` but targets [Gazebo](https://gazebosim.org) instead of real
hardware, reusing the same `robot.yaml`, `clearpath_config` parser, and `clearpath_common`
descriptions so a robot behaves the same in sim and on hardware. Please read the notes below before
opening a pull request.

## Getting started

1. Fork the repository and clone your fork.
2. Create a feature branch off `jazzy`:

   ```bash
   git checkout -b my-feature jazzy
   ```

3. Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)
   and Gazebo, then build the workspace:

   ```bash
   sudo apt-get install ros-${ROS_DISTRO}-ros-gz
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Install the pre-commit hooks (one-time setup):

   ```bash
   pip install pre-commit
   pre-commit install
   ```

## Linting

This repository uses [pre-commit](https://pre-commit.com/) to run linting and formatting checks
(trailing whitespace, end-of-file, YAML/JSON checks, `markdownlint`, and `flake8`) before each
commit. Run them against the whole tree before pushing:

```bash
pre-commit run --all-files
```

## Where things live

- [`clearpath_generator_gz`](clearpath_generator_gz) — generates the Gazebo launch and parameter
  files from the parsed config.
- [`clearpath_gz`](clearpath_gz) — simulation bringup, worlds, and assets (launched via
  `simulation.launch.py`).

## Testing your changes

Launch a simulation and confirm your change behaves as expected:

```bash
ros2 launch clearpath_gz simulation.launch.py
```

To test a specific world, use the `world` launch argument (see the [Worlds section of the
README](README.md#worlds) for the full list):

```bash
ros2 launch clearpath_gz simulation.launch.py world:=pipeline
```

Build and run any package tests through `colcon`:

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Generator tests

Changes to the generators in this repository (`clearpath_generator_gz`) may affect the generated
output for launch and parameter files. The
[clearpath_generator_tests](https://github.com/clearpathrobotics/clearpath_generator_tests)
repository versions the expected output and validates it through CI.

Before merging, ensure a corresponding branch with the **same name** exists in
`clearpath_generator_tests` with regenerated samples. See the
[Development Workflow](https://github.com/clearpathrobotics/clearpath_generator_tests#development-workflow)
section of that repository for the full process.

## Continuous integration

Every pull request runs [`clearpath_simulator_ci`](.github/workflows/ci.yml). The jobs differ in
whether they pick up **upstream changes** — commits on a branch with the **same name** as your
branch in one of the repositories listed in [`dependencies.repos`](dependencies.repos) (for example
`clearpath_common` or `clearpath_config`). The
[repos-dep-update-action](https://github.com/clearpathrobotics/repos-dep-update-action) swaps those
matching branches in for the "Head Branch" jobs, so those are the jobs that reflect your upstream
work.

| Job | Sees your upstream branches? | Expected result |
| --- | --- | --- |
| **jazzy** (`testing`/`main`) | Not your feature branches, but builds against the live `testing`/`main` binaries | Can fail when upstream packages change/regress in the `testing`/`main` ROS repos or changes to Clearpath packages this depends on |
| **Jazzy Clearpath Source** | No — default `dependencies.repos` branches | Should pass; unaffected by unmerged upstream branches |
| **Jazzy Clearpath Source with Head Branch** | Yes — your matching upstream branches | Must pass; a failure means your upstream changes broke the build |
| **Jazzy Clearpath Source with Base Branch** (PRs only) | Base branch of upstream | Baseline for the state before your change |
| **Jazzy Clearpath Source with Head Branch Generator Tests** | Yes, plus `clearpath_generator_gz_tests` | Fails if your change alters generator output and no matching branch with regenerated samples exists in `clearpath_generator_tests` |

If the **Generator Tests** job is the only source job that fails, the build itself is fine — your
change altered the generated output and you need a matching-named branch in
`clearpath_generator_tests` with regenerated samples (see [Generator tests](#generator-tests)
above).

## Submitting a pull request

1. Make sure the workspace builds and the simulation launches correctly.
2. Push your branch and open a pull request against `jazzy`.
3. Write a clear description of what the change does and the platform/world you tested against.
4. If your change touches generator output, note the matching `clearpath_generator_tests` branch.

## Reporting issues

Please open issues on the
[GitHub issue tracker](https://github.com/clearpathrobotics/clearpath_simulator/issues) and fill
out the bug report template, which walks you through the details we need to reproduce the problem.

## License

By contributing, you agree that your contributions will be licensed under the
[BSD-3-Clause license](LICENSE) that covers this project.
