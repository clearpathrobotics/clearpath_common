# Contributing to clearpath_common

Thanks for your interest in improving `clearpath_common`! This repository holds the
platform-agnostic building blocks shared by both real robots (`clearpath_robot`) and simulation
(`clearpath_simulator`): URDF descriptions, controllers, diagnostics, and the **common generator**
(`clearpath_generator_common`) that turns a parsed `robot.yaml` into launch/description output.
Because these packages sit at the center of the stack, changes here can ripple through the rest of
the system, so please read the notes below before opening a pull request.

## Getting started

1. Fork the repository and clone your fork.
2. Create a feature branch off `jazzy`:

   ```bash
   git checkout -b my-feature jazzy
   ```

3. Build the workspace and source it:

   ```bash
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
(trailing whitespace, end-of-file, YAML/JSON checks, `markdownlint`, `flake8`, and `cspell`
spell-checking) before each commit. Run them against the whole tree before pushing:

```bash
pre-commit run --all-files
```

## Where things live

This repository is a collection of packages (see the [Packages section of the
README](README.md#packages)). A few of the most commonly touched:

- [`clearpath_generator_common`](clearpath_generator_common/clearpath_generator_common) — the
  shared generator base that `clearpath_generator_robot` and `clearpath_generator_gz` build on. Its
  generators and templates produce bash setup, robot description (URDF/xacro), the discovery server
  config, and the manipulator SRDF.
- [`clearpath_description`](clearpath_description),
  [`clearpath_platform_description`](clearpath_platform_description), and the other `*_description`
  packages — URDF/xacro and mesh assets.
- [`clearpath_control`](clearpath_control) — controllers, localization, and teleoperation launch.
- [`clearpath_diagnostics`](clearpath_diagnostics) — diagnostic updater and aggregator.

Keep changes in the package that matches the layer they belong to, and avoid unrelated refactors in
the same pull request.

## Coding style

- Python: follow [PEP 8](https://peps.python.org/pep-0008/) with a 100-character line limit.
- Keep changes focused and scoped to the package you are modifying.

## Tests

Build and run the tests through `colcon`:

```bash
colcon build --symlink-install
colcon test --packages-select <package_name>
colcon test-result --verbose
```

Please add or update tests for any behavior you change.

## Generator tests

Changes to the generators in this repository (`clearpath_generator_common`) may affect the
generated output for launch files, parameter files, and descriptions. The
[clearpath_generator_tests](https://github.com/clearpathrobotics/clearpath_generator_tests)
repository versions the expected output and validates it through CI.

Before merging, ensure a corresponding branch with the **same name** exists in
`clearpath_generator_tests` with regenerated samples. See the
[Development Workflow](https://github.com/clearpathrobotics/clearpath_generator_tests#development-workflow)
section of that repository for the full process.

## Submitting a pull request

1. Make sure the workspace builds and `colcon test` passes.
2. Push your branch and open a pull request against `jazzy`.
3. Write a clear description of what the change does and why. Link any related issues.
4. If your change touches generator output, note the matching `clearpath_generator_tests` branch in
   the pull request description.

## Reporting issues

Please open issues on the
[GitHub issue tracker](https://github.com/clearpathrobotics/clearpath_common/issues) and fill out
the bug report template, which walks you through the details we need to reproduce the problem.

## License

By contributing, you agree that your contributions will be licensed under the
[BSD-3-Clause license](LICENSE) that covers this project.
