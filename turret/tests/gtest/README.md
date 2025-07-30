# Google Test Suite for Turret Project

This directory contains the Google Test (gtest) suite for the Turret project. These tests are designed to run on your local machine, providing a way to test the logic of the C++ code without needing to flash it to the Arduino board.

## Setup

### Dependencies

- **gtest/gmock**: The Google Test framework is required. This can be installed via your system's package manager. For example, on macOS with Homebrew:
  ```bash
  brew install googletest
  ```

### Mocks

The `tests/gtest/mocks/` directory contains mock implementations of the Arduino-specific libraries and functions. This allows the turret's C++ code, which is written for the Arduino environment, to be compiled and run in a standard C++ environment on your local machine.

When the tests are built, the `GMOCK_FLAG` preprocessor directive is defined. This is used in the main source code (`src/turret.cpp`) to conditionally include the mock headers instead of the real Arduino headers.

## Building and Running Tests

A `Makefile` is provided in the root of the project to simplify the build and test process.

### Build and Run

To build and run the tests, simply run the `test` target from the project's root directory:

```bash
make test
```

This command will:
1. Compile the test files, the source code, and the mock files.
2. Link them against the gtest libraries.
3. Create an executable file named `test` in the root directory.

To run the tests after building, execute the `test` binary:
```bash
./test
```

### Clean

To clean up the build artifacts, run the `clean` target:
```bash
make clean
```
