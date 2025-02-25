# Testing Guide

This guide provides instructions on how to write and set up tests for the `kinisi-motor-controller-firmware` project.

## Prerequisites

- Ensure you have PlatformIO installed. You can install it from [PlatformIO Installation](https://platformio.org/install).
- Ensure you have the necessary dependencies installed. You can install them using the following command:

```sh
pio lib install
```

## Writing Tests

1. **Create a Test Folder**: Create a new folder under the `test` directory for your test cases. For example, `test/test_new_feature`.

2. **Create Test Files**: Create test files in the new folder. For example, `test/test_new_feature/test_cases.c`.

3. **Include Necessary Headers**: Include the necessary headers in your test files. For example:

```c
#include "unity.h"
#include "your_header.h"
```

4. **Write Test Cases**: Write your test cases using the Unity framework. For example:

```c
void test_example(void) {
    TEST_ASSERT_EQUAL(1, 1);
}
```

5. **Set Up and Tear Down**: Implement the `setUp` and `tearDown` functions if needed. These functions will run before and after each test case, respectively.

```c
void setUp(void) {
    // Code to set up test environment
}

void tearDown(void) {
    // Code to clean up test environment
}
```

6. **Run Tests**: Add your test cases to the `main` function to run them.

```c
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_example);
    return UNITY_END();
}
```

## Running Tests

1. **Navigate to Project Directory**: Open a terminal and navigate to the project directory.

2. **Run Tests**: Use the following command to run the tests:

```sh
pio test -e native -f test_new_feature
```

Replace `test_new_feature` with the name of your test folder.

## Debugging Tests

- Use `printf` statements to print debug information.
- Increase verbosity by using the `-v` or `-vv` option with the `pio test` command.

```sh
pio test -e native -f test_new_feature -vv
```

## Configuring platformio.ini

Ensure that your `platformio.ini` file is configured correctly for running tests. Here is an example configuration for the `native` environment:

```ini
[env:native]
platform = native
test_build_src = yes
build_flags = 
    -D UNITY_INCLUDE_DOUBLE 
    -DUNITY_DOUBLE_PRECISION=1e-12 
    -Iinclude 
    -Ilib/FreeRTOS/src 
    -Ilib/hardware 
    -Ilib/libcontrollers/src
lib_deps =
    FreeRTOS
    cmsis
    stm32cube
test_framework = unity
lib_ignore = FreeRTOS, hardware
build_src_filter  = +<platform_omni.c>
```

## Example

Here is an example of a complete test file:

```c
#include "unity.h"
#include "your_header.h"

void setUp(void) {
    // Code to set up test environment
}

void tearDown(void) {
    // Code to clean up test environment
}

void test_example(void) {
    TEST_ASSERT_EQUAL(1, 1);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_example);
    return UNITY_END();
}
```

## Additional Resources

- [Unity Test Framework](http://www.throwtheswitch.org/unity)
- [PlatformIO Unit Testing](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html)
