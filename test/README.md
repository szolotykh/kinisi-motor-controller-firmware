# Testing Guidelines for Embedded Firmware

This directory is intended for PlatformIO Test Runner and project tests.

Unit Testing is a software testing method by which individual units of
source code, sets of one or more MCU program modules together with associated
control data, usage procedures, and operating procedures, are tested to
determine whether they are fit for use. Unit testing finds problems early
in the development cycle.

More information about PlatformIO Unit Testing:
- https://docs.platformio.org/en/latest/advanced/unit-testing/index.html

This document provides guidelines for writing and structuring tests for embedded firmware using the Unity test framework.

## Test Structure

Tests should be organized in the following structure:

1. **Test directory**: Create a directory named `test_<module>` for each module you want to test
2. **Test file**: Create a main test file named `test_<module>.c` in that directory
3. **Mock files**: Create mock files named `mock_<module>.h` and `mock_<module>.c` when needed

### Example Directory Structure:

```
test/
├── test_module_a/
│   ├── test_module_a.c
│   ├── mock_module_a.h
│   └── mock_module_a.c
└── test_module_b/
    └── test_module_b.c
```

## Writing Tests

### 1. Include Required Headers

```c
#include "unity.h"         // Unity testing framework
#include "<module>.h"      // Module being tested
#include "mock_<module>.h" // Mock implementation if needed
```

### 2. Set Up and Tear Down

Set up the test environment before each test and clean up after:

```c
void setUp(void) {
    // Reset mocks
    mock_reset_all();
    
    // Set up test interfaces
    set_interface_for_testing(&mock_interface);
    
    // Initialize test data
    initialize_test_data();
}

void tearDown(void) {
    // Clean up resources
    cleanup_test_resources();
}
```

### 3. Test Cases

Write test functions with clear names indicating what they test:

```c
void test_module_initializes_correctly(void) {
    // Arrange - Set up test conditions
    // ...
    
    // Act - Call the function under test
    module_initialize(param1, param2);
    
    // Assert - Check results
    TEST_ASSERT_EQUAL(expected_value, actual_value);
    TEST_ASSERT_TRUE(mock_function_was_called());
}
```

### 4. Main Function

Include a main function that runs all your tests:

```c
int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_module_initializes_correctly);
    RUN_TEST(test_module_handles_error_conditions);
    // Add all your tests here
    
    return UNITY_END();
}
```

## Creating and Using Mocks

### Step 1: Define Mock Interface Structure

Create a header file with your mock interface structure that mirrors the real interface:

```c
// In mock_hardware.h
#ifndef MOCK_HARDWARE_H
#define MOCK_HARDWARE_H

#include "hardware.h"

// Interface structure matching the real hardware interface
typedef struct {
    void (*initialize)(uint8_t device_id);
    int (*read_data)(uint8_t device_id, uint8_t* buffer, uint16_t length);
    int (*write_data)(uint8_t device_id, const uint8_t* data, uint16_t length);
} hardware_interface_t;

// Functions to get and set mock interface
const hardware_interface_t* mock_hardware_get_interface(void);
void mock_hardware_reset(void);

// Counter getters to verify calls
int mock_get_initialize_calls(void);
uint8_t mock_get_last_device_id(void);
const uint8_t* mock_get_last_written_data(void);

#endif // MOCK_HARDWARE_H
```

### Step 2: Implement Mock Functions

Create a source file with your mock implementation:

```c
// In mock_hardware.c
#include "mock_hardware.h"
#include <string.h>

// Tracking variables
static int initialize_calls = 0;
static uint8_t last_device_id = 0;
static uint8_t last_written_data[256];
static uint16_t last_written_length = 0;

// Mock function implementations
static void mock_initialize(uint8_t device_id) {
    initialize_calls++;
    last_device_id = device_id;
}

static int mock_read_data(uint8_t device_id, uint8_t* buffer, uint16_t length) {
    memset(buffer, 0xA5, length); // Fill with test pattern
    return length;
}

static int mock_write_data(uint8_t device_id, const uint8_t* data, uint16_t length) {
    if (length <= sizeof(last_written_data)) {
        memcpy(last_written_data, data, length);
        last_written_length = length;
        return length;
    }
    return -1;
}

// Create the mock interface structure
static const hardware_interface_t mock_interface = {
    .initialize = mock_initialize,
    .read_data = mock_read_data,
    .write_data = mock_write_data
};

// Interface getter
const hardware_interface_t* mock_hardware_get_interface(void) {
    return &mock_interface;
}

// Reset function
void mock_hardware_reset(void) {
    initialize_calls = 0;
    last_device_id = 0;
    memset(last_written_data, 0, sizeof(last_written_data));
    last_written_length = 0;
}

// Counter getters
int mock_get_initialize_calls(void) {
    return initialize_calls;
}

uint8_t mock_get_last_device_id(void) {
    return last_device_id;
}

const uint8_t* mock_get_last_written_data(void) {
    return last_written_data;
}
```

### Step 3: Add Interface Swapping in Production Code

In your real module, add a way to override the default interface:

```c
// In hardware.c
#include "hardware.h"

// Default implementation
static void hw_initialize(uint8_t device_id) { /* Real implementation */ }
static int hw_read_data(uint8_t device_id, uint8_t* buffer, uint16_t length) { /* Real implementation */ }
static int hw_write_data(uint8_t device_id, const uint8_t* data, uint16_t length) { /* Real implementation */ }

// Default interface
static const hardware_interface_t default_interface = {
    .initialize = hw_initialize,
    .read_data = hw_read_data,
    .write_data = hw_write_data
};

// Current interface - points to default by default
static const hardware_interface_t* current_interface = &default_interface;

// Public function to get the interface
const hardware_interface_t* hardware_get_interface(void) {
    return current_interface;
}

// Function for testing to override the interface
void hardware_set_interface_for_testing(const hardware_interface_t* interface) {
    current_interface = interface ? interface : &default_interface;
}
```

### Step 4: Use the Mock in Tests

Now you can use your mock in tests:

```c
// In test_module.c
#include "unity.h"
#include "module.h"
#include "mock_hardware.h"

void setUp(void) {
    mock_hardware_reset();
    hardware_set_interface_for_testing(mock_hardware_get_interface());
}

void tearDown(void) {
    hardware_set_interface_for_testing(NULL); // Reset to default
}

void test_module_initializes_hardware(void) {
    // Arrange
    const uint8_t expected_device_id = 42;
    
    // Act
    module_initialize(expected_device_id);
    
    // Assert
    TEST_ASSERT_EQUAL(1, mock_get_initialize_calls());
    TEST_ASSERT_EQUAL(expected_device_id, mock_get_last_device_id());
}

void test_module_writes_correct_data(void) {
    // Arrange
    const uint8_t expected_data[] = {0x01, 0x02, 0x03};
    
    // Act
    module_send_command(expected_data, sizeof(expected_data));
    
    // Assert
    const uint8_t* actual_data = mock_get_last_written_data();
    TEST_ASSERT_EQUAL_MEMORY(expected_data, actual_data, sizeof(expected_data));
}
```

## Testing Patterns and Best Practices

### 1. Test Different Categories

- **Unit Tests**: Test individual functions in isolation
- **Integration Tests**: Test how components work together
- **Error Tests**: Test error handling and edge cases
- **Functionality Tests**: Test complete features

### 2. Use Descriptive Test Names

Name tests to clearly describe what they verify:
- `test_module_initializes_correctly`
- `test_module_handles_null_pointer`
- `test_module_returns_error_when_device_unavailable`

### 3. Use Test Assertions Effectively

Choose the right assertion for each test:
- `TEST_ASSERT_EQUAL(expected, actual)` - Integers
- `TEST_ASSERT_EQUAL_FLOAT(expected, actual)` - Floating point
- `TEST_ASSERT_EQUAL_STRING(expected, actual)` - Strings
- `TEST_ASSERT_EQUAL_MEMORY(expected, actual, size)` - Memory blocks
- `TEST_ASSERT_TRUE(condition)` - Boolean conditions

### 4. Test Invalid and Edge Cases

Always test:
- Invalid inputs (NULL pointers, out-of-range values)
- Boundary conditions (empty buffers, max values)
- Error conditions (hardware failures, resource unavailability)

### 5. Keep Tests Independent

Each test should:
- Begin with a clean state (use setUp)
- Not depend on other tests running first
- Clean up after itself (use tearDown)

### 6. Use Test Coverage Tools

Measure your test coverage to identify untested code:
- Line coverage: Each line of code executed
- Branch coverage: Each decision path taken
- Function coverage: Each function called

## Example Test Cases

### Testing Initialization

```c
void test_initialize_with_valid_parameters(void) {
    // Arrange
    const uint8_t device_id = 1;
    const uint16_t buffer_size = 1024;
    
    // Act
    int result = module_initialize(device_id, buffer_size);
    
    // Assert
    TEST_ASSERT_EQUAL(MODULE_OK, result);
    TEST_ASSERT_EQUAL(1, mock_get_initialize_calls());
}
```

### Testing Error Handling

```c
void test_function_handles_null_pointer(void) {
    // Act
    int result = module_process_data(NULL, 10);
    
    // Assert
    TEST_ASSERT_EQUAL(MODULE_ERROR_NULL_POINTER, result);
}
```

### Testing Complex Logic

```c
void test_calculate_velocity_correctly(void) {
    // Arrange
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1};
    double encoder_values[] = {1000.0, 2000.0};
    double expected_velocity = 0.75;  // Expected based on calculation
    
    // Act
    double result = calculate_velocity(motor_indexes, encoder_values, 2);
    
    // Assert
    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_velocity, result);
}
```

## Board configuration generation (host tests)

Run `python test/test_board_config/run_tests.py` with GCC or Clang on PATH, or
set `CC` to its executable. This validates the JSON generator, deterministic
output and `--check`, MCU/board selection, and rejection of unsafe or unsupported
settings. It compiles all four generated C profiles and compares their mappings
against a snapshot of the original manual header. It also tests production LED
initialization and commands, including multiple LEDs and active-low polarity.
The separate motor and INIT suites below check integration behavior.
`python test/test_board_config/run_build_tests.py` additionally builds an isolated
synthetic board revision and checks the compiled startup and command fault paths.
It needs PlatformIO on PATH, or `--pio` set to its executable.

## Motor control models (host tests)

Run `python test/test_motor_control/run_tests.py` with GCC or Clang on PATH,
or set `CC` to its executable. This compiles the production motor control and
DRV8873 helper against a fake HAL for MP_V1 through MP_V4. It checks signed
duty, saturation, reversal, coast/brake, invalid input, shared timer setup,
complementary output selection, SPI failures/readback and fault-pin handling.
It also checks driver reinitialization after a reset, failed retries, and
reuse of running PWM channels. `test/test_position/run_tests.py` checks whole
platform rollback when any driver fails; `test/test_time_sync/run_tests.py`
checks that a failed driver cannot leave a PID/position controller running.
Physical PWM polarity and electrical behavior require powered board tests.

## Board temperature (host tests)

Run `python test/test_temperature/run_tests.py` with GCC or Clang on PATH, or
set `CC`. This compiles the production TMP102 driver against a fake HAL for all
four boards. It checks I2C3 addressing and pin setup, signed temperature decoding,
startup readiness, tick wraparound, configuration verification, I2C failures and
recovery. Legacy builds link without sensor I2C functions. The board configuration
suite checks sensor definitions and pin conflicts; the INIT suite checks protocol
compatibility and temperature command framing. Physical readings require a board.
Recovery tests model a timeout leaving BUSY asserted, configuration-transfer
failures, SDA release after nine clocks, permanently low SDA/SCL, timer wrap,
failed peripheral reinitialization, and two temperature sensors sharing the bus.
They verify bounded failure and a fresh conversion wait after recovery.

## IMU (host tests)

Run `python test/test_imu/run_tests.py` with GCC or Clang on PATH, or set `CC`.
It compiles the production BMI088 driver for all four board revisions and checks
SPI2 pin/mode setup, independent chip selects, accel dummy bytes, device IDs,
reset/write delays, signed six-axis SI conversion, settling and tick wraparound,
configuration readback, errors and recovery. Legacy builds link without IMU SPI
functions. Board generator tests check sensor/bus/pin definitions; INIT tests check
the 48-byte `GET_IMU` reply and invalid requests. Host tests do not verify the PCB.
