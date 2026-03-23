# IntelliVibe-BT Firmware — Coding Conventions

## File Structure

Every `.c` file follows this layout (see `Controller.c` as reference):

```c
/*******************************************************************************
********************************************************************************
Description:
<What this file handles>

Author(s): <names>

Date created: <month year>
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************

/******************************************************************************
// Data types
******************************************************************************/

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
```

Every `.h` file follows this layout (see `Controller.h` as reference):

```c
/*******************************************************************************
********************************************************************************
Description:
<What this file defines>

Author(s): <names>

Date created: <month year>
*******************************************************************************/

#ifndef FILENAME_H
#define FILENAME_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************

//******************************************************************************
// DEFINES
//******************************************************************************

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// EXTERNS
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************

#ifdef __cplusplus
}
#endif

#endif // FILENAME_H
```

## Naming Conventions

### Variables

| Scope | Prefix | Example |
|-------|--------|---------|
| File-local (static) | `m_` + PascalCase | `static uint8_t m_BatterySoc;` |
| Function-local | PascalCase (no prefix) | `bool Status = false;` |
| Struct members | PascalCase (no underscores) | `AdvData.AccelRms` |
| No global variables | — | Use getter/setter functions instead |

- Function-local variables MUST be declared at the beginning of the function body.
- Each keyword in a variable name starts with a capital letter (PascalCase).
- No underscores in variable names — use `AdvData` not `adv_data`, `InitFlag` not `init_flag`.
- The only underscore allowed is the `m_` prefix for file-scope static variables.

### Functions

- All functions in a file MUST be prefixed with the filename: `BLE_Init()`, `BLE_AdvStart()`, `BLEConfig_Write()`
- Public (global) functions are declared in the header file.
- Private (static) functions are declared in LOCAL FUNCTION PROTOTYPES section and defined in LOCAL FUNCTION DEFINITIONS section.
- Getter/setter pattern for accessing file-local variables from other files: `BLE_GetInitFlag()`, `BLE_SetDeviceName()`

### Function Documentation

Every function MUST have a description block:

```c
/*******************************************************************************
Description:
<What the function does — multi-line if needed>

Argument(s):
<argName> - <description>

Return:
<type> - <description>

Note(s):
<optional notes>
*******************************************************************************/
```

### Defines / Macros

- ALL_CAPS with underscores: `BLINK_INTERVAL_MS`, `EEPROM_PAGE_SIZE`

### Data Types (typedefs)

- PascalCase with `_t` suffix: `ConfigParam_t`, `AdvMfgData_t`

## Magic Numbers

- **No bare numeric literals** in code other than `0` and `1`.
- Every other constant must be a `#define` with a descriptive ALL_CAPS name.
- For byte sizes derived from types, use `sizeof()` instead of hardcoding `2`, `4`, `8`, etc.
- Examples:
  ```c
  /* BAD */
  BufLen = Duration * Rate * 3 * 2;
  k_sleep(K_SECONDS(5));

  /* GOOD */
  #define ACCEL_AXIS_COUNT        3
  #define ACCEL_BYTES_PER_SAMPLE  (ACCEL_AXIS_COUNT * sizeof(int16_t))
  BufLen = Duration * Rate * ACCEL_BYTES_PER_SAMPLE;

  #define INIT_SETTLE_TIME_S      5
  k_sleep(K_SECONDS(INIT_SETTLE_TIME_S));
  ```

## General Rules

- No global variables — ever. Use `static` file-scope with getter/setter functions.
- Prefer `LOG_INF` / `LOG_WRN` / `LOG_ERR` over `printk` (except early boot / fault handlers).
- Keep the same directory structure as the old firmware (`src/drivers/BLE/`, `src/DataProcessing/`, etc.).
- BLE UUIDs must remain identical to the old firmware for backward compatibility.

## Build

- Target: `nrf5340dk_nrf5340_cpuapp`
- SDK: NCS v2.6.1 (Zephyr v3.5.99)
- Toolchain path: `~/ncs/toolchains/2be090971e/`
- Build command: `west build -b nrf5340dk_nrf5340_cpuapp`
