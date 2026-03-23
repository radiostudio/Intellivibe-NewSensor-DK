/*******************************************************************************
********************************************************************************
Description:
This file handles system low-power GPIO management. Before entering sleep,
all GPIOs are configured to minimise leakage current:
  - SPI clock and MOSI pins: driven low (low-power pinctrl state)
  - Sensor interrupt pins: interrupts disabled, driven low
  - Input-only pins (MISO, I2S_SDIN, TEMP_ALERT): disconnected
  - Load switches (PSRAM VDD): disabled (driven low)
On wake, GPIOs are restored to their active configuration.

Pin assignments are defined via devicetree aliases so the same code works
across the DK and custom PCB by updating only the DTS overlay.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(SYSLP, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "SysLP.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************

/*
 * GPIO categories for low-power configuration (FRS 8.2 REQ-PWR-02):
 *
 * OUTPUT_DRIVE_LOW:  SPI clocks, MOSI — driven low in sleep
 * OUTPUT_DRIVE_HIGH: Chip selects (active-low) — driven high to deselect
 * INTERRUPT_PINS:    Sensor data-ready — disable interrupt, drive low
 * INPUT_DISCONNECT:  MISO, I2S_SDIN, TEMP_ALERT — disconnect (no pull)
 * LOAD_SWITCH:       PSRAM_VDD_CTRL — driven low to power off
 */

/* Maximum number of pins per category */
#define SYSLP_MAX_OUTPUT_LOW_PINS       4
#define SYSLP_MAX_OUTPUT_HIGH_PINS      2
#define SYSLP_MAX_INTERRUPT_PINS        3
#define SYSLP_MAX_INPUT_DISCONNECT_PINS 4
#define SYSLP_MAX_LOAD_SWITCH_PINS      1

//******************************************************************************
// DATA TYPES
//******************************************************************************
typedef struct
{
    const struct gpio_dt_spec *Pin;
    bool                       Valid;
} SysLPPin_t;

/******************************************************************************
// FILE SCOPE VARIABLES
******************************************************************************/

/*
 * Pin tables — populated by SysLP_Init() from devicetree.
 *
 * NOTE: These are currently empty stubs. When the DTS overlay defines the
 * actual pin aliases (e.g., acc-sck, acc-mosi, acc-ncs, acc-int, mag-int,
 * psram-vdd-ctrl, etc.), uncomment and populate.
 *
 * Example DTS alias:
 *   aliases {
 *       acc-sck = &gpio0 8 GPIO_ACTIVE_HIGH;
 *   };
 *
 * Example gpio_dt_spec:
 *   static const struct gpio_dt_spec m_AccSck = GPIO_DT_SPEC_GET(DT_ALIAS(acc_sck), gpios);
 */

static bool m_Initialised = false;

/*
 * TODO: Define gpio_dt_spec for each pin when DTS overlay is ready.
 *
 * Output drive low (sleep state = 0):
 *   ACC_SCK, ACC_MOSI, PSRAM_SCK, PSRAM_MOSI
 *
 * Output drive high (sleep state = 1, deselect):
 *   ACC_NCS, PSRAM_NCS
 *
 * Interrupt pins (disable IRQ, drive low):
 *   ACC_INT, MAG_INT, TEMP_ALERT (if interrupt-capable)
 *
 * Input disconnect (floating disabled):
 *   ACC_MISO, PSRAM_MISO, I2S_SDIN, TEMP_ALERT (if input-only)
 *
 * Load switches (drive low = power off):
 *   PSRAM_VDD_CTRL
 */

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static void SysLP_ConfigureOutputLow(void);
static void SysLP_ConfigureOutputHigh(void);
static void SysLP_DisableInterrupts(void);
static void SysLP_DisconnectInputs(void);
static void SysLP_DisableLoadSwitches(void);
static void SysLP_RestoreOutputs(void);
static void SysLP_RestoreInterrupts(void);
static void SysLP_RestoreInputs(void);
static void SysLP_EnableLoadSwitches(void);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the SysLP module. Validates that all GPIO devices referenced
by the pin tables are ready. Must be called once at startup before
SysLP_EnterSleep() or SysLP_ExitSleep() are used.

Argument(s):
None

Return:
int - 0 on success, -1 if any GPIO device is not ready.
*******************************************************************************/
int SysLP_Init(void)
{
    LOG_INF("SysLP init");

    /* TODO: Validate gpio_is_ready_dt() for each pin when DTS is ready.
     *
     * Example:
     * if (!gpio_is_ready_dt(&m_AccSck))
     * {
     *     LOG_ERR("ACC_SCK GPIO not ready");
     *     return -1;
     * }
     */

    m_Initialised = true;
    LOG_INF("SysLP init complete (stubs)");
    return 0;
}

/*******************************************************************************
Description:
Configure all GPIOs to their low-power state before entering sleep.
Must be called after stopping BLE advertising and before k_sleep().

Order of operations:
  1. Disable sensor interrupt pins (prevent spurious wakes)
  2. Drive SPI clock/MOSI pins low
  3. Drive chip select pins high (deselect)
  4. Disconnect input-only pins (no floating)
  5. Disable load switches (power off external ICs)

Argument(s):
None

Return:
None
*******************************************************************************/
void SysLP_EnterSleep(void)
{
    if (!m_Initialised)
    {
        LOG_WRN("SysLP not initialised, skipping sleep config");
        return;
    }

    LOG_DBG("Configuring GPIOs for sleep");

    SysLP_DisableInterrupts();
    SysLP_ConfigureOutputLow();
    SysLP_ConfigureOutputHigh();
    SysLP_DisconnectInputs();
    SysLP_DisableLoadSwitches();
}

/*******************************************************************************
Description:
Restore all GPIOs to their active state after waking from sleep.
Must be called after k_sleep() returns and before sensor re-init.

Order of operations:
  1. Enable load switches (power on external ICs, wait for ready)
  2. Restore output pins to default config
  3. Restore input pins
  4. Re-enable sensor interrupt pins

Argument(s):
None

Return:
None
*******************************************************************************/
void SysLP_ExitSleep(void)
{
    if (!m_Initialised)
    {
        LOG_WRN("SysLP not initialised, skipping wake config");
        return;
    }

    LOG_DBG("Restoring GPIOs for active mode");

    SysLP_EnableLoadSwitches();
    SysLP_RestoreOutputs();
    SysLP_RestoreInputs();
    SysLP_RestoreInterrupts();
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Drive SPI clock and MOSI pins low to minimise leakage during sleep.
Applies to ACC_SCK, ACC_MOSI, PSRAM_SCK, PSRAM_MOSI.

Argument(s):
None

Return:
None
*******************************************************************************/
static void SysLP_ConfigureOutputLow(void)
{
    /* TODO: When DTS pins are defined:
     *
     * gpio_pin_configure_dt(&m_AccSck, GPIO_OUTPUT_INACTIVE);
     * gpio_pin_set_dt(&m_AccSck, 0);
     *
     * gpio_pin_configure_dt(&m_AccMosi, GPIO_OUTPUT_INACTIVE);
     * gpio_pin_set_dt(&m_AccMosi, 0);
     *
     * gpio_pin_configure_dt(&m_PsramSck, GPIO_OUTPUT_INACTIVE);
     * gpio_pin_set_dt(&m_PsramSck, 0);
     *
     * gpio_pin_configure_dt(&m_PsramMosi, GPIO_OUTPUT_INACTIVE);
     * gpio_pin_set_dt(&m_PsramMosi, 0);
     */
}

/*******************************************************************************
Description:
Drive chip select pins high (deselect) during sleep. Active-low CS pins
must be held high to prevent unintended communication.
Applies to ACC_NCS, PSRAM_NCS.

Argument(s):
None

Return:
None
*******************************************************************************/
static void SysLP_ConfigureOutputHigh(void)
{
    /* TODO: When DTS pins are defined:
     *
     * gpio_pin_configure_dt(&m_AccNcs, GPIO_OUTPUT_ACTIVE);
     * gpio_pin_set_dt(&m_AccNcs, 1);
     *
     * gpio_pin_configure_dt(&m_PsramNcs, GPIO_OUTPUT_ACTIVE);
     * gpio_pin_set_dt(&m_PsramNcs, 1);
     */
}

/*******************************************************************************
Description:
Disable sensor data-ready interrupts and drive the pins low to prevent
spurious wakes during sleep.
Applies to ACC_INT, MAG_INT.

Argument(s):
None

Return:
None
*******************************************************************************/
static void SysLP_DisableInterrupts(void)
{
    /* TODO: When DTS pins are defined:
     *
     * gpio_pin_interrupt_configure_dt(&m_AccInt, GPIO_INT_DISABLE);
     * gpio_pin_configure_dt(&m_AccInt, GPIO_OUTPUT_INACTIVE);
     * gpio_pin_set_dt(&m_AccInt, 0);
     *
     * gpio_pin_interrupt_configure_dt(&m_MagInt, GPIO_INT_DISABLE);
     * gpio_pin_configure_dt(&m_MagInt, GPIO_OUTPUT_INACTIVE);
     * gpio_pin_set_dt(&m_MagInt, 0);
     */
}

/*******************************************************************************
Description:
Disconnect input-only pins to prevent floating inputs drawing current.
Applies to ACC_MISO, PSRAM_MISO, I2S_SDIN, TEMP_ALERT.

Argument(s):
None

Return:
None
*******************************************************************************/
static void SysLP_DisconnectInputs(void)
{
    /* TODO: When DTS pins are defined:
     *
     * gpio_pin_configure_dt(&m_AccMiso, GPIO_DISCONNECTED);
     * gpio_pin_configure_dt(&m_PsramMiso, GPIO_DISCONNECTED);
     * gpio_pin_configure_dt(&m_I2sSdin, GPIO_DISCONNECTED);
     * gpio_pin_configure_dt(&m_TempAlert, GPIO_DISCONNECTED);
     */
}

/*******************************************************************************
Description:
Disable load switches to power off external ICs during sleep.
PSRAM_VDD_CTRL is active-high, so drive low to power off.

Argument(s):
None

Return:
None

Note(s):
FRS 8.3: Power-down sequence — deassert, wait 1 ms.
*******************************************************************************/
static void SysLP_DisableLoadSwitches(void)
{
    /* TODO: When DTS pins are defined:
     *
     * gpio_pin_set_dt(&m_PsramVddCtrl, 0);
     * k_msleep(1);
     */
}

/*******************************************************************************
Description:
Restore SPI clock, MOSI, and chip select pins to their default active
configuration after waking from sleep.

Argument(s):
None

Return:
None

Note(s):
The actual SPI peripheral re-init is handled by the sensor driver modules
(Accel_Init, PSRAM_Init, etc.). This function only restores the GPIO config.
*******************************************************************************/
static void SysLP_RestoreOutputs(void)
{
    /* TODO: When DTS pins are defined:
     *
     * Restore SPI pins to their default pinctrl state.
     * This is typically handled by the SPI driver on re-init,
     * but explicit restore may be needed for dedicated GPIO-controlled CS.
     *
     * gpio_pin_configure_dt(&m_AccNcs, GPIO_OUTPUT_ACTIVE);
     * gpio_pin_set_dt(&m_AccNcs, 1);
     *
     * gpio_pin_configure_dt(&m_PsramNcs, GPIO_OUTPUT_ACTIVE);
     * gpio_pin_set_dt(&m_PsramNcs, 1);
     */
}

/*******************************************************************************
Description:
Restore input pins to their active configuration after waking from sleep.

Argument(s):
None

Return:
None
*******************************************************************************/
static void SysLP_RestoreInputs(void)
{
    /* TODO: When DTS pins are defined:
     *
     * gpio_pin_configure_dt(&m_AccMiso, GPIO_INPUT);
     * gpio_pin_configure_dt(&m_PsramMiso, GPIO_INPUT);
     * gpio_pin_configure_dt(&m_I2sSdin, GPIO_INPUT);
     */
}

/*******************************************************************************
Description:
Re-enable sensor data-ready interrupt pins after waking from sleep.

Argument(s):
None

Return:
None

Note(s):
The actual interrupt callback registration is handled by the sensor driver
modules (Accel_Init, Mag_Init). This function re-enables the GPIO interrupt
configuration.
*******************************************************************************/
static void SysLP_RestoreInterrupts(void)
{
    /* TODO: When DTS pins are defined:
     *
     * gpio_pin_configure_dt(&m_AccInt, GPIO_INPUT);
     * gpio_pin_interrupt_configure_dt(&m_AccInt, GPIO_INT_EDGE_TO_ACTIVE);
     *
     * gpio_pin_configure_dt(&m_MagInt, GPIO_INPUT);
     * gpio_pin_interrupt_configure_dt(&m_MagInt, GPIO_INT_EDGE_TO_ACTIVE);
     */
}

/*******************************************************************************
Description:
Enable load switches to power on external ICs after waking from sleep.
PSRAM_VDD_CTRL is active-high, so drive high to power on.

Argument(s):
None

Return:
None

Note(s):
FRS 8.3: Power-up sequence — assert, wait 1 ms, then ready.
*******************************************************************************/
static void SysLP_EnableLoadSwitches(void)
{
    /* TODO: When DTS pins are defined:
     *
     * gpio_pin_set_dt(&m_PsramVddCtrl, 1);
     * k_msleep(1);
     */
}
