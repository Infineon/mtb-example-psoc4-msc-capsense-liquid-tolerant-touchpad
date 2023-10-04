/******************************************************************************
 * File Name: main.c
 *
 * Description: This is the source code for the PSoC 4 MSC CAPSENSE™ Liquid
 * tolerant touchpad code example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "stdio.h"
#include "string.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/
 /* CAPSENSE interrupt priority */
#define CAPSENSE_MSC0_INTR_PRIORITY      (3U)
#define CAPSENSE_MSC1_INTR_PRIORITY      (3U)
#define CY_ASSERT_FAILED                 (0U)
#define MSC_CAPSENSE_WIDGET_INACTIVE     (0U)

/* EZI2C interrupt priority must be higher than CapSense interrupt. */
#define EZI2C_INTR_PRIORITY        (2U)

/* ILO Frequency in Hz */
#define ILO_FREQUENCY_HZ           (40000U)

/* WDT interrupt priority */
#define WDT_INTERRUPT_PRIORITY     (3U)

/* Gesture Macros */
#define SINGLE_CLICK            (0x0001U)
#define DOUBLE_CLICK            (0x0002U)
#define SCROLL_RIGHT            (0x00020010U)
#define SCROLL_LEFT             (0x00030010U)
#define SCROLL_UP               (0x000010U)
#define SCROLL_DOWN             (0x010010U)
#define FLICK_UP                (0x00000080U)
#define FLICK_DOWN              (0x010000080U)
#define FLICK_RIGHT             (0x02000080U)
#define FLICK_LEFT              (0x03000080U)
#define TWO_FINGER_ZOOM_IN      (0x00000200U)
#define TWO_FINGER_ZOOM_OUT     (0x00800200U)
#define TWO_FINGER_CLICK        (0x00000008U)

/* Delays */
#define DELAY_MS    (5U) /* in ms */

/*******************************************************************************
 * Global Definitions
 *******************************************************************************/
 /* EZI2C slave context structure */
cy_stc_scb_ezi2c_context_t ezi2c_context;

/* UART context structure */
cy_stc_scb_uart_context_t scb_1_context;

/* WDT interrupt service routine configuration */
const cy_stc_sysint_t wdt_isr_cfg =
{
    .intrSrc = srss_interrupt_wdt_IRQn,    /* Interrupt source is WDT interrupt */
    .intrPriority = WDT_INTERRUPT_PRIORITY /* Interrupt priority is 0 */
};

/* Variable to check whether WDT interrupt is triggered */
bool flag = false;

/* Variable to store the counts required after ILO compensation */
static uint32_t ilo_cycles = 0U;
static uint32_t ilo_compensated_counts = 0U;
static uint32_t DESIRED_WDT_INTERVAL_MS = 100000U;

/* Variable to check the state of an LED */
uint16_t bright = 100U;
uint8_t state = 0U;
uint16_t inc;
uint16_t dec;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
/* CAPSENSE functions */ 
static void initialize_capsense(void);
static void capsense_msc0_isr(void);
static void capsense_msc1_isr(void);
static void initialize_capsense_tuner(void);

/* EZ-I2C ISR */
static void ezi2c_isr(void);

/* PWM functions to control led brightness */ 
void incr_brightness(uint16_t inc);
void decr_brightness(uint16_t dec);
void toggle_pwm(void);

/* WDT function */ 
void wdt_isr(void); /* WDT interrupt service routine */
void wdt_trigger(void);
cy_en_syspm_status_t deep_sleep_callback(
    cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode);

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CapSense
 *  - initialize tuner communication
 *  - scan touch input continuously
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* This variable is used to store the string of characters to be displayed on UART Terminal */
    char buf[10] = " ";

    /* This varible is used to store timestamp value */
    uint32_t user_time_stamp = 2U;

    /* Proximity touch state */
    uint8_t proximity_state = 0U;

    /* This variable is used to implement software counter */
    uint8_t soft_counter = 0U;

    /* variable to store decode values */
    uint32_t gest = 0U, lgest = 0U;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initializes the SCB for debug UART port */
    result = Cy_SCB_UART_Init(SCB1, &scb_1_config, &scb_1_context);

    /* SCB UART init failed. Stop program execution */
    if (result != CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enables the SCB block for the UART operation */
    Cy_SCB_UART_Enable(SCB1);

    /* Initialize timestamp for gestures */
    Cy_CapSense_SetGestureTimestamp(user_time_stamp, &cy_capsense_context);

    /* Print Charaters on UART Terminal */
    Cy_SCB_UART_PutString(SCB1, "Touchpad 10x16\r\n");

    /* Initialize EZI2C */
    initialize_capsense_tuner();

    /* Initialize MSC CapSense */
    initialize_capsense();

    /* Initialize the TCPWM block */
    Cy_TCPWM_PWM_Init(pwm2_HW, pwm2_NUM, &pwm2_config);
    
    /* Enable the TCPWM block */
    Cy_TCPWM_PWM_Enable(pwm2_HW, pwm2_NUM);
    
    /* Triggers a software start on the selected TCPWMs */
    Cy_TCPWM_TriggerStart(pwm2_HW, pwm2_MASK);

    /* Configure the interrupt with a vector at Wdt_Isr(). */
    cy_en_sysint_status_t sysintStatus = Cy_SysInt_Init(&wdt_isr_cfg, wdt_isr);
    
    if (CY_SYSINT_SUCCESS != sysintStatus)
    {
        /* Insert the error handling here */
        CY_ASSERT(0);
    }
    
    NVIC_EnableIRQ(wdt_isr_cfg.intrSrc); /* Enable the interrupt. */
    
    /*This function initializes the WDT block*/
    Cy_WDT_Init();
    
    /* Enable the ILO */
    Cy_SysClk_IloEnable();
    
    /* Now switch the WDC timers clocking to ILO */
    /* Disable the WCO */
    Cy_SysClk_WcoDisable();
    
    /* Enable WDT */
    Cy_WDT_Enable();
    
    /* Unmask the WDT interrupt */
    Cy_WDT_UnmaskInterrupt();

    /* Callback parameters for EzI2C */
    cy_stc_syspm_callback_params_t ezi2cCallbackParams =
    {
        .base       = SCB0,
        .context    = &ezi2c_context
    };

    /* Callback declaration for EzI2C Deep Sleep callback */
    cy_stc_syspm_callback_t ezi2cCallback =
    {
        .callback       = (Cy_SysPmCallback)&Cy_SCB_EZI2C_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &ezi2cCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 0
    };

    /* SysPm callback params */
    cy_stc_syspm_callback_params_t sysClkCallbackParams =
    {
        .base       = CYBSP_MSC0_HW,
        .context    = &cy_capsense_context
    };

    /* SysPm callback params */
    cy_stc_syspm_callback_params_t sysClkCallbackParams1 =
    {
        .base       = CYBSP_MSC1_HW,
        .context    = &cy_capsense_context
    };

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback =
    {
        .callback       = &deep_sleep_callback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 1
    };

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback1 =
    {
        .callback       = &deep_sleep_callback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams1,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 1
    };

    /* Register EzI2C Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&ezi2cCallback);
    
    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback);
    
    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback1);

    for (;;)
    {
        /* WDT interrupt source */
        wdt_trigger();

        if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Check the system state */
            if(proximity_state == 0)
            {
                /* 100MS Deep-Sleep state, scan only proximity sensor */
                Cy_CapSense_ScanSlots(cy_capsense_context.ptrWdConfig[CY_CAPSENSE_PROXIMITY0_WDGT_ID].firstSlotId,
                    cy_capsense_context.ptrWdConfig[CY_CAPSENSE_PROXIMITY0_WDGT_ID].numSlots, &cy_capsense_context);

                /* Process only the proximity widget */
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_PROXIMITY0_WDGT_ID, &cy_capsense_context);

                /* Check if proximity sensor is active */
                proximity_state = Cy_CapSense_IsProximitySensorActive(CY_CAPSENSE_PROXIMITY0_WDGT_ID, CY_CAPSENSE_PROXIMITY0_SNS0_ID, &cy_capsense_context);

            }

            if(proximity_state)
            {
                /* Configure the wake up period to 10ms */
                DESIRED_WDT_INTERVAL_MS = 10000U;
                
                /* Initialize baselines */
                Cy_CapSense_InitializeWidgetBaseline(CY_CAPSENSE_PROXIMITY0_WDGT_ID, &cy_capsense_context);

                /* Scan touchpad widget */
                Cy_CapSense_ScanSlots(cy_capsense_context.ptrWdConfig[CY_CAPSENSE_TOUCHPAD0_WDGT_ID].firstSlotId,
                    cy_capsense_context.ptrWdConfig[CY_CAPSENSE_TOUCHPAD0_WDGT_ID].numSlots, &cy_capsense_context);

                /* increment the timestamp register */
                Cy_CapSense_IncrementGestureTimestamp(&cy_capsense_context);

                /* Process only the touchpad widget */
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_TOUCHPAD0_WDGT_ID, &cy_capsense_context);

                /* decode all the gestures */
                gest = Cy_CapSense_DecodeWidgetGestures(CY_CAPSENSE_TOUCHPAD0_WDGT_ID, &cy_capsense_context);

                if (gest != lgest)
                {
                    if (gest > 0U)
                    {
                        switch (gest)
                        {
                        case SINGLE_CLICK : Cy_SCB_UART_PutString(SCB1, "Single Click \r\n"); /* Print Charaters on UART Terminal */
                        toggle_pwm(); /* Turns LED ON/OFF based on touch status */
                        break;

                        case DOUBLE_CLICK : Cy_SCB_UART_PutString(SCB1, "Double Click \r\n");
                        toggle_pwm();
                        break;

                        case SCROLL_DOWN :Cy_SCB_UART_PutString(SCB1, "Scroll Down \r\n");
                        decr_brightness(bright); /* Decreases brightness of an LED */
                        break;

                        case SCROLL_UP :Cy_SCB_UART_PutString(SCB1, "Scroll up \r\n");
                        incr_brightness(bright); /* Increases brightness of an LED */
                        break;

                        case SCROLL_RIGHT : Cy_SCB_UART_PutString(SCB1, "Scroll right \r\n");
                        incr_brightness(bright); 
                        break;

                        case SCROLL_LEFT : Cy_SCB_UART_PutString(SCB1, "Scroll left \r\n");
                        decr_brightness(bright); 
                        break;

                        case FLICK_UP: Cy_SCB_UART_PutString(SCB1, "flick up \r\n");
                        incr_brightness(bright); 
                        break;

                        case FLICK_DOWN : Cy_SCB_UART_PutString(SCB1, "flick down \r\n");
                        decr_brightness(bright); 
                        break;

                        case FLICK_RIGHT : Cy_SCB_UART_PutString(SCB1, "flick right \r\n");
                        incr_brightness(bright); 
                        break;

                        case FLICK_LEFT : Cy_SCB_UART_PutString(SCB1, "flick left \r\n");
                        decr_brightness(bright);
                        break;

                        case TWO_FINGER_CLICK : Cy_SCB_UART_PutString(SCB1, "Two Finger Click \r\n");
                        Cy_GPIO_Inv(CYBSP_USER_LED3_PORT, CYBSP_USER_LED3_NUM); /* Invert LED3 (out value = ~(out value)) */
                        break;

                        case TWO_FINGER_ZOOM_OUT : Cy_SCB_UART_PutString(SCB1, "Two Finger Zoom OUT \r\n");
                        decr_brightness(bright);
                        break;

                        case TWO_FINGER_ZOOM_IN : Cy_SCB_UART_PutString(SCB1, "Two Finger Zoom In \r\n");
                        incr_brightness(bright);
                        break;

                        default :
                            sprintf(buf, "%lx", (unsigned long)gest); /* Sends formatted output to a string */
                            Cy_SCB_UART_PutString(SCB1, buf); /* Print buffer value on UART Terminal */
                            Cy_SCB_UART_PutString(SCB1, "\r\n"); /* Print Charaters on UART Terminal */

                            break;
                        }
                    }

                    /* Reset varibles */
                    soft_counter = 0;
                    lgest = gest;
                }

                /* If the CapSense touch is inactive, increment the software counter */
                if(gest == 0)
                {
                    soft_counter++;
                }

                if(soft_counter > 100)
                {
                    /* CapSense touch is inactive. Increase Deep-Sleep duration */
                    DESIRED_WDT_INTERVAL_MS = 100000U;

                    /* Reset counter */
                    soft_counter = 0;

                    /* Clear Proximity Sensor status */
                    proximity_state = 0;
                }
            }

            /* Establishes synchronized communication with the CapSense Tuner tool */
            Cy_CapSense_RunTuner(&cy_capsense_context);
        }
    }
}

/*****************************************************************************
 * Function Name: wdt_isr
 ******************************************************************************
 * Summary:
 * This function is the handler for the WDT interrupt
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *****************************************************************************/
void wdt_isr(void)
{
    /* Clears the WDT match flag */
    Cy_WDT_ClearInterrupt();
    
    /* Set the interrupt flag */
    flag = true;
}

/*******************************************************************************
 * Function Name: wdt_trigger
 ********************************************************************************
 * Summary:
 *  Updates the set match value to the WDT block.
 *  Enters into deep sleep mode.
 *
 * Return:
 *  void
 *
 * Parameters:
 * void
 *
 *******************************************************************************/
void wdt_trigger(void){

    if (flag)
    {
        /* Clear the interrupt flag */
        flag = false;

        /* Update the match count  */
        Cy_WDT_SetMatch((uint16_t)(ilo_compensated_counts + Cy_WDT_GetMatch())); /* Program the next WDT event */
    }

    /* Start ILO measurement */
    Cy_SysClk_IloStartMeasurement();

    /* Get the ILO compensated counts i.e. the actual counts for the
        desired ILO frequency. ILO default accuracy is +/- 60%.
        Note that DESIRED_WDT_INTERVAL should be less than the total
        count time */
    while (CY_SYSCLK_SUCCESS != Cy_SysClk_IloCompensate(DESIRED_WDT_INTERVAL_MS, &ilo_cycles)); /* The desired WDT delay in second */
    ilo_compensated_counts = (uint32_t)ilo_cycles;

    /* Stop ILO measurement before entering deep sleep mode */
    Cy_SysClk_IloStopMeasurement();

    /* Delay to empty the UART tx buffer */
    Cy_SysLib_Delay(DELAY_MS);

    /* Enter deep sleep mode */
    Cy_SysPm_CpuEnterDeepSleep();
}

/*******************************************************************************
 * Function Name: toggle_pwm
 ********************************************************************************
 * Summary:
 *  Function to process the touch. Depending on the command passed as parameter,
 *  new compare values are calculated.
 *
 *  The values are written to the respective buffer registers and a compare
 *  swap is issued.
 *
 *******************************************************************************/
void toggle_pwm()
{
    state = !state;

    if (state)
    {
        /* Modify the compare value here */
        Cy_TCPWM_PWM_SetCompare0(pwm2_HW, pwm2_NUM, bright);
        Cy_TCPWM_PWM_SetCompare1(pwm2_HW, pwm2_NUM, 1000U - bright);
    }

    else
    {
        /* Modify the compare value here */
        Cy_TCPWM_PWM_SetCompare0(pwm2_HW, pwm2_NUM, 0U);
        Cy_TCPWM_PWM_SetCompare1(pwm2_HW, pwm2_NUM, 1000U);
    }
}

/*******************************************************************************
 * Function Name: incr_brightness
 ********************************************************************************
 * Summary:
 *  Function to process the touch. Depending on the command passed as parameter,
 *  new compare values are calculated.
 *
 * The values are written to the respective buffer registers and compare is issued.
 *
 *******************************************************************************/
void incr_brightness(uint16_t inc)
{
    if(inc<1000U)
    {
        inc++;
        uint16_t compare = Cy_TCPWM_PWM_GetCompare0(pwm2_HW, pwm2_NUM); /* Get the currently existing period value */
        uint16_t new_compare = compare + inc;

        if(new_compare > Cy_TCPWM_PWM_GetPeriod0(pwm2_HW, pwm2_NUM)){
            new_compare = Cy_TCPWM_PWM_GetPeriod0(pwm2_HW, pwm2_NUM);
        }
        Cy_TCPWM_PWM_SetCompare0(pwm2_HW, pwm2_NUM, new_compare); /* Modify the compare value here */
    }
}

/*******************************************************************************
 * Function Name: decr_brightness
 ********************************************************************************
 * Summary:
 *  Function to process the touch. Depending on the command passed as parameter,
 *  new compare values are calculated.
 *
 *  The values are written to the respective buffer registers and a compare is issued.
 *
 *******************************************************************************/

void decr_brightness(uint16_t dec)
{
    uint16_t compare = Cy_TCPWM_PWM_GetCompare0(pwm2_HW, pwm2_NUM); /* Get the currently existing period value */
    uint16_t new_compare;

    if(compare >= dec){
        new_compare = compare - dec;
    }
    else
    {
        new_compare = 0;
    }
    Cy_TCPWM_PWM_SetCompare0(pwm2_HW, pwm2_NUM, new_compare); /* Modify the compare value here */
}

/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CapSense blocks and configures the CapSense
 *  interrupt.
 *
 *******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CapSense interrupt configuration MSC 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
        .intrSrc = CY_MSC0_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* CapSense interrupt configuration MSC 1 */
    const cy_stc_sysint_t capsense_msc1_interrupt_config =
    {
        .intrSrc = CY_MSC1_IRQ,
        .intrPriority = CAPSENSE_MSC1_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* CapSense initialization failed, the middleware may not operate
         * as expected, and repeating of initialization is required.*/
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CapSense interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        /* Initialize CapSense interrupt for MSC 1 */
        Cy_SysInt_Init(&capsense_msc1_interrupt_config, capsense_msc1_isr);
        NVIC_ClearPendingIRQ(capsense_msc1_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc1_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}

/*******************************************************************************
 * Function Name: capsense_msc0_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense MSC0 block.
 *
 *******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC0_HW, &cy_capsense_context);
}

/*******************************************************************************
 * Function Name: capsense_msc1_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense MSC1 block.
 *
 *******************************************************************************/
static void capsense_msc1_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC1_HW, &cy_capsense_context);
}

/*******************************************************************************
 * Function Name: initialize_capsense_tuner
 ********************************************************************************
 * Summary:
 *  EZI2C module to communicate with the CapSense Tuner tool.
 *
 *******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = CYBSP_EZI2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Initialize the interrupt */
    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    
    /* Enable the interrupt */
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CapSense data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
        sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
        &ezi2c_context);

    /* Enables the SCB block for the EZI2C operation */
    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);
}

/*******************************************************************************
 * Function Name: ezi2c_isr
 ********************************************************************************
 * Summary:
 * Wrapper function for handling interrupts from EZI2C block.
 *
 *******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}

/*******************************************************************************
 * Function Name: deep_sleep_callback
 ********************************************************************************
 * Summary:
 * Deep Sleep callback implementation. It changes PWM and UART status based on
 * the power state and MCU state.
 *
 * Parameters:
 *  callbackParams: The pointer to the callback parameters structure
 *  cy_stc_syspm_callback_params_t.
 *  mode: Callback mode, see cy_en_syspm_callback_mode_t
 *
 * Return:
 *  Entered status, see cy_en_syspm_status_t.
 *
 *******************************************************************************/
cy_en_syspm_status_t deep_sleep_callback(
    cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {    /* Check if the device is ready to enter the low power mode */
    case CY_SYSPM_CHECK_READY:

        while(Cy_SCB_UART_IsTxComplete(scb_1_HW) == 0U)
        {
            /* Wait until the TX FIFO
             * and Shifter are empty and there is no more data to send. */
        }

        /* Disable the UART */
        Cy_SCB_UART_Disable(scb_1_HW, &scb_1_context);

        ret_val = CY_SYSPM_SUCCESS;
        break;

        /* Roll back the actions performed in the previously executed callback with CY_SYSPM_CHECK_READY */
    case CY_SYSPM_CHECK_FAIL:

        /* Enable the UART */
        Cy_SCB_UART_Enable(scb_1_HW);

        ret_val = CY_SYSPM_SUCCESS;
        break;

        /* Performs the actions to be done before entering into the low power mode */
    case CY_SYSPM_BEFORE_TRANSITION:

        /* Disable the PWM */
        Cy_TCPWM_PWM_Disable(pwm2_HW, pwm2_NUM);

        ret_val = CY_SYSPM_SUCCESS;
        break;

        /* Performs the actions to be done after exiting the low power mode if entered */
    case CY_SYSPM_AFTER_TRANSITION:

        /* Enable and Start the PWM */
        Cy_TCPWM_PWM_Enable(pwm2_HW, pwm2_NUM);
        Cy_TCPWM_TriggerStart(pwm2_HW, pwm2_MASK);

        /* Enable the UART */
        Cy_SCB_UART_Enable(scb_1_HW);

        ret_val = CY_SYSPM_SUCCESS;
        break;

    default:
        /* Don't do anything in the other modes */
        ret_val = CY_SYSPM_SUCCESS;
        break;
    }
    return ret_val;
}

/* [] END OF FILE */
