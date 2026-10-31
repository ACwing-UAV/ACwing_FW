/**
 * @file  usbd_conf.c
 * @brief Low-level glue between the ST USB device library and the HAL PCD
 *        driver of the STM32WB55 (USB FS device, PA11/PA12). Written after
 *        the ST template usbd_conf_template.c and the STM32CubeWB CDC
 *        examples.
 *
 * Clocking: the USB peripheral runs from HSI48, trimmed by the CRS on the
 * USB start-of-frame (no crystal needed). On the WB55, CPU2 also uses HSI48
 * (RNG): CPU1 holds hardware semaphore CFG_HW_CLK48_CONFIG_SEMID while USB is
 * on, so CPU2 never switches HSI48 off (AN5289 §4.7).
 */
#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "hw_conf.h"
#include "stm32wbxx_ll_crs.h"
#include "stm32wbxx_ll_rcc.h"
#include "stm32wbxx_ll_hsem.h"
#include "stm32wbxx_ll_bus.h"

PCD_HandleTypeDef hpcd_USB_FS;

/* ------------------------------------------------------------------ clocks */
static void clk48_on(void)
{
    /* keep HSI48 on for us: CPU2 checks this semaphore before turning it off */
    while (LL_HSEM_1StepLock(HSEM, CFG_HW_CLK48_CONFIG_SEMID)) { }
    while (LL_HSEM_1StepLock(HSEM, CFG_HW_RCC_SEMID)) { }
    LL_RCC_HSI48_Enable();
    while (!LL_RCC_HSI48_IsReady()) { }
    LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_HSI48);
    LL_HSEM_ReleaseLock(HSEM, CFG_HW_RCC_SEMID, 0);

    /* CRS: trim HSI48 on the 1 kHz USB SOF */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CRS);
    LL_CRS_SetSyncDivider(LL_CRS_SYNC_DIV_1);
    LL_CRS_SetSyncPolarity(LL_CRS_SYNC_POLARITY_RISING);
    LL_CRS_SetSyncSignalSource(LL_CRS_SYNC_SOURCE_USB);
    LL_CRS_SetReloadCounter(__LL_CRS_CALC_CALCULATE_RELOADVALUE(48000000u, 1000u));
    LL_CRS_SetFreqErrorLimit(34u);
    LL_CRS_EnableAutoTrimming();
    LL_CRS_EnableFreqErrorCounter();
}

/* ------------------------------------------------------------------ MSP */
void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd)
{
    if (pcd->Instance != USB) return;
    /* PA11/PA12 are set to AF10 (USB) by MX_GPIO_Init */
    clk48_on();
    __HAL_RCC_USB_CLK_ENABLE();
    HAL_PWREx_EnableVddUSB();
    /* below configMAX_SYSCALL_INTERRUPT_PRIORITY: the callbacks use no RTOS call, but keep it safe */
    HAL_NVIC_SetPriority(USB_LP_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcd)
{
    if (pcd->Instance != USB) return;
    __HAL_RCC_USB_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(USB_LP_IRQn);
}

void USB_LP_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_FS); }

/* ------------------------------------------------------------------ PCD -> USBD */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *pcd)          { USBD_LL_SetupStage((USBD_HandleTypeDef *)pcd->pData, (uint8_t *)pcd->Setup); }
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *pcd, uint8_t ep) { USBD_LL_DataOutStage((USBD_HandleTypeDef *)pcd->pData, ep, pcd->OUT_ep[ep].xfer_buff); }
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *pcd, uint8_t ep)  { USBD_LL_DataInStage((USBD_HandleTypeDef *)pcd->pData, ep, pcd->IN_ep[ep].xfer_buff); }
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *pcd)                  { USBD_LL_SOF((USBD_HandleTypeDef *)pcd->pData); }
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *pcd)
{
    USBD_LL_SetSpeed((USBD_HandleTypeDef *)pcd->pData, USBD_SPEED_FULL);
    USBD_LL_Reset((USBD_HandleTypeDef *)pcd->pData);
}
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *pcd)              { USBD_LL_Suspend((USBD_HandleTypeDef *)pcd->pData); }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *pcd)               { USBD_LL_Resume((USBD_HandleTypeDef *)pcd->pData); }
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t ep) { USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)pcd->pData, ep); }
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t ep)  { USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)pcd->pData, ep); }
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *pcd)              { USBD_LL_DevConnected((USBD_HandleTypeDef *)pcd->pData); }
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *pcd)           { USBD_LL_DevDisconnected((USBD_HandleTypeDef *)pcd->pData); }

/* ------------------------------------------------------------------ USBD_LL */
static USBD_StatusTypeDef st(HAL_StatusTypeDef h)
{
    return h == HAL_OK ? USBD_OK : (h == HAL_BUSY ? USBD_BUSY : USBD_FAIL);
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_FS.pData = pdev;
    pdev->pData = &hpcd_USB_FS;
    hpcd_USB_FS.Instance = USB;
    hpcd_USB_FS.Init.dev_endpoints = 8;
    hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) return USBD_FAIL;

    /* packet memory: 8 × 8-byte BTABLE, then the endpoint buffers */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00, PCD_SNG_BUF, 0x18);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80, PCD_SNG_BUF, 0x58);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_IN_EP,  PCD_SNG_BUF, 0xC0);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_OUT_EP, PCD_SNG_BUF, 0x110);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_CMD_EP, PCD_SNG_BUF, 0x100);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)  { return st(HAL_PCD_DeInit((PCD_HandleTypeDef *)pdev->pData)); }
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)   { return st(HAL_PCD_Start((PCD_HandleTypeDef *)pdev->pData)); }
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)    { return st(HAL_PCD_Stop((PCD_HandleTypeDef *)pdev->pData)); }

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep, uint8_t type, uint16_t mps)
{
    return st(HAL_PCD_EP_Open((PCD_HandleTypeDef *)pdev->pData, ep, mps, type));
}
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep)      { return st(HAL_PCD_EP_Close((PCD_HandleTypeDef *)pdev->pData, ep)); }
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep)      { return st(HAL_PCD_EP_Flush((PCD_HandleTypeDef *)pdev->pData, ep)); }
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep)      { return st(HAL_PCD_EP_SetStall((PCD_HandleTypeDef *)pdev->pData, ep)); }
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep) { return st(HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *)pdev->pData, ep)); }

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    PCD_HandleTypeDef *h = (PCD_HandleTypeDef *)pdev->pData;
    return (ep & 0x80u) ? h->IN_ep[ep & 0x7Fu].is_stall : h->OUT_ep[ep & 0x7Fu].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t addr)
{
    return st(HAL_PCD_SetAddress((PCD_HandleTypeDef *)pdev->pData, addr));
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep, uint8_t *buf, uint32_t size)
{
    return st(HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)pdev->pData, ep, buf, size));
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep, uint8_t *buf, uint32_t size)
{
    return st(HAL_PCD_EP_Receive((PCD_HandleTypeDef *)pdev->pData, ep, buf, size));
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep);
}

void USBD_LL_Delay(uint32_t ms) { HAL_Delay(ms); }

/* ------------------------------------------------------------------ static memory */
void *USBD_static_malloc(uint32_t size)
{
    static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef) / 4u) + 1u];
    return size <= sizeof mem ? mem : NULL;
}
void USBD_static_free(void *p) { (void)p; }
