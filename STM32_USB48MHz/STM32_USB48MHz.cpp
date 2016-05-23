// STM32_USB48MHz.cpp 2015/6/20
#include "STM32_USB48MHz.h"

#if defined(TARGET_STM32F1)
bool HSE_SystemClock_Config(void) { // STM32F103RB
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return false;
    }
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        return false;
    }
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        return false;
    }
    return true;
}

bool HSI_SystemClock_Config(void) { // STM32F103RB
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return false;
    }
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        return false;
    }
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        return false;
    }
    return true;
}

uint32_t STM32_getUSBclock() { // STM32F1
    RCC_OscInitTypeDef RCC_OscInitStruct;
    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
    uint32_t src = 0;
    switch(RCC_OscInitStruct.PLL.PLLSource) {
        case RCC_PLLSOURCE_HSI_DIV2:
            src = HSI_VALUE / 2;
            break;
        case RCC_PLLSOURCE_HSE:
            src = HSE_VALUE;
            switch(RCC_OscInitStruct.HSEPredivValue) {
                case RCC_HSE_PREDIV_DIV1: src /= 1; break;
                case RCC_HSE_PREDIV_DIV2: src /= 2; break;
            }
            break;
    }
    switch(RCC_OscInitStruct.PLL.PLLMUL) {
        case RCC_PLL_MUL2: src *= 2; break;
        case RCC_PLL_MUL3: src *= 3; break;
        case RCC_PLL_MUL4: src *= 4; break;
        case RCC_PLL_MUL6: src *= 6; break;
        case RCC_PLL_MUL8: src *= 8; break;
        case RCC_PLL_MUL12: src *= 12; break;
        case RCC_PLL_MUL13: src *= 13; break;
        case RCC_PLL_MUL14: src *= 14; break;
        case RCC_PLL_MUL15: src *= 15; break;
        case RCC_PLL_MUL16: src *= 16; break;
    }
    RCC_PeriphCLKInitTypeDef  PeriphClkInit;
    HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkInit);
    switch(PeriphClkInit.UsbClockSelection) {
        case RCC_USBPLLCLK_DIV1: src /= 1; break;
        case RCC_USBPLLCLK_DIV1_5: src = src * 2 / 3; break;
    }
    return src;
}

#elif defined(TARGET_STM32L1)
bool HSE_SystemClock_Config(void) { // STM32L152RE
    __PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return false;
    }
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        return false;
    }
    return true;
}

bool HSI_SystemClock_Config(void) { // STM32L152RE
    __PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return false;
    }
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        return false;
    }
    return true;
}

uint32_t STM32_getUSBclock() { // STM32L1
    RCC_OscInitTypeDef RCC_OscInit;
    HAL_RCC_GetOscConfig(&RCC_OscInit);
    uint32_t src = 0;
    switch(RCC_OscInit.PLL.PLLSource) {
        case RCC_PLLSOURCE_HSI: src = HSI_VALUE; break;
        case RCC_PLLSOURCE_HSE: src = HSE_VALUE; break;
    }
    switch(RCC_OscInit.PLL.PLLMUL) {
        case RCC_PLL_MUL3: src *= 3; break;
        case RCC_PLL_MUL4: src *= 4; break;
        case RCC_PLL_MUL6: src *= 6; break;
        case RCC_PLL_MUL8: src *= 8; break;
        case RCC_PLL_MUL12: src *= 12; break;
        case RCC_PLL_MUL16: src *= 16; break;
        case RCC_PLL_MUL24: src *= 24; break;
        case RCC_PLL_MUL32: src *= 32; break;
        case RCC_PLL_MUL48: src *= 48; break;
    }
    return src / 2;
}
#else
#error "target error"
#endif

bool STM32_HSE_USB48MHz() {
    HAL_RCC_DeInit();
    if (!HSE_SystemClock_Config()) {
        return false;
    }
    SystemCoreClockUpdate();
    return true;
}

bool STM32_HSI_USB48MHz() {
    HAL_RCC_DeInit();
    if (!HSI_SystemClock_Config()) {
        return false;
    }
    SystemCoreClockUpdate();
    return true;
}

bool STM32_USB48MHz() {
    HAL_RCC_DeInit();
    if (!HSE_SystemClock_Config()) {
        if (!HSI_SystemClock_Config()) {
            return false;
        }
    }
    SystemCoreClockUpdate();
    return true;
}


