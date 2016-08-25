import qbs

CppApplication {
    consoleApplication: true
    cpp.defines: [
        "TOOLCHAIN_GCC_CW=1",
        "STM32F407xx",
        "USE_HAL_DRIVER"
    ]
    cpp.positionIndependentCode: false
    cpp.debugInformation: true
    cpp.commonCompilerFlags: [
        "-mthumb","-mcpu=cortex-m4","-mfloat-abi=hard","-mfpu=fpv4-sp-d16",
        //"-flto",
        "-fdata-sections",
        "-ffunction-sections",
        "-fno-inline"
    ]
    cpp.cxxLanguageVersion: "c++14"
    cpp.includePaths: [
        "../Inc",
        "../Drivers/CMSIS/Include",
        "../Drivers/STM32F4xx_HAL_Driver/Inc",
        "../Drivers/CMSIS/Device/ST/STM32F4xx/Include"
    ]

    files: [
        "../Src/main.cpp",
        "../Src/*.c",
        "../Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c",
        "../Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c",
        "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
    ]
    cpp.linkerFlags:[
        "-flto","-mthumb","-mcpu=cortex-m4","-mfloat-abi=hard","-mfpu=fpv4-sp-d16","--specs=nano.specs","-Wl,--start-group",
        "-Wl,--gc-sections","-T",path+"/STM32F4_flash.ld","-lnosys","-lgcc","-lc"
    ]


    Group {     // Properties for the produced executable
        fileTagsFilter: product.type
        qbs.install: true
    }
}
