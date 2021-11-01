################################################################################
#									       #
#     Shared variables:							       #
#	- PROJECT							       #
#	- DRIVERS							       #
#	- INCLUDE							       #
#	- PLATFORM_DRIVERS						       #
#	- NO-OS								       #
#									       #
################################################################################
# app
SRCS += $(DRIVERS)/rf-transceiver/navassa/adrv9002_init_data.c \
	$(DRIVERS)/rf-transceiver/navassa/adrv9002_conv.c \
	$(DRIVERS)/rf-transceiver/navassa/adrv9002.c \
        $(PROJECT)/src/app/axi_core_rxtx.c \
        $(PROJECT)/src/app/app_main.c \
        $(PROJECT)/src/app/main.c
INCS += $(DRIVERS)/rf-transceiver/navassa/adrv9002.h \
        $(DRIVERS)/rf-transceiver/navassa/devices/adrv9001/public/include/adi_adrv9001_profileutil.h \
	$(PROJECT)/src/app/ORxGainTable.h \
	$(PROJECT)/src/app/RxGainTable.h \
        $(PROJECT)/src/app/TxAttenTable.h \
        $(PROJECT)/src/app/axi_core_rxtx.h \
        $(PROJECT)/src/app/app_main.h
# hal
SRCS += $(PROJECT)/src/hal/no_os_platform.c \
        $(PROJECT)/src/hal/io.c \
        $(PROJECT)/src/hal/alt_cache/alt_cache.c
INCS += $(PROJECT)/src/hal/parameters.h \
	$(PROJECT)/src/hal/no_os_platform.h \
	$(PROJECT)/src/hal/adi_platform.h \
	$(PROJECT)/src/hal/adi_platform_types.h \
	$(PROJECT)/src/firmware/Navassa_EvaluationFw.h \
        $(PROJECT)/src/firmware/Navassa_Stream.h \
        $(PROJECT)/src/hal/alt_cache/alt_cache.h \
        $(PROJECT)/src/hal/alt_cache/alt_interrupt_common.h \
        $(PROJECT)/src/hal/alt_cache/alt_mmu.h \
        $(PROJECT)/src/hal/alt_cache/alt_sysmgr.h \
        $(PROJECT)/src/hal/alt_cache/hwlib.h \
        $(PROJECT)/src/hal/alt_cache/alt_int_device.h
# no-OS drivers
SRCS += $(PLATFORM_DRIVERS)/altera_gpio.c \
        $(PLATFORM_DRIVERS)/altera_spi.c \
	$(NO-OS)/drivers/gpio/gpio.c \
	$(DRIVERS)/spi/spi.c \
	$(PLATFORM_DRIVERS)/delay.c \
	$(NO-OS)/util/util.c \
	$(DRIVERS)/axi_core/axi_adc_core/axi_adc_core.c \
	$(DRIVERS)/axi_core/axi_dac_core/axi_dac_core.c \
	$(DRIVERS)/axi_core/axi_dmac/axi_dmac.c \
	$(PLATFORM_DRIVERS)/axi_io.c
INCS +=	$(INCLUDE)/spi.h \
	$(PLATFORM_DRIVERS)/spi_extra.h \
	$(INCLUDE)/gpio.h \
	$(PLATFORM_DRIVERS)/gpio_extra.h \
	$(INCLUDE)/error.h \
	$(INCLUDE)/delay.h \
	$(INCLUDE)/util.h \
	$(INCLUDE)/print_log.h \
	$(DRIVERS)/axi_core/axi_adc_core/axi_adc_core.h \
	$(DRIVERS)/axi_core/axi_dac_core/axi_dac_core.h \
	$(DRIVERS)/axi_core/axi_dmac/axi_dmac.h \
        $(INCLUDE)/axi_io.h \
        $(INCLUDE)/socal.h \
        $(INCLUDE)/altera_avalon_spi_regs.h \
        $(INCLUDE)/alt_types.h \
        $(INCLUDE)/hps.h \
        $(INCLUDE)/alt_uart.h

# Navassa API sources
SRC_DIRS += $(DRIVERS)/rf-transceiver/navassa

# IIO
ifeq (y,$(strip $(TINYIIOD)))
LIBRARIES += iio
SRCS += $(PROJECT)/src/app/app_iio.c \
	$(PLATFORM_DRIVERS)/uart.c \
	$(PLATFORM_DRIVERS)/irq.c \
	$(NO-OS)/util/list.c \
	$(NO-OS)/util/fifo.c \
	$(DRIVERS)/axi_core/iio_axi_adc/iio_axi_adc.c \
	$(DRIVERS)/axi_core/iio_axi_dac/iio_axi_dac.c
INCS += $(PROJECT)/src/app/app_iio.h \
	$(INCLUDE)/uart.h \
	$(INCLUDE)/irq.h \
	$(PLATFORM_DRIVERS)/irq_extra.h \
	$(PLATFORM_DRIVERS)/uart_extra.h \
	$(INCLUDE)/fifo.h \
	$(INCLUDE)/list.h \
	$(DRIVERS)/axi_core/iio_axi_adc/iio_axi_adc.h \
	$(DRIVERS)/axi_core/iio_axi_dac/iio_axi_dac.h
endif
