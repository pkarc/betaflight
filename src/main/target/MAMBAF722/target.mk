F7X2RE_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8963.c \
            drivers/max7456.c