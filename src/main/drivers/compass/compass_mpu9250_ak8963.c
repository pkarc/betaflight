/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_MPU9250_AK8963

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "drivers/compass/compass.h"

#include "drivers/compass/compass_mpu9250_ak8963.h"

#include "scheduler/scheduler.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

// This sensor is also available also part of the MPU-9250 connected to the secondary I2C bus.

// AK8963, mag sensor address
#define AK8963_MAG_I2C_ADDRESS          0x0C
#define AK8963_DEVICE_ID                0x48

// Registers
#define AK8963_MAG_REG_WIA              0x00
#define AK8963_MAG_REG_INFO             0x01
#define AK8963_MAG_REG_ST1              0x02
#define AK8963_MAG_REG_HXL              0x03
#define AK8963_MAG_REG_HXH              0x04
#define AK8963_MAG_REG_HYL              0x05
#define AK8963_MAG_REG_HYH              0x06
#define AK8963_MAG_REG_HZL              0x07
#define AK8963_MAG_REG_HZH              0x08
#define AK8963_MAG_REG_ST2              0x09
#define AK8963_MAG_REG_CNTL1            0x0A
#define AK8963_MAG_REG_CNTL2            0x0B
#define AK8963_MAG_REG_ASCT             0x0C // self test
#define AK8963_MAG_REG_I2CDIS           0x0F
#define AK8963_MAG_REG_ASAX             0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAY             0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAZ             0x12 // Fuse ROM z-axis sensitivity adjustment value

#define MPU9250_ADDRESS                 0x68
#define MPU9250_REG_USER_CTRL           0x6A
#define MPU9250_BIT_I2C_MST_DIS         0x00
#define MPU9250_BIT_SIG_COND_RST        0x01
#define MPU9250_REG_SIG_PATH_RST        0x68
#define MPU9250_BIT_GYRO_RST            0x04
#define MPU9250_BIT_ACCL_RST            0x02
#define MPU9250_BIT_TEMP_RST            0x01
#define MPU9250_REG_INT_PIN_CFG         0x37
#define MPU9250_BIT_BYPASS_EN           0x02
#define MPU9250_BIT_INT_ANYRD_2CLEAR    0x10
#define MPU9250_REG_PWR_MGMT_1          0x6B
#define MPU9250_BIT_H_RESET             0x80
#define MPU9250_BIT_CLEAR_SLEEP         0x00
#define MPU9250_BIT_AUTO_PLL            0x01
#define MPU9250_REG_PWR_MGMT_2          0x6C
#define MPU9250_BIT_DIS_XYZA            0x38
#define MPU9250_BIT_DIS_XYZG            0x07
#define MPU9250_REG_I2C_MST_CTRL        0x24
#define MPU9250_BIT_I2C_MST_CLK_400     0x0D
#define MPU9250_REG_WIA                 0x75
#define MPU9250_DEVICE_ID               0x71

#define READ_FLAG                       0x80
#define I2C_SLV0_EN                     0x80    

#define ST1_DATA_READY                  0x01
#define ST1_DATA_OVERRUN                0x02

#define ST2_MAG_SENSOR_OVERFLOW         0x08

#define CNTL1_MODE_POWER_DOWN           0x00
#define CNTL1_MODE_ONCE                 0x01
#define CNTL1_MODE_CONT1                0x02
#define CNTL1_MODE_CONT2                0x06
#define CNTL1_MODE_SELF_TEST            0x08
#define CNTL1_MODE_FUSE_ROM             0x0F
#define CNTL1_BIT_14_BIT                0x00
#define CNTL1_BIT_16_BIT                0x10

#define CNTL2_SOFT_RESET                0x01

#define I2CDIS_DISABLE_MASK             0x1D

busDevice_t mpu9250DeviceInstance;
busDevice_t *mpu9250Busdev = &mpu9250DeviceInstance;

static int16_t parseMag(uint8_t *raw, int16_t gain) {
  int ret = (int16_t)(raw[1] << 8 | raw[0]) * gain / 256;
  return constrain(ret, INT16_MIN, INT16_MAX);
}

bool mpu9250CheckAndEnableBypass()
{

    uint8_t bypass_cfg = 0;
    bool ack = busReadRegisterBuffer(mpu9250Busdev, MPU9250_REG_INT_PIN_CFG, &bypass_cfg, 1);               // check for MPU9250

#ifdef USE_CLI_DEBUG_PRINT
        cliDebugPrintLinef("mpu bypass ack %d", ack);
        cliDebugPrintLinef("MPU9250_REG_INT_PIN_CFG    0x%X", bypass_cfg);
#endif

    if (!ack) {
        return false;
    }

    if(bypass_cfg == MPU9250_BIT_BYPASS_EN){
        return true;
    }

#ifdef USE_CLI_DEBUG_PRINT
    cliDebugPrintLine("resetting mpu9250 ...");
#endif
    //Reset
    //busWriteRegister(mpu9250Busdev, MPU9250_REG_SIG_PATH_RST, MPU9250_BIT_GYRO_RST | MPU9250_BIT_ACCL_RST | MPU9250_BIT_TEMP_RST);
    //busWriteRegister(mpu9250Busdev, MPU9250_REG_USER_CTRL, MPU9250_BIT_SIG_COND_RST);
    //busWriteRegister(mpu9250Busdev, MPU9250_REG_PWR_MGMT_1, MPU9250_BIT_H_RESET);
    //busWriteRegister(mpu9250Busdev, MPU9250_REG_PWR_MGMT_1, MPU9250_BIT_CLEAR_SLEEP);
    //delay(10);
    //busWriteRegister(mpu9250Busdev, MPU9250_REG_PWR_MGMT_1, MPU9250_BIT_AUTO_PLL);
    //delay(10);
    //busWriteRegister(mpu9250Busdev, MPU9250_REG_I2C_MST_CTRL, MPU9250_BIT_I2C_MST_CLK_400);
    //delay(10);
#ifdef USE_CLI_DEBUG_PRINT
    cliDebugPrintLine("enable mpu9250 bypass ...");
#endif
    busWriteRegister(mpu9250Busdev, MPU9250_REG_USER_CTRL, MPU9250_BIT_I2C_MST_DIS);
    //busWriteRegister(mpu9250Busdev, MPU9250_REG_PWR_MGMT_2, MPU9250_BIT_DIS_XYZA | MPU9250_BIT_DIS_XYZG);
    busWriteRegister(mpu9250Busdev, MPU9250_REG_INT_PIN_CFG, MPU9250_BIT_BYPASS_EN);
    delay(1);

    return true;

}

static bool mpu9250ak8963Read(magDev_t *mag, int16_t *magData)
{

    bool ack = false;

    const busDevice_t *busdev = &mag->busdev;
    
    uint8_t status;

    /*
    if(!mpu9250CheckAndEnableBypass()){
        return false;
    }
    */

   if (busBusy(busdev, NULL)) {
        return false;
    }

    ack = busReadRegisterBuffer(busdev, AK8963_MAG_REG_ST1, &status, 1);

    if (!ack || (status & ST1_DATA_READY) == 0) {
        // too early. queue the status read again
        return false;
    }

    uint8_t buf[7];

    ack = busReadRegisterBuffer(busdev, AK8963_MAG_REG_HXL, buf, 7);

    if (!ack) {
        return false;
    }

    uint8_t status2 = buf[6];

    // start reading again    uint8_t status2 = buf[6];
    //busWriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_ONCE);

    if (status2 & ST2_MAG_SENSOR_OVERFLOW) {
        return false;
    }

    magData[X] = parseMag(buf + 0, mag->magGain[X]);
    magData[Y] = parseMag(buf + 2, mag->magGain[Y]);
    magData[Z] = parseMag(buf + 4, mag->magGain[Z]);

    return true;
}

static bool mpu9250ak8963Init(magDev_t *mag)
{
    uint8_t asa[3];
    uint8_t status1, status2;

    const busDevice_t *busdev = &mag->busdev;

    busWriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN);               // power down before entering fuse mode
    //delay(1);
    busWriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_FUSE_ROM);                 // Enter Fuse ROM access mode
    //delay(1);
    busReadRegisterBuffer(busdev, AK8963_MAG_REG_ASAX, asa, sizeof(asa));                // Read the x-, y-, and z-axis calibration values

    mag->magGain[X] = asa[X] + 128;
    mag->magGain[Y] = asa[Y] + 128;
    mag->magGain[Z] = asa[Z] + 128;

    busWriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN);               // power down after reading.
    //delay(1);

    // Clear status registers
    busReadRegisterBuffer(busdev, AK8963_MAG_REG_ST1, &status1, 1);

    //cliDebugPrintLinef("AK8963_MAG_REG_ST1    0x%X", status1);

    busReadRegisterBuffer(busdev, AK8963_MAG_REG_ST2, &status2, 1);

    //cliDebugPrintLinef("AK8963_MAG_REG_ST2    0x%X", status2);

    // Trigger first measurement
    //busWriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_CONT1);
    busWriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_CONT2);
    //busWriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_ONCE);
    //delay(4);

    return true;
}

void mpu9259ak8963BusDeInit(const busDevice_t *busdev)
{
    UNUSED(busdev);
}

#define DETECTION_MAX_RETRY_COUNT   5

static bool ak8963Detect(busDevice_t *busdev)
{
#ifdef USE_CLI_DEBUG_PRINT
    cliDebugPrintLine("detecting ak8963 ...");
#endif

    //busWriteRegister(busdev, AK8963_MAG_REG_CNTL2, CNTL2_SOFT_RESET);                    // reset MAG
    //delay(4);

    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        delay(10);

        uint8_t sig = 0;
        bool ack = busReadRegisterBuffer(busdev, AK8963_MAG_REG_WIA, &sig, 1);               // check for AK8963

#ifdef USE_CLI_DEBUG_PRINT
        cliDebugPrintLinef("ak8963 detect ack %d", ack);
        cliDebugPrintLinef("AK8963_MAG_REG_WIA    0x%X", sig);
#endif

        if (ack && sig == AK8963_DEVICE_ID) {
            return true;
        }
    }

    return false;
}

static bool mpu9250Detect(busDevice_t *busdev)
{

#ifdef USE_CLI_DEBUG_PRINT
    cliDebugPrintLine("detecting mpu5920...");
#endif

    mpu9250Busdev->bustype = busdev->bustype;
    mpu9250Busdev->busdev_u.i2c.device = busdev->busdev_u.i2c.device;
    mpu9250Busdev->busdev_u.i2c.address = MPU9250_ADDRESS;

    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        delay(10);

        uint8_t sig = 0;
        bool ack = busReadRegisterBuffer(mpu9250Busdev, MPU9250_REG_WIA, &sig, 1);               // check for MPU9250

#ifdef USE_CLI_DEBUG_PRINT
        cliDebugPrintLinef("mpu5920 detect ack %d", ack);
        cliDebugPrintLinef("MPU9250_REG_WIA    0x%X", sig);
#endif

        if (ack && sig == MPU9250_DEVICE_ID) {
            return true;
        }
    }

    return false;
}

bool mpu9250ak8963Detect(magDev_t *mag)
{

    busDevice_t *busdev = &mag->busdev;

    if (!mpu9250Detect(busdev)) {
        mpu9259ak8963BusDeInit(busdev);
        return false;
    }

    if(!mpu9250CheckAndEnableBypass()){
        mpu9259ak8963BusDeInit(busdev);
        return false;
    }

    if (!ak8963Detect(busdev)) {
        mpu9259ak8963BusDeInit(busdev);
        return false;
    }

    mag->init = mpu9250ak8963Init;
    mag->read = mpu9250ak8963Read;

    return true;
}
#endif
