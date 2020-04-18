/*
 * icm20649_reg.h
 *
 *  Created on: 2020年3月25日
 *      Author: Administrator
 */

#ifndef ICM20649_ICM20649_REG_H_
#define ICM20649_ICM20649_REG_H_

#define SPI_SPEED  (7 * 1000 * 1000) // 7 MHz SPI
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);


#define  FS_SEL_250_DPS 0
#define  FS_SEL_500_DPS	Bit3        // 0b01000
#define  FS_SEL_1000_DPS Bit4,        // 0b10000
#define  FS_SEL_2000_DPS (Bit4 | Bit3) // 0b11000
#define FCHOICE_B_8KHZ_BYPASS_DLPF (Bit1 | Bit0)

#define	GYRO_FS_SEL_500_DPS  0           // 0b00 = ±500 dps
#define	GYRO_FS_SEL_1000_DPS Bit1       // 0b01 = ±1000 dps
#define	GYRO_FS_SEL_2000_DPS Bit2        // 0b10 = ±2000 dps
#define	GYRO_FS_SEL_4000_DPS (Bit2 | Bit1) // 0b11 = ±4000 dps
#define	GYRO_FCHOICE         Bit0       // 0 – Bypass gyro DLPF

#define	ACCEL_FS_SEL_2G   0,           // 0b00000
#define	ACCEL_FS_SEL_4G   Bit3         // 0b01000
#define	ACCEL_FS_SEL_8G   Bit4         // 0b10000
#define	ACCEL_FS_SEL_16G  (Bit4 | Bit3) // 0b11000
#define ACCEL_FS_SEL_32G  (Bit2 | Bit1)

static constexpr float TEMPERATURE_SENSITIVITY = 326.8f; // LSB/C
static constexpr float ROOM_TEMPERATURE_OFFSET = 25.f; // C
static constexpr uint8_t DIR_READ = 0x80;
static constexpr uint8_t WHOAMI = 0xE1;

#define REG_BANK0 0x00U
#define REG_BANK1 0x01U
#define REG_BANK2 0x02U
#define REG_BANK3 0x03U


#define ICM20649_REG(b, r)      ((((uint16_t)b) << 8)|(r))
#define GET_BANK(r)         ((r) >> 8)
#define GET_REG(r)          ((r) & 0xFFU)


//Register Map
#define ICM20649_WHO_AM_I               ICM20649_REG(REG_BANK0,0x00U)
#define ICM20649_USER_CTRL              ICM20649_REG(REG_BANK0,0x03U)

#       define BIT_USER_CTRL_I2C_MST_RESET          0x02 // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_SRAM_RESET             0x04 // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET              0x08 // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS             0x10 // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN             0x20 // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                0x40 // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                 0x80     // Enable DMP operations

#define ICM20649_LP_CONFIG              ICM20649_REG(REG_BANK0,0x05U)
#define ICM20649_PWR_MGMT_1             ICM20649_REG(REG_BANK0,0x06U)
#       define BIT_PWR_MGMT_1_CLK_INTERNAL          0x00 // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_AUTO              0x01 // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_STOP              0x07 // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS              0x08 // disable temperature sensor
#       define BIT_PWR_MGMT_1_SLEEP                 0x40 // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET          0x80 // reset entire device
#define ICM20649_PWR_MGMT_2             ICM20649_REG(REG_BANK0,0x07U)
#define ICM20649_INT_PIN_CFG            ICM20649_REG(REG_BANK0,0x0FU)
#       define BIT_BYPASS_EN                        0x02
#       define BIT_INT_RD_CLEAR                     0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                     0x20    // latch data ready pin
#define ICM20649_INT_ENABLE             ICM20649_REG(REG_BANK0,0x10U)
#       define BIT_PLL_RDY_EN                       0x04
#define ICM20649_INT_ENABLE_1           ICM20649_REG(REG_BANK0,0x11U)
#define ICM20649_INT_ENABLE_2           ICM20649_REG(REG_BANK0,0x12U)
#define ICM20649_INT_ENABLE_3           ICM20649_REG(REG_BANK0,0x13U)
#define ICM20649_I2C_MST_STATUS         ICM20649_REG(REG_BANK0,0x17U)
#define ICM20649_INT_STATUS             ICM20649_REG(REG_BANK0,0x19U)

#define ICM20649_INT_STATUS_1           ICM20649_REG(REG_BANK0,0x1AU)
#define ICM20649_INT_STATUS_2           ICM20649_REG(REG_BANK0,0x1BU)
#define ICM20649_INT_STATUS_3           ICM20649_REG(REG_BANK0,0x1CU)
#define ICM20649_DELAY_TIMEH            ICM20649_REG(REG_BANK0,0x28U)
#define ICM20649_DELAY_TIMEL            ICM20649_REG(REG_BANK0,0x29U)
#define ICM20649_ACCEL_XOUT_H           ICM20649_REG(REG_BANK0,0x2DU)
#define ICM20649_ACCEL_XOUT_L           ICM20649_REG(REG_BANK0,0x2EU)
#define ICM20649_ACCEL_YOUT_H           ICM20649_REG(REG_BANK0,0x2FU)
#define ICM20649_ACCEL_YOUT_L           ICM20649_REG(REG_BANK0,0x30U)
#define ICM20649_ACCEL_ZOUT_H           ICM20649_REG(REG_BANK0,0x31U)
#define ICM20649_ACCEL_ZOUT_L           ICM20649_REG(REG_BANK0,0x32U)
#define ICM20649_GYRO_XOUT_H            ICM20649_REG(REG_BANK0,0x33U)
#define ICM20649_GYRO_XOUT_L            ICM20649_REG(REG_BANK0,0x34U)
#define ICM20649_GYRO_YOUT_H            ICM20649_REG(REG_BANK0,0x35U)
#define ICM20649_GYRO_YOUT_L            ICM20649_REG(REG_BANK0,0x36U)
#define ICM20649_GYRO_ZOUT_H            ICM20649_REG(REG_BANK0,0x37U)
#define ICM20649_GYRO_ZOUT_L            ICM20649_REG(REG_BANK0,0x38U)
#define ICM20649_TEMP_OUT_H             ICM20649_REG(REG_BANK0,0x39U)
#define ICM20649_TEMP_OUT_L             ICM20649_REG(REG_BANK0,0x3AU)
#define ICM20649_EXT_SLV_SENS_DATA_00   ICM20649_REG(REG_BANK0,0x3BU)
#define ICM20649_EXT_SLV_SENS_DATA_01   ICM20649_REG(REG_BANK0,0x3CU)
#define ICM20649_EXT_SLV_SENS_DATA_02   ICM20649_REG(REG_BANK0,0x3DU)
#define ICM20649_EXT_SLV_SENS_DATA_03   ICM20649_REG(REG_BANK0,0x3EU)
#define ICM20649_EXT_SLV_SENS_DATA_04   ICM20649_REG(REG_BANK0,0x3FU)
#define ICM20649_EXT_SLV_SENS_DATA_05   ICM20649_REG(REG_BANK0,0x40U)
#define ICM20649_EXT_SLV_SENS_DATA_06   ICM20649_REG(REG_BANK0,0x41U)
#define ICM20649_EXT_SLV_SENS_DATA_07   ICM20649_REG(REG_BANK0,0x42U)
#define ICM20649_EXT_SLV_SENS_DATA_08   ICM20649_REG(REG_BANK0,0x43U)
#define ICM20649_EXT_SLV_SENS_DATA_09   ICM20649_REG(REG_BANK0,0x44U)
#define ICM20649_EXT_SLV_SENS_DATA_10   ICM20649_REG(REG_BANK0,0x45U)
#define ICM20649_EXT_SLV_SENS_DATA_11   ICM20649_REG(REG_BANK0,0x46U)
#define ICM20649_EXT_SLV_SENS_DATA_12   ICM20649_REG(REG_BANK0,0x47U)
#define ICM20649_EXT_SLV_SENS_DATA_13   ICM20649_REG(REG_BANK0,0x48U)
#define ICM20649_EXT_SLV_SENS_DATA_14   ICM20649_REG(REG_BANK0,0x49U)
#define ICM20649_EXT_SLV_SENS_DATA_15   ICM20649_REG(REG_BANK0,0x4AU)
#define ICM20649_EXT_SLV_SENS_DATA_16   ICM20649_REG(REG_BANK0,0x4BU)
#define ICM20649_EXT_SLV_SENS_DATA_17   ICM20649_REG(REG_BANK0,0x4CU)
#define ICM20649_EXT_SLV_SENS_DATA_18   ICM20649_REG(REG_BANK0,0x4DU)
#define ICM20649_EXT_SLV_SENS_DATA_19   ICM20649_REG(REG_BANK0,0x4EU)
#define ICM20649_EXT_SLV_SENS_DATA_20   ICM20649_REG(REG_BANK0,0x4FU)
#define ICM20649_EXT_SLV_SENS_DATA_21   ICM20649_REG(REG_BANK0,0x50U)
#define ICM20649_EXT_SLV_SENS_DATA_22   ICM20649_REG(REG_BANK0,0x51U)
#define ICM20649_EXT_SLV_SENS_DATA_23   ICM20649_REG(REG_BANK0,0x52U)
#define ICM20649_FIFO_EN_1              ICM20649_REG(REG_BANK0,0x66U)
#       define BIT_SLV3_FIFO_EN                     0x08
#       define BIT_SLV2_FIFO_EN                     0x04
#       define BIT_SLV1_FIFO_EN                     0x02
#       define BIT_SLV0_FIFI_EN0                    0x01
#define ICM20649_FIFO_EN_2              ICM20649_REG(REG_BANK0,0x67U)
#       define BIT_ACCEL_FIFO_EN                    0x10
#       define BIT_ZG_FIFO_EN                       0x08
#       define BIT_YG_FIFO_EN                       0x04
#       define BIT_XG_FIFO_EN                       0x02
#       define BIT_TEMP_FIFO_EN                     0x01
#define ICM20649_FIFO_RST               ICM20649_REG(REG_BANK0,0x68U)
#define ICM20649_FIFO_MODE              ICM20649_REG(REG_BANK0,0x69U)
#define ICM20649_FIFO_COUNTH            ICM20649_REG(REG_BANK0,0x70U)
#define ICM20649_FIFO_COUNTL            ICM20649_REG(REG_BANK0,0x71U)
#define ICM20649_FIFO_R_W               ICM20649_REG(REG_BANK0,0x72U)
#define ICM20649_DATA_RDY_STATUS        ICM20649_REG(REG_BANK0,0x74U)
#define ICM20649_FIFO_CFG               ICM20649_REG(REG_BANK0,0x76U)

#define ICM20649_SELF_TEST_X_GYRO       ICM20649_REG(REG_BANK1,0x02U)
#define ICM20649_SELF_TEST_Y_GYRO       ICM20649_REG(REG_BANK1,0x03U)
#define ICM20649_SELF_TEST_Z_GYRO       ICM20649_REG(REG_BANK1,0x04U)
#define ICM20649_SELF_TEST_X_ACCEL      ICM20649_REG(REG_BANK1,0x0EU)
#define ICM20649_SELF_TEST_Y_ACCEL      ICM20649_REG(REG_BANK1,0x0FU)
#define ICM20649_SELF_TEST_Z_ACCEL      ICM20649_REG(REG_BANK1,0x10U)
#define ICM20649_XA_OFFS_H              ICM20649_REG(REG_BANK1,0x14U)
#define ICM20649_XA_OFFS_L              ICM20649_REG(REG_BANK1,0x15U)
#define ICM20649_YA_OFFS_H              ICM20649_REG(REG_BANK1,0x17U)
#define ICM20649_YA_OFFS_L              ICM20649_REG(REG_BANK1,0x18U)
#define ICM20649_ZA_OFFS_H              ICM20649_REG(REG_BANK1,0x1AU)
#define ICM20649_ZA_OFFS_L              ICM20649_REG(REG_BANK1,0x1BU)
#define ICM20649_TIMEBASE_CORRECTIO     ICM20649_REG(REG_BANK1,0x28U)

#define ICM20649_GYRO_SMPLRT_DIV        ICM20649_REG(REG_BANK2,0x00U)
#define ICM20649_GYRO_CONFIG_1          ICM20649_REG(REG_BANK2,0x01U)
#       define BIT_GYRO_NODLPF_9KHZ                 0x00
#       define BIT_GYRO_DLPF_ENABLE                 0x01
#       define GYRO_DLPF_CFG_229HZ                  0x00
#       define GYRO_DLPF_CFG_188HZ                  0x01
#       define GYRO_DLPF_CFG_154HZ                  0x02
#       define GYRO_DLPF_CFG_73HZ                   0x03
#       define GYRO_DLPF_CFG_35HZ                   0x04
#       define GYRO_DLPF_CFG_17HZ                   0x05
#       define GYRO_DLPF_CFG_9HZ                    0x06
#       define GYRO_DLPF_CFG_376HZ                  0x07
#       define GYRO_DLPF_CFG_SHIFT                  0x03
#       define BITS_GYRO_FS_250DPS                  0x00
#       define BITS_GYRO_FS_500DPS                  0x02
#       define BITS_GYRO_FS_1000DPS                 0x04
#       define BITS_GYRO_FS_2000DPS                 0x06
#       define BITS_GYRO_FS_2000DPS_20649           0x04
#       define BITS_GYRO_FS_MASK                    0x06 // only bits 1 and 2 are used for gyro full scale so use this to mask off other bits
#define ICM20649_GYRO_CONFIG_2          ICM20649_REG(REG_BANK2,0x02U)
#define ICM20649_XG_OFFS_USRH           ICM20649_REG(REG_BANK2,0x03U)
#define ICM20649_XG_OFFS_USRL           ICM20649_REG(REG_BANK2,0x04U)
#define ICM20649_YG_OFFS_USRH           ICM20649_REG(REG_BANK2,0x05U)
#define ICM20649_YG_OFFS_USRL           ICM20649_REG(REG_BANK2,0x06U)
#define ICM20649_ZG_OFFS_USRH           ICM20649_REG(REG_BANK2,0x07U)
#define ICM20649_ZG_OFFS_USRL           ICM20649_REG(REG_BANK2,0x08U)
#define ICM20649_ODR_ALIGN_EN           ICM20649_REG(REG_BANK2,0x09U)
#define ICM20649_ACCEL_SMPLRT_DIV_1     ICM20649_REG(REG_BANK2,0x10U)
#define ICM20649_ACCEL_SMPLRT_DIV_2     ICM20649_REG(REG_BANK2,0x11U)
#define ICM20649_ACCEL_INTEL_CTRL       ICM20649_REG(REG_BANK2,0x12U)
#define ICM20649_ACCEL_WOM_THR          ICM20649_REG(REG_BANK2,0x13U)
#define ICM20649_ACCEL_CONFIG           ICM20649_REG(REG_BANK2,0x14U)
#       define BIT_ACCEL_NODLPF_4_5KHZ               0x00
#       define BIT_ACCEL_DLPF_ENABLE                 0x01
#       define ACCEL_DLPF_CFG_265HZ                  0x00
#       define ACCEL_DLPF_CFG_136HZ                  0x02
#       define ACCEL_DLPF_CFG_68HZ                   0x03
#       define ACCEL_DLPF_CFG_34HZ                   0x04
#       define ACCEL_DLPF_CFG_17HZ                   0x05
#       define ACCEL_DLPF_CFG_8HZ                    0x06
#       define ACCEL_DLPF_CFG_499HZ                  0x07
#       define ACCEL_DLPF_CFG_SHIFT                  0x03
#       define BITS_ACCEL_FS_2G                      0x00
#       define BITS_ACCEL_FS_4G                      0x02
#       define BITS_ACCEL_FS_8G                      0x04
#       define BITS_ACCEL_FS_16G                     0x06
#       define BITS_ACCEL_FS_30G_20649               0x06
#       define BITS_ACCEL_FS_MASK                    0x06 // only bits 1 and 2 are used for gyro full scale so use this to mask off other bits
#define ICM20649_FSYNC_CONFIG           ICM20649_REG(REG_BANK2,0x52U)
#       define FSYNC_CONFIG_EXT_SYNC_TEMP          0x01
#       define FSYNC_CONFIG_EXT_SYNC_GX            0x02
#       define FSYNC_CONFIG_EXT_SYNC_GY            0x03
#       define FSYNC_CONFIG_EXT_SYNC_GZ            0x04
#       define FSYNC_CONFIG_EXT_SYNC_AX            0x05
#       define FSYNC_CONFIG_EXT_SYNC_AY            0x06
#       define FSYNC_CONFIG_EXT_SYNC_AZ            0x07
#define ICM20649_TEMP_CONFIG            ICM20649_REG(REG_BANK2,0x53U)
#define ICM20649_MOD_CTRL_USR           ICM20649_REG(REG_BANK2,0x54U)

#define ICM20649_I2C_MST_ODR_CONFIG     ICM20649_REG(REG_BANK3,0x00U)
#define ICM20649_I2C_MST_CTRL           ICM20649_REG(REG_BANK3,0x01U)
#       define BIT_I2C_MST_P_NSR                    0x10
#       define BIT_I2C_MST_CLK_400KHZ               0x0D
#define ICM20649_I2C_MST_DELAY_CTRL     ICM20649_REG(REG_BANK3,0x02U)
#       define BIT_I2C_SLV0_DLY_EN              0x01
#       define BIT_I2C_SLV1_DLY_EN              0x02
#       define BIT_I2C_SLV2_DLY_EN              0x04
#       define BIT_I2C_SLV3_DLY_EN              0x08
#define ICM20649_I2C_SLV0_ADDR          ICM20649_REG(REG_BANK3,0x03U)
#define ICM20649_I2C_SLV0_REG           ICM20649_REG(REG_BANK3,0x04U)
#define ICM20649_I2C_SLV0_CTRL          ICM20649_REG(REG_BANK3,0x05U)
#define ICM20649_I2C_SLV0_DO            ICM20649_REG(REG_BANK3,0x06U)
#define ICM20649_I2C_SLV1_ADDR          ICM20649_REG(REG_BANK3,0x07U)
#define ICM20649_I2C_SLV1_REG           ICM20649_REG(REG_BANK3,0x08U)
#define ICM20649_I2C_SLV1_CTRL          ICM20649_REG(REG_BANK3,0x09U)
#define ICM20649_I2C_SLV1_DO            ICM20649_REG(REG_BANK3,0x0AU)
#define ICM20649_I2C_SLV2_ADDR          ICM20649_REG(REG_BANK3,0x0BU)
#define ICM20649_I2C_SLV2_REG           ICM20649_REG(REG_BANK3,0x0CU)
#define ICM20649_I2C_SLV2_CTRL          ICM20649_REG(REG_BANK3,0x0DU)
#define ICM20649_I2C_SLV2_DO            ICM20649_REG(REG_BANK3,0x0EU)
#define ICM20649_I2C_SLV3_ADDR          ICM20649_REG(REG_BANK3,0x0FU)
#define ICM20649_I2C_SLV3_REG           ICM20649_REG(REG_BANK3,0x10U)
#define ICM20649_I2C_SLV3_CTRL          ICM20649_REG(REG_BANK3,0x11U)
#define ICM20649_I2C_SLV3_DO            ICM20649_REG(REG_BANK3,0x12U)
#define ICM20649_I2C_SLV4_ADDR          ICM20649_REG(REG_BANK3,0x13U)
#define ICM20649_I2C_SLV4_REG           ICM20649_REG(REG_BANK3,0x14U)
#define ICM20649_I2C_SLV4_CTRL          ICM20649_REG(REG_BANK3,0x15U)
#define ICM20649_I2C_SLV4_DO            ICM20649_REG(REG_BANK3,0x16U)
#define ICM20649_I2C_SLV4_DI            ICM20649_REG(REG_BANK3,0x17U)

#define ICM20649_BANK_SEL               0x7F

// WHOAMI values
#define ICM20649_WHOAMI_ICM20648		0xe0
#define ICM20649_WHOAMI_ICM20948		0xea
#define ICM20649_WHOAMI_ICM20649        0xe1


#endif /* ICM20649_ICM20649_REG_H_ */
