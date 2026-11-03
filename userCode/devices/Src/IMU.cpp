//
// IMU.cpp —— BMI088 SPI 驱动 + 航向角估计（GCC / Keil 通用，寄存器序列参考官方 C 板例程 13.spi_bmi088）
//
#include "IMU.h"
#include "spi.h"
#include <math.h>

/* ============================== 可调参数 ============================== */

// 陀螺 Z 轴方向修正：默认逆时针(向左转)为正；如果实测方向相反，改成 -1.0f
#define IMU_YAW_SIGN        (1.0f)

// 零偏校准采样次数（1ms 一次，1000 次约 1 秒；校准时车必须完全静止）
#define IMU_CALI_SAMPLES    1000

// 陀螺零偏校准后的静止死区（rad/s），小于该值视为 0，减少漂移
#define IMU_GYRO_DEADBAND   (0.004f)

// SPI1 时钟：CubeMX 默认 256 分频太慢；64 分频 = 84MHz/64 ≈ 1.3MHz（BMI088 SPI 上限 10MHz）
#define IMU_SPI_PRESCALER   SPI_BAUDRATEPRESCALER_64

/* ============================ BMI088 寄存器 =========================== */

#define BMI088_ACC_CHIP_ID              0x00u
#define BMI088_ACC_CHIP_ID_VALUE        0x1Eu
#define BMI088_ACC_SOFTRESET            0x7Eu
#define BMI088_ACC_PWR_CTRL             0x7Du
#define BMI088_ACC_ENABLE_ACC_ON        0x04u
#define BMI088_ACC_PWR_CONF             0x7Cu
#define BMI088_ACC_PWR_ACTIVE_MODE      0x00u
#define BMI088_ACC_CONF                 0x40u
#define BMI088_ACC_CONF_800HZ           (0x80u | 0x20u | 0x0Bu)  // MUST_Set | NORMAL | 800Hz
#define BMI088_ACC_RANGE                0x41u
#define BMI088_ACC_RANGE_3G             0x00u
#define BMI088_INT1_IO_CTRL             0x53u
#define BMI088_ACC_INT1_IO_ENABLE       0x08u
#define BMI088_INT_MAP_DATA             0x58u
#define BMI088_ACC_INT1_DRDY            0x04u
#define BMI088_ACCEL_XOUT_L             0x12u
#define BMI088_ACCEL_3G_SEN             0.0008974358974f

#define BMI088_GYRO_CHIP_ID             0x00u
#define BMI088_GYRO_CHIP_ID_VALUE       0x0Fu
#define BMI088_GYRO_SOFTRESET           0x14u
#define BMI088_GYRO_RANGE               0x0Fu
#define BMI088_GYRO_2000                0x00u
#define BMI088_GYRO_BANDWIDTH           0x10u
#define BMI088_GYRO_1000_116_HZ         (0x80u | 0x02u)
#define BMI088_GYRO_LPM1                0x11u
#define BMI088_GYRO_NORMAL_MODE         0x00u
#define BMI088_GYRO_CTRL                0x15u
#define BMI088_DRDY_ON                  0x80u
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16u
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18u
#define BMI088_GYRO_DRDY_IO_INT3        0x01u
#define BMI088_2000DPS_SEN              0.00106526443603169529841533860381f  // rad/s per LSB
#define BMI088_COM_WAIT_US              150u
#define BMI088_SOFTRESET_DELAY_MS       80u

/* ============================== 静态成员 ============================== */

float IMU::yaw_ = 0.0f;
float IMU::gyroBias_[3] = {0.0f, 0.0f, 0.0f};
float IMU::gyro_[3] = {0.0f, 0.0f, 0.0f};
uint8_t IMU::errorCode = 0xFFu;
uint8_t IMU::debugAccelId = 0x00u;
uint8_t IMU::debugGyroId = 0x00u;

/* ============================== SPI 底层 ============================== */

static inline void accelCs(uint8_t level)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void gyroCs(uint8_t level)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t spiTxRx(uint8_t tx)
{
    uint8_t rx = 0u;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 100u);
    return rx;
}

static void writeAccelReg(uint8_t reg, uint8_t data)
{
    accelCs(0);
    spiTxRx(reg);
    spiTxRx(data);
    accelCs(1);
}

static void writeGyroReg(uint8_t reg, uint8_t data)
{
    gyroCs(0);
    spiTxRx(reg);
    spiTxRx(data);
    gyroCs(1);
}

// 加速度计 SPI 读：发地址后要先读一个 dummy 字节，再读数据
static uint8_t readAccelReg(uint8_t reg)
{
    uint8_t data;
    accelCs(0);
    spiTxRx(reg | 0x80u);
    spiTxRx(0xFFu);
    data = spiTxRx(0xFFu);
    accelCs(1);
    return data;
}

// 陀螺仪 SPI 读：发地址后直接读数据
static uint8_t readGyroReg(uint8_t reg)
{
    uint8_t data;
    gyroCs(0);
    spiTxRx(reg | 0x80u);
    data = spiTxRx(0xFFu);
    gyroCs(1);
    return data;
}

static void readAccelMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    accelCs(0);
    spiTxRx(reg | 0x80u);
    spiTxRx(0xFFu);
    for (uint8_t i = 0; i < len; ++i) {
        buf[i] = spiTxRx(0xFFu);
    }
    accelCs(1);
}

static void readGyroMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    gyroCs(0);
    spiTxRx(reg | 0x80u);
    for (uint8_t i = 0; i < len; ++i) {
        buf[i] = spiTxRx(0xFFu);
    }
    gyroCs(1);
}

/* ============================ 初始化流程 ============================== */

uint8_t IMU::initAccel_()
{
    // 上电后前两次读芯片 ID 只用于把加速度计从 I2C 切到 SPI，值不可信（官方例程同样丢弃不判）
    (void)readAccelReg(BMI088_ACC_CHIP_ID);
    HAL_Delay(2);
    (void)readAccelReg(BMI088_ACC_CHIP_ID);
    HAL_Delay(2);

    // 软件复位
    writeAccelReg(BMI088_ACC_SOFTRESET, 0xB6u);
    HAL_Delay(BMI088_SOFTRESET_DELAY_MS);

    // 复位后再读 2 次，以最后一次为准
    for (uint8_t i = 0u; i < 2u; ++i) {
        debugAccelId = readAccelReg(BMI088_ACC_CHIP_ID);
        HAL_Delay(2);
    }
    if (debugAccelId != BMI088_ACC_CHIP_ID_VALUE) {
        return 1u;
    }

    writeAccelReg(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON);
    HAL_Delay(1);
    writeAccelReg(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE);
    HAL_Delay(1);
    writeAccelReg(BMI088_ACC_CONF, BMI088_ACC_CONF_800HZ);
    HAL_Delay(1);
    writeAccelReg(BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G);
    HAL_Delay(1);
    writeAccelReg(BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE);
    HAL_Delay(1);
    writeAccelReg(BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY);
    HAL_Delay(1);

    if (readAccelReg(BMI088_ACC_PWR_CTRL) != BMI088_ACC_ENABLE_ACC_ON) {
        return 3u;
    }
    if (readAccelReg(BMI088_ACC_CONF) != BMI088_ACC_CONF_800HZ) {
        return 4u;
    }
    return 0u;
}

uint8_t IMU::initGyro_()
{
    // 同加速度计：复位前先读两次把接口切到 SPI，不判结果
    (void)readGyroReg(BMI088_GYRO_CHIP_ID);
    HAL_Delay(2);
    (void)readGyroReg(BMI088_GYRO_CHIP_ID);
    HAL_Delay(2);

    writeGyroReg(BMI088_GYRO_SOFTRESET, 0xB6u);
    HAL_Delay(BMI088_SOFTRESET_DELAY_MS);

    for (uint8_t i = 0u; i < 2u; ++i) {
        debugGyroId = readGyroReg(BMI088_GYRO_CHIP_ID);
        HAL_Delay(2);
    }
    if (debugGyroId != BMI088_GYRO_CHIP_ID_VALUE) {
        return 11u;
    }

    writeGyroReg(BMI088_GYRO_RANGE, BMI088_GYRO_2000);
    HAL_Delay(1);
    writeGyroReg(BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ);
    HAL_Delay(1);
    writeGyroReg(BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE);
    HAL_Delay(1);
    writeGyroReg(BMI088_GYRO_CTRL, BMI088_DRDY_ON);
    HAL_Delay(1);
    writeGyroReg(BMI088_GYRO_INT3_INT4_IO_CONF, 0x00u);
    HAL_Delay(1);
    writeGyroReg(BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3);
    HAL_Delay(1);

    if (readGyroReg(BMI088_GYRO_RANGE) != BMI088_GYRO_2000) {
        return 13u;
    }
    return 0u;
}

void IMU::readGyro_(float gyro[3])
{
    uint8_t buf[8] = {0};
    int16_t raw;
    readGyroMulti(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] != BMI088_GYRO_CHIP_ID_VALUE) {
        gyro[0] = gyro[1] = gyro[2] = 0.0f;
        return;
    }
    raw = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    gyro[0] = (float)raw * BMI088_2000DPS_SEN;
    raw = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    gyro[1] = (float)raw * BMI088_2000DPS_SEN;
    raw = (int16_t)(((uint16_t)buf[7] << 8) | buf[6]);
    gyro[2] = (float)raw * BMI088_2000DPS_SEN;
}

void IMU::Init()
{
    // 1. 提高 SPI1 速率（CubeMX 默认 256 分频太慢，1ms 内读不完）
    hspi1.Init.BaudRatePrescaler = IMU_SPI_PRESCALER;
    (void)HAL_SPI_Init(&hspi1);

    // 2. 初始化加速度计与陀螺仪：整轮失败就重试（官方例程是 while(BMI088_init())，
    //    上电第一次 SPI 访问经常读不到芯片 ID，多试两次通常就能过）
    uint8_t err = 0xFFu;
    for (uint8_t attempt = 0u; attempt < 3u; ++attempt) {
        err = initAccel_();
        if (err == 0u) {
            err = initGyro_();
        }
        if (err == 0u) {
            break;
        }
        HAL_Delay(10);
    }
    errorCode = err;
    if (err != 0u) {
        return;
    }

    // 3. 陀螺零偏校准（车必须静止）
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float g[3];
    for (int i = 0; i < IMU_CALI_SAMPLES; ++i) {
        readGyro_(g);
        sum[0] += g[0];
        sum[1] += g[1];
        sum[2] += g[2];
        HAL_Delay(1);
    }
    gyroBias_[0] = sum[0] / (float)IMU_CALI_SAMPLES;
    gyroBias_[1] = sum[1] / (float)IMU_CALI_SAMPLES;
    gyroBias_[2] = sum[2] / (float)IMU_CALI_SAMPLES;

    yaw_ = 0.0f;
    gyro_[0] = gyro_[1] = gyro_[2] = 0.0f;
}

void IMU::Handle()
{
    if (errorCode != 0u) {
        return;
    }
    float g[3];
    readGyro_(g);
    g[0] -= gyroBias_[0];
    g[1] -= gyroBias_[1];
    g[2] -= gyroBias_[2];

    if (fabsf(g[2]) < IMU_GYRO_DEADBAND) {
        g[2] = 0.0f;
    }

    gyro_[0] = g[0];
    gyro_[1] = g[1];
    gyro_[2] = g[2];

    // TIM10 每 1ms 调用一次，积分得到 yaw
    yaw_ += IMU_YAW_SIGN * g[2] * 0.001f;
    if (yaw_ > 3.1415926f) {
        yaw_ -= 6.2831853f;
    } else if (yaw_ < -3.1415926f) {
        yaw_ += 6.2831853f;
    }
}

float IMU::getYaw()
{
    return yaw_;
}

float IMU::getGyroZ()
{
    return gyro_[2];
}

void IMU::resetYaw(float yaw)
{
    yaw_ = yaw;
}
