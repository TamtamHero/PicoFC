/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_hmc5883l_interface_template.c
 * @brief     driver hmc5883l interface template source file
 * @version   2.0.0
 * @author    Shifeng Li
 * @date      2021-03-20
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/03/20  <td>2.0      <td>Shifeng Li  <td>format the code
 * <tr><td>2020/12/02  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_hmc5883l_interface.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t hmc5883l_interface_iic_init(void)
{
    i2c_init(i2c_default, 400 * 1000);
    // Enable SDA and SCL pins (4, 5 on a Pico)
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    // i2c enabled by mpu6050 first, no need to do it again
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t hmc5883l_interface_iic_deinit(void)
{
    i2c_deinit(i2c_default);
    // i2c disabled by mpu6050 first, no need to do it again
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr is the iic device write address
 * @param[in]  reg is the iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t hmc5883l_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    i2c_write_blocking(i2c_default, addr, &reg, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buf, len, false);
    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr is the iic device write address
 * @param[in] reg is the iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t request[50]; // somehow we have to send it all at once...
uint8_t hmc5883l_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    request[0] = reg;
    memcpy(request+1, buf, len);
    i2c_write_blocking(i2c_default, addr, request, len+1, false);
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void hmc5883l_interface_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void hmc5883l_interface_debug_print(const char *const fmt, ...)
{
    printf(fmt);
}

/*********** User Facing Functions *************/

static hmc5883l_handle_t hmc5883l_handle;        /**< hmc5883l handle */

/**
 * @brief  basic example init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t hmc5883l_basic_init(void)
{
    uint8_t res;

    /* link interface function */
    DRIVER_HMC5883L_LINK_INIT(&hmc5883l_handle, hmc5883l_handle_t);
    DRIVER_HMC5883L_LINK_IIC_INIT(&hmc5883l_handle, hmc5883l_interface_iic_init);
    DRIVER_HMC5883L_LINK_IIC_DEINIT(&hmc5883l_handle, hmc5883l_interface_iic_deinit);
    DRIVER_HMC5883L_LINK_IIC_READ(&hmc5883l_handle, hmc5883l_interface_iic_read);
    DRIVER_HMC5883L_LINK_IIC_WRITE(&hmc5883l_handle, hmc5883l_interface_iic_write);
    DRIVER_HMC5883L_LINK_DELAY_MS(&hmc5883l_handle, hmc5883l_interface_delay_ms);
    DRIVER_HMC5883L_LINK_DEBUG_PRINT(&hmc5883l_handle, hmc5883l_interface_debug_print);

    hmc5883l_interface_debug_print("hmc5883l: init started\n");

    uint8_t mode = 0, status = 0;

    /* hmc5883l init */
    res = hmc5883l_init(&hmc5883l_handle);
    if (res != 0)
    {
        hmc5883l_interface_debug_print("hmc5883l: init failed.\n");

        return 1;
    }

    /* set average sample */
    res = hmc5883l_set_average_sample(&hmc5883l_handle, HMC5883L_BASIC_DEFAULT_AVERAGE_SAMPLE);
    if (res != 0)
    {
        hmc5883l_interface_debug_print("hmc5883l: set average sample failed.\n");
        (void)hmc5883l_deinit(&hmc5883l_handle);

        return 1;
    }

    /* set data output rate */
    res = hmc5883l_set_data_output_rate(&hmc5883l_handle, HMC5883L_BASIC_DEFAULT_DATA_OUTPUT_RATE);
    if (res != 0)
    {
        hmc5883l_interface_debug_print("hmc5883l: set data output rate failed.\n");
        (void)hmc5883l_deinit(&hmc5883l_handle);

        return 1;
    }

    /* set mode */
    res = hmc5883l_set_mode(&hmc5883l_handle, HMC5883L_BASIC_DEFAULT_MODE);
    if (res != 0)
    {
        hmc5883l_interface_debug_print("hmc5883l: set mode failed.\n");
        (void)hmc5883l_deinit(&hmc5883l_handle);

        return 1;
    }

    /* set gain */
    res = hmc5883l_set_gain(&hmc5883l_handle, HMC5883L_BASIC_DEFAULT_GAIN);
    if (res != 0)
    {
        hmc5883l_interface_debug_print("hmc5883l: set gain failed.\n");
        (void)hmc5883l_deinit(&hmc5883l_handle);

        return 1;
    }

    /* start continuous read */
    res = hmc5883l_start_continuous_read(&hmc5883l_handle);
    if (res != 0)
    {
        hmc5883l_interface_debug_print("hmc5883l: start continuous read failed:: %u \n", res);
        (void)hmc5883l_deinit(&hmc5883l_handle);

        return 1;
    }

    res = hmc5883l_handle.iic_read(0x1E, 0x02, (uint8_t *)&mode, 1);
    res = hmc5883l_handle.iic_read(0x1E, 0x09, (uint8_t *)&status, 1);
    hmc5883l_interface_debug_print("hmc5883l: init done. mode:%u status:%u\n", mode, status);

    return 0;
}

/**
 * @brief      basic example read
 * @param[out] *m_gauss points to a converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t hmc5883l_basic_read(float m_gauss[3])
{
    int16_t raw[3];
    /* read x,y,z data */
    return hmc5883l_continuous_read(&hmc5883l_handle, (int16_t *)raw, m_gauss);
}

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t hmc5883l_basic_deinit(void)
{
    uint8_t res;

    /* stop continuous read*/
    res = hmc5883l_stop_continuous_read(&hmc5883l_handle);
    if (res != 0)
    {
        return 1;
    }

    /* close hmc5883l */
    if (hmc5883l_deinit(&hmc5883l_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}