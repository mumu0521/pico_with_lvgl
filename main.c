/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Example of writing via DMA to the SPI interface and similarly reading it back
// via a loopback.

#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <hardware/gpio.h>
#include <src/display/lv_display.h>
#include <stdio.h>
#include <stdlib.h>
#include "lvgl.h"
#include "lv_st7789.h"
#include "lv_example_get_started.h"
#include "lv_demo_benchmark.h"

#define LCD_H_RES 135
#define LCD_V_RES 240

#define PLL_SYS_KHZ (150 * 1000)
#define LCD_SPI_PORT (spi1)
#define LCD_DC_PIN (8)
#define LCD_CS_PIN (9)
#define LCD_CLK_PIN (10)
#define LCD_MOSI_PIN (11)
#define LCD_RST_PIN (12)
#define LCD_BL_PIN (13)

static uint dma_tx;
static dma_channel_config c;

static lv_display_t *disp;
static struct repeating_timer lvgl_timer;
/* Send short command to the LCD. This function shall wait until the transaction
 * finishes. */
void lcd_send_cmd(lv_display_t *disp,
                  const uint8_t *cmd,
                  size_t cmd_size,
                  const uint8_t *param,
                  size_t param_size)
{
    spi_set_format(LCD_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_put(LCD_DC_PIN, 0);
    gpio_put(LCD_CS_PIN, 0);
    //* write cmd
    spi_write_blocking(LCD_SPI_PORT, (uint8_t *) cmd, cmd_size);

    //* write data
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(LCD_SPI_PORT, (uint8_t *) param, param_size);
    gpio_put(LCD_CS_PIN, 1);
}

/* Send large array of pixel data to the LCD. If necessary, this function has to
 * do the byte-swapping. This function can do the transfer in the background. */
void lcd_send_color(lv_display_t *disp,
                    const uint8_t *cmd,
                    size_t cmd_size,
                    uint8_t *param,
                    size_t param_size)
{
    spi_set_format(LCD_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_put(LCD_DC_PIN, 0);
    gpio_put(LCD_CS_PIN, 0);

    spi_write_blocking(LCD_SPI_PORT, cmd, cmd_size);
    gpio_put(LCD_DC_PIN, 1);
    spi_set_format(LCD_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    dma_channel_configure(dma_tx, &c, &spi_get_hw(LCD_SPI_PORT)->dr,
                          (void *) param,  // read address
                          (param_size / 2), true);
}

static void dma_handler(void)
{
    if (dma_channel_get_irq0_status(dma_tx)) {
        dma_channel_acknowledge_irq0(dma_tx);
        gpio_put(LCD_CS_PIN, 1);
        lv_display_flush_ready(disp);
    }
}

static bool repeating_lvgl_timer_callback(struct repeating_timer *t)
{
    lv_tick_inc(5);
    return true;
}

int main()
{
    //* CLOCK Config
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    clock_configure(clk_peri, 0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    PLL_SYS_KHZ * 1000, PLL_SYS_KHZ * 1000);


    stdio_init_all();
    //* io enable

    spi_init(LCD_SPI_PORT, 150 * 1000 * 1000);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);

    gpio_init(LCD_DC_PIN);
    gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    gpio_init(LCD_CS_PIN);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_init(LCD_RST_PIN);

    gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 1);

    // TODO backlight 100%
    gpio_init(LCD_BL_PIN);
    gpio_set_dir(LCD_BL_PIN, GPIO_OUT);
    gpio_put(LCD_BL_PIN, 1);

    // DMA Config
    dma_tx = dma_claim_unused_channel(true);
    c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_dreq(&c, spi_get_dreq(LCD_SPI_PORT, true));
    dma_channel_set_irq0_enabled(dma_tx, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);


    add_repeating_timer_ms(5, repeating_lvgl_timer_callback, NULL, &lvgl_timer);

    lv_init();

    disp = lv_st7789_create(LCD_H_RES, LCD_V_RES, LV_LCD_FLAG_NONE,
                            lcd_send_cmd, lcd_send_color);

    lv_st7789_set_gap(disp, 40, 53);
    lv_st7789_set_invert(disp, true);

    if (disp == NULL) {
        LV_LOG_ERROR("Create failed\n");
    }
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);

    /* Allocate draw buffers on the heap. In this example we use two partial
     * buffers of 1/10th size of the screen */
    lv_color_t *buf1 = NULL;

    uint32_t buf_size =
        LCD_H_RES * LCD_V_RES / 10 *
        lv_color_format_get_size(lv_display_get_color_format(disp));

    buf1 = lv_malloc(buf_size);
    if (buf1 == NULL) {
        LV_LOG_ERROR("display draw buffer malloc failed");
        return 0;
    }

    lv_display_set_buffers(disp, buf1, NULL, buf_size,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    // lv_example_get_started_1();
    lv_demo_benchmark();

    while (true) {
        lv_task_handler();
        sleep_ms(5);
    }
}
