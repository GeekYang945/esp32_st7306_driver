#ifndef ST7306_LCD_H
#define ST7306_LCD_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <esp_timer.h>
#include <memory.h>

#define LCD_SPI_HOST       SPI2_HOST

#define LCD_PIN_MOSI GPIO_NUM_3
#define LCD_PIN_CLK GPIO_NUM_46
#define LCD_PIN_CS GPIO_NUM_9
#define LCD_PIN_DC GPIO_NUM_10
#define LCD_PIN_RST GPIO_NUM_11
#define LCD_PIN_TE GPIO_NUM_12

#define LCD_SPI_CLOCK_MHZ 40

#define LCD_RST_LOW          gpio_set_level(LCD_PIN_RST,0)
#define LCD_RST_HIGH         gpio_set_level(LCD_PIN_RST,1)
#define LCD_DC_LOW           gpio_set_level(LCD_PIN_DC,0)
#define LCD_DC_HIGH          gpio_set_level(LCD_PIN_DC,1)


void SPI_INIT();

void LCD_Init();

void writeCommand(uint8_t cmd);

void writeData(uint8_t data);

void writeDataBatch(const uint8_t *data, uint32_t size);


#define ST7306_WIDTH  210
#define ST7306_HEIGHT  480
#define XS (0x04)
#define XE (0x38)

#define YS (0)
#define YE (ST7306_HEIGHT/2-1)

#define ROW_NUMS (ST7306_HEIGHT/2)


class ST7306_LCD {
public:
    typedef union {
        struct {
            uint16_t blue: 5;
            uint16_t green: 6;
            uint16_t red: 5;
        } ch;
        uint16_t full;
    } st7306_color16_t;
    
    typedef union {
        struct {
            uint8_t dummy0: 1;
            uint8_t dummy1: 1;
            uint8_t r1: 1;
            uint8_t g1: 1;
            uint8_t b1: 1;
            uint8_t r0: 1;
            uint8_t g0: 1;
            uint8_t b0: 1;
        } ch;
        uint8_t full;
    } st7306_pixel_t;
    
    typedef struct {
        st7306_pixel_t buff[ST7306_WIDTH + 2];
    } row_data_t;

public:
    
    
    ST7306_LCD();
    
    void begin();
    
    void end();
    
    void drawPixel(uint16_t x, uint16_t y, uint16_t color);
    
    void fillScreen(uint16_t color);
    
    void clearDisplay();
    
    static void refresh();
    
    static void refreshReal();
    
    static void refreshSignalHandler(void *arg);
    
    uint16_t width() const { return _width; };
    
    uint16_t height() const { return _height; };
    
    static volatile bool needRefresh;
    static uint8_t blankByte;
    static uint16_t lineByteSize;
    static uint16_t fullByteSize;
    static row_data_t frameBuffer[];
    st7306_color16_t color16;
    
    
    uint16_t WIDTH;
    uint16_t HEIGHT;
    uint16_t _width;
    uint16_t _height;
    uint8_t rotation;
};

extern ST7306_LCD st7306Lcd;

#endif //ST7306_LCD_H
