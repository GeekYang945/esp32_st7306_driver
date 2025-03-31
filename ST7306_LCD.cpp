#include "ST7306_LCD.h"

static spi_device_handle_t lcd_spi;
spi_transaction_t spi_trans;

void SPI_INIT() {
    static const spi_bus_config_t lcd_spi_bus_config = {
        .mosi_io_num = LCD_PIN_MOSI,
        .sclk_io_num = LCD_PIN_CLK,
        .max_transfer_sz = 212 * 120, //最大传输半屏字节数
    };
    static const spi_device_interface_config_t lcd_spi_driver_config = {
        .mode = 0,
        .clock_speed_hz = LCD_SPI_CLOCK_MHZ * 1000000,
        .spics_io_num = LCD_PIN_CS,
        .queue_size = 10,
    };
    spi_bus_initialize(LCD_SPI_HOST, &lcd_spi_bus_config, SPI_DMA_CH_AUTO);
    spi_bus_add_device(LCD_SPI_HOST, &lcd_spi_driver_config, &lcd_spi);
}

static void write(uint8_t cmd) {
    memset(&spi_trans, 0, sizeof(spi_trans));
    spi_trans.length = 8;
    spi_trans.tx_buffer = &cmd;
    spi_device_polling_transmit(lcd_spi, &spi_trans);
    spi_device_acquire_bus(lcd_spi, portMAX_DELAY);
    spi_device_release_bus(lcd_spi);
}

void writeCommand(uint8_t cmd) {
    LCD_DC_LOW;
    write(cmd);
}

void writeData(uint8_t data) {
    LCD_DC_HIGH;
    write(data);
}

void writeDataBatch(const uint8_t *data, uint32_t size) {
    LCD_DC_HIGH;
    memset(&spi_trans, 0, sizeof(spi_trans));
    spi_trans.length = size * 8;
    spi_trans.tx_buffer = data;
    spi_device_polling_transmit(lcd_spi, &spi_trans);
    spi_device_acquire_bus(lcd_spi, portMAX_DELAY);
    spi_device_release_bus(lcd_spi);
}

#define Delay vTaskDelay

void LCD_Init() {
    LCD_RST_LOW;
    vTaskDelay(50);
    LCD_RST_HIGH;
    
    SPI_INIT();
    
    writeCommand(0xD6); // NVM Load Control
    writeData(0x17);
    writeData(0x02);
    writeCommand(0xD1); //Booster Enable
    writeData(0x01);
    writeCommand(0xC0); //Gate Voltage Control
    writeData(0x0E); // VGH=15V
    writeData(0x0A); // VGL=-10V
    writeCommand(0xC1); //VSHP Setting
    writeData(0x41); //VSHP1=5V
    writeData(0x41); //VSHP2=5V
    writeData(0x41); //VSHP3=5V
    writeData(0x41); //VSHP4=5V
    writeCommand(0xC2); //VSLP Setting
    writeData(0x32); //VSLP1=1V
    writeData(0x32); //VSLP2=1V
    writeData(0x32); //VSLP3=1V
    writeData(0x32); //VSLP4=1V
    writeCommand(0xC4); //VSHN Setting
    writeData(0x46); //VSHN1=-3.9V
    writeData(0x46); //VSHN2=-3.9V
    writeData(0x46); //VSHN3=-3.9V
    writeData(0x46); //VSHN4=-3.9V
    writeCommand(0xC5); //VSLN Setting
    writeData(0x46); //VSLN1=-0.4V
    writeData(0x46); //VSLN2=-0.4V
    writeData(0x46); //VSLN3=-0.4V
    writeData(0x46); //VSLN4=-0.4V
    writeCommand(0xB2); //Frame Rate Control
    writeData(0x12); //HPM=32Hz ; LPM=1Hz
    writeCommand(0xB3); //Update Period Gate EQ Control in HPM
    writeData(0xE5);
    writeData(0xF6);
    writeData(0x05); //HPM EQ Control
    writeData(0x46);
    writeData(0x77);
    writeData(0x77);
    writeData(0x77);
    writeData(0x77);
    writeData(0x76);
    writeData(0x45);
    writeCommand(0xB4); //Update Period Gate EQ Control in LPM
    writeData(0x05); //LPM EQ Control
    writeData(0x46);
    writeData(0x77);
    writeData(0x77);
    writeData(0x77);
    writeData(0x77);
    writeData(0x76);
    writeData(0x45);
    writeCommand(0xB7); //Source EQ Enable
    writeData(0x13);
    writeCommand(0xB0); //Gate Line Setting
    writeData(0x78); //480 line
    writeCommand(0x11); //Sleep-out
    vTaskDelay(120);
    
    writeCommand(0xD8); //OSC Setting
    writeData(0x80); //51Hz
//    writeData(0x8F); //32Hz
    
    
    writeData(0xE9);
    writeCommand(0xC9); //Source Voltage Select
    writeData(0x00); //VSHP1; VSLP1 ; VSHN1 ; VSLN1
    
    writeCommand(0x36); //Memory Data Access Control
    writeData(0x48);
    
    writeCommand(0x3A); //Data Format Select
    writeData(0x32);
    
    writeCommand(0xB9); //Gamma Mode Setting
    writeData(0x00); // 4GS
    
    writeCommand(0xB8); //Panel Setting
    writeData(0x0A);
    
    writeCommand(0x35); //TE Setting
    writeData(0x00);
    writeCommand(0xD0); //Enable Auto Power down
    writeData(0xFF);
    writeCommand(0x38); //HPM ON
}


QueueHandle_t gpioEventQueue = nullptr;
TaskHandle_t gpioInterruptTaskHandle;

void IRAM_ATTR gpioInterruptHandler(void *arg) {
    auto gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpioEventQueue, &gpio_num, NULL);
}

volatile bool ST7306_LCD::needRefresh;
uint8_t ST7306_LCD::blankByte;
uint16_t ST7306_LCD::lineByteSize;
uint16_t ST7306_LCD::fullByteSize;
ST7306_LCD::row_data_t ST7306_LCD::frameBuffer[ROW_NUMS];


ST7306_LCD::ST7306_LCD() : WIDTH(ST7306_WIDTH), HEIGHT(ST7306_HEIGHT) {
    _width = WIDTH;
    _height = HEIGHT;
    rotation = 0;
}

void ST7306_LCD::begin() {
    lineByteSize = ST7306_WIDTH;
    fullByteSize = sizeof frameBuffer;
    blankByte = 0x00;
    //空字节显示白色
    memset(frameBuffer, blankByte, fullByteSize);
    
    gpio_config_t gpioConfig = {
        .pin_bit_mask = 0,
        .mode=GPIO_MODE_OUTPUT,
        .pull_up_en=GPIO_PULLUP_DISABLE,
        .pull_down_en=GPIO_PULLDOWN_DISABLE,
        .intr_type=GPIO_INTR_DISABLE,
    };
    
    gpio_num_t gpioList[] = {
        LCD_PIN_MOSI,
        LCD_PIN_CLK,
        LCD_PIN_CS,
        LCD_PIN_DC,
        LCD_PIN_RST,
    };
    for (auto &gpioNUm: gpioList) {
        gpioConfig.pin_bit_mask = 1ull << gpioNUm;
        gpio_config(&gpioConfig);
    }
    
    gpio_config(&gpioConfig);
    
    LCD_Init();
    
    clearDisplay();
    
    writeCommand(0x29);   //DISPLAY ON
    
    //配置TE引脚上升沿中断刷新屏幕
    gpioConfig.pin_bit_mask = 1ull << LCD_PIN_TE;
    gpioConfig.mode = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    gpioConfig.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&gpioConfig);
    gpioEventQueue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(refreshSignalHandler, "ExampleTask", 4096, nullptr, 10, &gpioInterruptTaskHandle);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LCD_PIN_TE, gpioInterruptHandler, (void *) LCD_PIN_TE);
    
}

void ST7306_LCD::end() {
    gpio_isr_handler_remove(LCD_PIN_TE);
    vTaskDelay(50);
    refreshReal();
    vTaskDelay(20);
    writeCommand(0x39);
}

//画点函数 绘制一屏buffer耗时22毫秒 刷屏10毫秒
//drawPixel fillScreen cost 22665 us refresh cost 10378 us
void ST7306_LCD::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
    color16.full = color;
    
    uint16_t yIndex = y >> 1;
    uint16_t xIndex = x + 2;
    
    //取要操作的像素指针
    st7306_pixel_t *bufferPtr = &frameBuffer[yIndex].buff[xIndex];
    
    uint8_t maskBit = (color16.ch.blue & 0b10000) >> 2 | (color16.ch.green & 0b100000) >> 4 | (color16.ch.red & 0b10000) >> 4;
    
    //取颜色掩码，奇数行为后三个BIT 偶数行为中间三个BIT 前两个BIT为空
    if (y % 2 == 0) {
        bufferPtr->full |= 0b00011100;
        maskBit <<= 2;
    } else {
        bufferPtr->full |= 0b11100000;
        maskBit <<= 5;
    }

//  BIT置为0才是想要的颜色
    bufferPtr->full &= (~maskBit);
    
}

//填充整屏颜色函数
//fast fillScreen cost 1698 us refresh cost 10378 us
//绘制一屏buffer耗时不到2毫秒 刷屏10毫秒
void ST7306_LCD::fillScreen(uint16_t color) {
    uint32_t t1 = esp_timer_get_time();
    color16.full = color;
    uint8_t maskBit = (color16.ch.blue & 0b10000) >> 2 | (color16.ch.green & 0b100000) >> 4 | (color16.ch.red & 0b10000) >> 4;
    maskBit = (maskBit << 2) | (maskBit << 5);
    for (auto &rowData: frameBuffer) {
//      x起始位置+2 前两个字节为空字节
        for (uint16_t x = 0; x < lineByteSize; x++) {
            rowData.buff[x + 2].full |= 0b11111100;
            rowData.buff[x + 2].full &= (~maskBit);
        }
    }
    uint32_t t2 = esp_timer_get_time();
    refreshReal();
    uint32_t t3 = esp_timer_get_time();
    printf("fast fillScreen cost %lu us refresh cost %lu us\n", t2 - t1, t3 - t2);
    
}

void ST7306_LCD::clearDisplay() {
    memset(frameBuffer, blankByte, fullByteSize);
    refreshReal();
}

void ST7306_LCD::refresh() {
    needRefresh = true;
}

//真正的刷屏函数，刷新整屏buffer
void ST7306_LCD::refreshReal() {
    writeCommand(0x2A);
    writeData(XS);
    writeData(XE);
    writeCommand(0x2B);
    writeData(YS);
    writeData(YE);
    writeCommand(0x2C);
    writeDataBatch((uint8_t *) &frameBuffer, fullByteSize / 2);
    writeDataBatch((uint8_t *) &frameBuffer[ROW_NUMS / 2], fullByteSize / 2);
    needRefresh = false;
}

uint32_t prevTick;
uint32_t curTick;
uint32_t FPS;

//TE引脚中断函数，判断needRefresh为真才调用refreshReal函数
void ST7306_LCD::refreshSignalHandler(void *arg) {
    auto ioNum = (uint32_t) arg;
    int gpioValue;
    while (true) {
        if (xQueueReceive(gpioEventQueue, &ioNum, portMAX_DELAY)) {
            gpioValue = gpio_get_level((gpio_num_t) ioNum);
            if (gpioValue == 1) {
                curTick = esp_timer_get_time();
                FPS = (1000 * 1000) / (curTick - prevTick);
                if (needRefresh) {
//                    printf("get TE SIGNAL! FPS:%ld needRefresh = true\n", FPS);
                    refreshReal();
                } else {
//                    printf("get TE SIGNAL! FPS:%ld needRefresh = false----------\n", FPS);
                }
                prevTick = curTick;
            }
        }
    }
}


ST7306_LCD st7306Lcd;
