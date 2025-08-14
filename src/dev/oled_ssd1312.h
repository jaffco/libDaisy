#pragma once
#ifndef SA_OLED_SSD1312_H
#define SA_OLED_SSD1312_H /**< & */

#include "dev/oled_ssd130x.h"

namespace daisy
{

/**
 * A driver implementation for the SSD1312 OLED display controller
 * 
 * Based on the u8g2 reference implementation. The SSD1312 is an
 * enhanced version of the SSD1306 with better contrast and improved
 * characteristics. This driver follows the proven u8g2 initialization
 * sequence for reliable operation.
 */
template <size_t width, size_t height, typename Transport>
class SSD1312Driver
{
  public:
    struct Config
    {
        typename Transport::Config transport_config;
    };

    void Init(Config config)
    {
        transport_.Init(config.transport_config);

        // Initialize state
        updateing_ = false;

        // SSD1312 Init routine based on u8g2 reference implementation
        // u8x8_d_ssd1312_128x32_init_seq from u8g2 library
        
        // Display off
        transport_.SendCommand(0xAE);
        
        // Clock divide ratio (0x00=1) and oscillator frequency (0x8)
        transport_.SendCommand(0xD5);
        transport_.SendCommand(0x80);
        
        // Multiplex ratio - height dependent
        transport_.SendCommand(0xA8);
        switch(height)
        {
            case 16: transport_.SendCommand(0x0F); break;  // 16-1
            case 32: transport_.SendCommand(0x3F); break;  // SSD1312 uses 0x3F for 32px!
            case 48: transport_.SendCommand(0x2F); break;  // 48-1
            case 64: transport_.SendCommand(0x3F); break;  // 64-1
            default: transport_.SendCommand(0x3F); break;  // Default to 64
        }
        
        // Display offset - SSD1312 specific
        transport_.SendCommand(0xD3);
        if(height == 32) {
            transport_.SendCommand(0x30);  // Line shift by 3*16 = 48 for 128x32
        } else {
            transport_.SendCommand(0x00);
        }
        
        // Set display start line to 0
        transport_.SendCommand(0x40);
        
        // Charge pump setting - enable
        transport_.SendCommand(0x8D);
        transport_.SendCommand(0x14);
        
        // Memory addressing mode - horizontal
        transport_.SendCommand(0x20);
        transport_.SendCommand(0x00);
        
        // Segment remap a0/a1 (flips L/R)
        transport_.SendCommand(0xA0);
        
        // COM output scan direction c0: normal, c8: reverse
        transport_.SendCommand(0xC8);
        
        // COM pins hardware configuration
        transport_.SendCommand(0xDA);
        // Force 0x12 for SSD1312 - many panels need alternative COM pin config
        // even for 32-pixel height displays (unlike SSD1306)
        transport_.SendCommand(0x12);
        
        // Contrast control register
        transport_.SendCommand(0x81);
        transport_.SendCommand(0x9F);  // SSD1312 optimized contrast
        
        // Pre-charge period
        transport_.SendCommand(0xD9);
        transport_.SendCommand(0x22);
        
        // VCOMH deselect level
        transport_.SendCommand(0xDB);
        transport_.SendCommand(0x34);
        
        // Deactivate scroll
        transport_.SendCommand(0x2E);
        
        // Output RAM to display
        transport_.SendCommand(0xA4);
        
        // Normal display mode (not inverted)
        transport_.SendCommand(0xA6);
        
        // Clear display buffer
        Fill(false);
        
        // Display on
        transport_.SendCommand(0xAF);
    }

    size_t Width() const { return width; }
    size_t Height() const { return height; }

    void DrawPixel(uint_fast8_t x, uint_fast8_t y, bool on)
    {
        if(x >= width || y >= height)
            return;
        if(on)
            buffer_[x + (y / 8) * width] |= (1 << (y % 8));
        else
            buffer_[x + (y / 8) * width] &= ~(1 << (y % 8));
    }

    void Fill(bool on)
    {
        for(size_t i = 0; i < sizeof(buffer_); i++)
        {
            buffer_[i] = on ? 0xff : 0x00;
        }
    }

    /**
     * Update the display using u8g2-style page addressing
     * Based on u8g2's proven SSD1312 implementation
     * For now, only blocking mode - DMA can be added later
     */
    void Update()
    {
        // Always use blocking update for now (keep it simple and working)
        // Set column address range (0 to width-1)
        transport_.SendCommand(0x21);
        transport_.SendCommand(0x00);       // Column start address
        transport_.SendCommand(width - 1);  // Column end address
        
        // Set page address range (0 to height/8-1)
        transport_.SendCommand(0x22);
        transport_.SendCommand(0x00);           // Page start address
        transport_.SendCommand((height / 8) - 1); // Page end address
        
        // Send the buffer data efficiently
        transport_.SendData(buffer_, sizeof(buffer_));
        updateing_ = false;
    }

    /**
     * Has update finished - always true for blocking mode
     */
    bool UpdateFinished() { return true; }

    /**
     * Set display contrast (0-255)
     */
    void SetContrast(uint8_t contrast)
    {
        transport_.SendCommand(0x81);
        transport_.SendCommand(contrast);
    }

    /**
     * Turn display on/off
     */
    void SetDisplayOn(bool on)
    {
        transport_.SendCommand(on ? 0xAF : 0xAE);
    }

    /**
     * Invert display colors
     */
    void SetInvert(bool invert)
    {
        transport_.SendCommand(invert ? 0xA7 : 0xA6);
    }

  protected:
    Transport transport_;
    uint8_t   buffer_[width * height / 8];
    bool      updateing_;
};

/**
 * A driver for the SSD1312 128x64 OLED displays connected via 4 wire SPI  
 */
using SSD13124WireSpi128x64Driver
    = daisy::SSD1312Driver<128, 64, SSD130x4WireSpiTransport>;

/**
 * A driver for the SSD1312 128x32 OLED displays connected via 4 wire SPI  
 */
using SSD13124WireSpi128x32Driver
    = daisy::SSD1312Driver<128, 32, SSD130x4WireSpiTransport>;

/**
 * A driver for the SSD1312 64x48 OLED displays connected via 4 wire SPI  
 */
using SSD13124WireSpi64x48Driver
    = daisy::SSD1312Driver<64, 48, SSD130x4WireSpiTransport>;

/**
 * A driver for the SSD1312 64x32 OLED displays connected via 4 wire SPI  
 */
using SSD13124WireSpi64x32Driver
    = daisy::SSD1312Driver<64, 32, SSD130x4WireSpiTransport>;

/**
 * A driver for the SSD1312 128x64 OLED displays connected via I2C  
 */
using SSD1312I2c128x64Driver
    = daisy::SSD1312Driver<128, 64, SSD130xI2CTransport>;

/**
 * A driver for the SSD1312 128x32 OLED displays connected via I2C  
 */
using SSD1312I2c128x32Driver
    = daisy::SSD1312Driver<128, 32, SSD130xI2CTransport>;

/**
 * A driver for the SSD1312 64x48 OLED displays connected via I2C  
 */
using SSD1312I2c64x48Driver
    = daisy::SSD1312Driver<64, 48, SSD130xI2CTransport>;

/**
 * A driver for the SSD1312 64x32 OLED displays connected via I2C  
 */
using SSD1312I2c64x32Driver
    = daisy::SSD1312Driver<64, 32, SSD130xI2CTransport>;

/**
 * A driver for the SSD1312 128x64 OLED displays connected via 4 wire Soft SPI  
 */
using SSD13124WireSoftSpi128x64Driver
    = daisy::SSD1312Driver<128, 64, SSD130x4WireSoftSpiTransport>;

/**
 * A driver for the SSD1312 128x32 OLED displays connected via 4 wire Soft SPI  
 */
using SSD13124WireSoftSpi128x32Driver
    = daisy::SSD1312Driver<128, 32, SSD130x4WireSoftSpiTransport>;

} // namespace daisy

#endif
