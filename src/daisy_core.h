#pragma once
#ifndef DSY_CORE_HW_H
#define DSY_CORE_HW_H /**< & */
#include <stdint.h>
#include <stdlib.h>

#if defined(_MSC_VER)
#define FORCE_INLINE __forceinline /**< & */
#elif defined(__clang__)
#define FORCE_INLINE inline __attribute__((always_inline)) /**< & */
#pragma clang diagnostic ignored "-Wduplicate-decl-specifier"
#elif defined(__GNUC__)
#define FORCE_INLINE inline __attribute__((always_inline)) /**< & */
#else
#error unknown compiler
#endif

/** @addtogroup utility
    @{
*/

/** Macro for area of memory that is configured as cacheless
This should be used primarily for DMA buffers, and the like.
*/
#define DMA_BUFFER_MEM_SECTION __attribute__((section(".sram1_bss")))
/** 
THE DTCM RAM section is also non-cached. However, is not suitable 
for DMA transfers. Performance is on par with internal SRAM w/ 
cache enabled.
*/
#define DTCM_MEM_SECTION __attribute__((section(".dtcmram_bss")))

#define FBIPMAX 0.999985f             /**< close to 1.0f-LSB at 16 bit */
#define FBIPMIN (-FBIPMAX)            /**< - (1 - LSB) */
#define U82F_SCALE 0.0078740f         /**< 1 / 127 */
#define F2U8_SCALE 127.0f             /**< 128 - 1 */
#define S82F_SCALE 0.0078125f         /**< 1 / (2**7) */
#define F2S8_SCALE 127.0f             /**< (2 ** 7) - 1 */
#define S162F_SCALE 3.0517578125e-05f /**< 1 / (2** 15) */
#define F2S16_SCALE 32767.0f          /**< (2 ** 15) - 1 */
#define F2S24_SCALE 8388608.0f        /**< 2 ** 23 */
#define S242F_SCALE 1.192092896e-07f  /**< 1 / (2 ** 23) */
#define S24SIGN 0x800000              /**< 2 ** 23 */
#define S322F_SCALE 4.6566129e-10f    /**< 1 / (2** 31) */
#define F2S32_SCALE 2147483647.f      /**< (2 ** 31) - 1 */

#ifdef __cplusplus

#define USE_CLK_CYCLE_COUNTER
#ifdef USE_CLK_CYCLE_COUNTER


#include "stm32h7xx.h"
#include "core_cm7.h"
#include "stm32h7xx_hal_rcc.h" // For HAL_RCC_GetSysClockFreq

#include <stdint.h>
#include <string.h>
#include <string>
class STM32CycleProfiler {
    public:
        static constexpr int kMaxProfiledSections = 32;
        static STM32CycleProfiler& Instance() {
            static STM32CycleProfiler instance;
            return instance;
        }

        // Delete copy and move constructors/assignment operators
        STM32CycleProfiler(const STM32CycleProfiler&) = delete;
        STM32CycleProfiler(STM32CycleProfiler&&) = delete;
        STM32CycleProfiler& operator=(const STM32CycleProfiler&) = delete;
        STM32CycleProfiler& operator=(STM32CycleProfiler&&) = delete;

    private:
        STM32CycleProfiler() = default;
        ~STM32CycleProfiler() = default;

        float cpu_freq_Mhz_ = 0.f;
        
        struct ProfileChannel {
            std::string name    = "";
            uint32_t    start   = 0;
            uint32_t    end     = 0;
            bool        valid   = false;

            float avg = 0.f;
            uint32_t numMeasurements = 0;
            uint32_t min = UINT32_MAX;
            uint32_t max = 0;            
        };
        ProfileChannel channels_[kMaxProfiledSections] = {};

        int findChannel(std::string name) const {
            for (int i = 0; i < kMaxProfiledSections; i++) {
                if (channels_[i].name == name) {
                    return i;
                }
            }
            return -1;
        }

        int findOrCreateChannel(std::string name) {
            int returnVal = -1;
            returnVal = findChannel(name);
            if (returnVal >= 0) {
                return returnVal;
            }
            // If we made it this far, a profiling channel was not found, let's create a new one
            for (int i = 0; i < kMaxProfiledSections; i++) {
                if (channels_[i].name == "") {
                    channels_[i].name = name;
                    return i;
                }
            }
            // Else if we couldn't find an empty channel, return -1
            return -1;
        }
    
    public:

        void Init() {
            this->cpu_freq_Mhz_ = static_cast<float>(HAL_RCC_GetSysClockFreq()) / 1e6f;
            // this->cpu_freq_Mhz_ = 480.f; // Default to 480 MHz if HAL is not available
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // enable DWT
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // enable cycle counter
            DWT->CYCCNT = 0; // reset counter
            this->Reset();
        }

        void Reset() {
            // Reset all channels - call this if you want to clear accumulated data
            for (int i = 0; i < kMaxProfiledSections; i++) {
                channels_[i].name = "";
                channels_[i].start = 0;
                channels_[i].end = 0;
                channels_[i].valid = false;
                channels_[i].min = UINT32_MAX;
                channels_[i].max = 0;
                channels_[i].avg = 0.f;
                channels_[i].numMeasurements = 0;
            }
        }

        int Begin(std::string name) {
            int channelIndex = findOrCreateChannel(name);
            if (channelIndex < 0 || channelIndex >= kMaxProfiledSections) return -1;
            channels_[channelIndex].valid = false;
            channels_[channelIndex].start = DWT->CYCCNT;
            return channelIndex;
        }

        int End(std::string name) {
            uint32_t end = DWT->CYCCNT;
            
            int channelIndex = findOrCreateChannel(name);
            if (channelIndex >= kMaxProfiledSections) return -1;

            channels_[channelIndex].end = end;
            channels_[channelIndex].valid = true;
            
            // Update min, max, avg
            uint32_t duration = end - channels_[channelIndex].start;
            if (duration < channels_[channelIndex].min) { // min
                channels_[channelIndex].min = duration;
            }
            if (duration > channels_[channelIndex].max) { // max
                channels_[channelIndex].max = duration;
            }

            // Update average using a simple running average formula
            channels_[channelIndex].numMeasurements++;
            channels_[channelIndex].avg += (static_cast<float>(duration) - channels_[channelIndex].avg) / static_cast<float>(channels_[channelIndex].numMeasurements);
            return channelIndex;
        }

        uint32_t GetCycles(std::string name) const {
            int channelID = findChannel(name);
            if (channelID < 0 || channelID >= kMaxProfiledSections) return -1;
            if (!channels_[channelID].valid) return -1;
            // Only return the time difference when it is valid
            return channels_[channelID].end - channels_[channelID].start;
        }
        float getTimeMicroseconds(std::string name) const {
            int channelID = findChannel(name);
            if (channelID < 0 || channelID >= kMaxProfiledSections) return -1.f;
            if (!channels_[channelID].valid) return -1.f;
            // Only return the time difference when it is valid
            uint32_t cycles = channels_[channelID].end - channels_[channelID].start;
            return static_cast<float>(cycles) / this->cpu_freq_Mhz_;
        }

        float getSystemFrequencyMHz() const {
            return this->cpu_freq_Mhz_;
        }

        std::string GetAllChannels() const {
            std::string channelList;
            for (int i = 0; i < kMaxProfiledSections; i++) {
                // I want to print all the fields for all the channels
                channelList += "Channel " + std::to_string(i) + ": " + channels_[i].name + "\n" + 
                               "  Valid: " + (channels_[i].valid ? "Yes" : "No") + "\n" +
                               "  Avg Cycles: " + std::to_string(channels_[i].avg) + "\n" +
                               "  Min Cycles: " + std::to_string(channels_[i].min) + "\n" +
                               "  Max Cycles: " + std::to_string(channels_[i].max) + "\n" + " Start: " +
                               std::to_string(channels_[i].start) + "\n" + " End: " +   
                               std::to_string(channels_[i].end) + "\n\n";
            }
            return channelList;
        }

        std::string GetFullReportMicroseconds() const {
            std::string report;
            report += "STM32 Cycle Profiler Report:\n";
            report += "----------------------------------------\n";
            for (int i = 0; i < kMaxProfiledSections; i++) {
                if (channels_[i].name != "") {
                    report += "Section: ";
                    report += channels_[i].name;
                    report += "\n";
                    report += "  Avg Time (us): " + std::to_string(channels_[i].avg / this->cpu_freq_Mhz_) + "\n";
                    report += "  Min Time (us): " + std::to_string(static_cast<float>(channels_[i].min) / this->cpu_freq_Mhz_) + "\n";
                    report += "  Max Time (us): " + std::to_string(static_cast<float>(channels_[i].max) / this->cpu_freq_Mhz_) + "\n";
                    report += "----------------------------------------\n";
                }
            }
            return report;
        }
        std::string GetFullReportCyclesRaw() const {
            std::string report;
            report += "STM32 Cycle Profiler Report (Cycles):\n";
            report += "----------------------------------------\n";
            for (int i = 0; i < kMaxProfiledSections; i++) {
                if (channels_[i].name != "") {
                    report += "Section: ";
                    report += channels_[i].name;
                    report += "\n";
                    report += "  Avg Cycles: " + std::to_string(channels_[i].avg) + "\n";
                    report += "  Min Cycles: " + std::to_string(channels_[i].min) + "\n";
                    report += "  Max Cycles: " + std::to_string(channels_[i].max) + "\n";
                    report += "----------------------------------------\n";
                }
            }
            return report;
        }
};

#define PROFILE_BEGIN(name) \
    STM32CycleProfiler::Instance().Begin(name)
#define PROFILE_END(name) \
    STM32CycleProfiler::Instance().End(name)

// #else // In the event we want to disable this and not have to search through the codebase for all profiling points
// #define PROFILE_BEGIN(name)
// #define PROFILE_END(name)

#endif //USE_CLK_CYCLE_COUNTER


#endif // __cplusplus

/** shorthand macro for simplifying the reading of the left 
 *  channel of a non-interleaved output buffer named out */
#define OUT_L out[0]

/** shorthand macro for simplifying the reading of the right 
 *  channel of a non-interleaved output buffer named out */
#define OUT_R out[1]

/** shorthand macro for simplifying the reading of the left 
 *  channel of a non-interleaved input buffer named in */
#define IN_L in[0]

/** shorthand macro for simplifying the reading of the right 
 *  channel of a non-interleaved input buffer named in */
#define IN_R in[1]

/** 
    Computes cube.
    \param x Number to be cubed
    \return x ^ 3
*/
FORCE_INLINE float cube(float x)
{
    return (x * x) * x;
}

/** 
    Converts unsigned 8-bit to float
    \param x Number to be scaled.
    \return Scaled number.
*/
FORCE_INLINE float u82f(uint8_t x)
{
    return ((float)x - 127.f) * U82F_SCALE;
}

/**
    Converts float to unsigned 8-bit
*/
FORCE_INLINE uint8_t f2u8(float x)
{
    x = x <= FBIPMIN ? FBIPMIN : x;
    x = x >= FBIPMAX ? FBIPMAX : x;
    return (uint8_t)((x * F2U8_SCALE) + F2U8_SCALE);
}


/** 
    Converts Signed 8-bit to float
    \param x Number to be scaled.
    \return Scaled number.
*/
FORCE_INLINE float s82f(int8_t x)
{
    return (float)x * S82F_SCALE;
}

/**
    Converts float to Signed 8-bit
*/
FORCE_INLINE int8_t f2s8(float x)
{
    x = x <= FBIPMIN ? FBIPMIN : x;
    x = x >= FBIPMAX ? FBIPMAX : x;
    return (int32_t)(x * F2S8_SCALE);
}

/** 
    Converts Signed 16-bit to float
    \param x Number to be scaled.
    \return Scaled number.
*/
FORCE_INLINE float s162f(int16_t x)
{
    return (float)x * S162F_SCALE;
}

/**
    Converts float to Signed 16-bit
*/
FORCE_INLINE int16_t f2s16(float x)
{
    x = x <= FBIPMIN ? FBIPMIN : x;
    x = x >= FBIPMAX ? FBIPMAX : x;
    return (int32_t)(x * F2S16_SCALE);
}

/**
    Converts Signed 24-bit to float
 */
FORCE_INLINE float s242f(int32_t x)
{
    x = (x ^ S24SIGN) - S24SIGN; //sign extend aka ((x<<8)>>8)
    return (float)x * S242F_SCALE;
}
/**
    Converts float to Signed 24-bit
 */
FORCE_INLINE int32_t f2s24(float x)
{
    x = x <= FBIPMIN ? FBIPMIN : x;
    x = x >= FBIPMAX ? FBIPMAX : x;
    return (int32_t)(x * F2S24_SCALE);
}

/**
    Converts Signed 32-bit to float
 */
FORCE_INLINE float s322f(int32_t x)
{
    return (float)x * S322F_SCALE;
}
/**
    Converts float to Signed 24-bit
 */
FORCE_INLINE int32_t f2s32(float x)
{
    x = x <= FBIPMIN ? FBIPMIN : x;
    x = x >= FBIPMAX ? FBIPMAX : x;
    return (int32_t)(x * F2S32_SCALE);
}

#ifdef __cplusplus

namespace daisy
{
/** @brief GPIO Port names */
enum GPIOPort
{
    PORTA, /**< Port A */
    PORTB, /**< Port B */
    PORTC, /**< Port C */
    PORTD, /**< Port D */
    PORTE, /**< Port E */
    PORTF, /**< Port F */
    PORTG, /**< Port G */
    PORTH, /**< Port H */
    PORTI, /**< Port I */
    PORTJ, /**< Port J */
    PORTK, /**< Port K */
    PORTX, /**< Used as a dummy port to signal an invalid pin. */
};

/** @brief representation of hardware port/pin combination */
struct Pin
{
    GPIOPort port;
    uint8_t  pin;

    /** @brief Constructor creates a valid pin. 
     *  @param pt GPIOPort between PA, and PK corresponding to STM32 Port.
     *  @param pn pin number in range of 0-15
    */
    constexpr Pin(const GPIOPort pt, const uint8_t pn) : port(pt), pin(pn) {}

    /** @brief Basic Constructor creates an invalid Pin object */
    constexpr Pin() : port(PORTX), pin(255) {}

    /** @brief checks validity of a Pin 
     *  @retval returns true if the port is a valid hardware pin, otherwise false.
    */
    constexpr bool IsValid() const { return port != PORTX && pin < 16; }

    /** @brief comparison operator for checking equality between Pin objects */
    constexpr bool operator==(const Pin &rhs) const
    {
        return (rhs.port == port) && (rhs.pin == pin);
    }

    /** @brief comparison operator for checking inequality between Pin objects */
    constexpr bool operator!=(const Pin &rhs) const { return !operator==(rhs); }
};


/** Enums and a simple struct for defining a hardware pin on the MCU
 *  These correlate with the stm32 datasheet, and are used to configure
 *  the hardware.
 * 
 *  This along with the dsy_gpio_pin class should no longer be used.
 *  They are available for backwards compatability. 
 * 
 *  Please use GPIOPort enum and the Pin struct instead.
 */
typedef enum
{
    DSY_GPIOA, /**< & */
    DSY_GPIOB, /**< & */
    DSY_GPIOC, /**< & */
    DSY_GPIOD, /**< & */
    DSY_GPIOE, /**< & */
    DSY_GPIOF, /**< & */
    DSY_GPIOG, /**< & */
    DSY_GPIOH, /**< & */
    DSY_GPIOI, /**< & */
    DSY_GPIOJ, /**< & */
    DSY_GPIOK, /**< & */
    DSY_GPIOX, /** This is a non-existant port for unsupported bits of hardware. */
    DSY_GPIO_LAST, /** Final enum member */
} dsy_gpio_port;

/** Hardware define pins 
 *  
 *  The dsy_gpio_pin struct should no longer be used, and is only available for
 *  backwards compatability.
 * 
 *  Please use Pin struct instead.
 */
[[deprecated("Use daisy::Pin instead")]] typedef struct
{
    dsy_gpio_port port; /**< & */
    uint8_t       pin;  /**< number 0-15 */

    constexpr operator Pin() const
    {
        return Pin(static_cast<GPIOPort>(port), pin);
    }

} dsy_gpio_pin;

} // namespace daisy

#endif // __cplusplus

#endif
/** @} */
