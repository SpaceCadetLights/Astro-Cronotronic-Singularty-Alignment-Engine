#pragma once

/*
    i2s_out_xyz_mks_dlc32.h
    Part of Grbl_ESP32
    Pin assignments for the ESP32 I2S 6-axis board
    2018    - Bart Dring
    2020    - Mitch Bradley
    2020    - Michiyasu Odaki
    2021    - @Makerbase 
    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Grbl_ESP32.  If not, see <http://www.gnu.org/licenses/>.
*/

#define USE_BOARD_V2_0

#define USE_LCD_DMA

#define USR_USER_KB  

#define USR_Z_MOTOR

// #define USE_BEEP

// #define USE_BL_TOUCH

#define MACHINE_NAME                "MKS DLC32"
#ifdef USE_BOARD_V2_0
    #define BOARD_NAME              "Board:MKS DLC32 CNC V2.0"
#else 
    #define BOARD_NAME              "Board:MKS DLC32 CNC V1.0"
#endif

#define FW_NAME                     "V2.1(8M.H35.20220421)"


#ifdef N_AXIS
    #undef N_AXIS
#endif
#define N_AXIS 3

#define HTTP_UPLOAD_BUFLEN 1024*10

#define ENABLE_SOFTWARE_DEBOUNCE

#ifdef ENABLE_SD_CARD
    #undef ENABLE_SD_CARD
#endif

// I2S (steppers & other output-only pins)
#define USE_I2S_OUT
#define USE_I2S_STEPS
#define DEFAULT_STEPPER ST_I2S_STATIC

// I2S pins set
#define I2S_OUT_BCK                 GPIO_NUM_16
#define I2S_OUT_WS                  GPIO_NUM_17
#define I2S_OUT_DATA                GPIO_NUM_21
    
// X I2S pin set    
#define X_DISABLE_PIN               I2SO(0)
#define X_DIRECTION_PIN             I2SO(2)
#define X_STEP_PIN                  I2SO(1)

// Y I2S pin set
#define Y_DISABLE_PIN               I2SO(0)
#define Y_DIRECTION_PIN             I2SO(6)
#define Y_STEP_PIN                  I2SO(5)
    
// Z I2S pin set
#if defined(USR_Z_MOTOR)
#define Z_DISABLE_PIN               I2SO(0)
#define Z_DIRECTION_PIN             I2SO(4)
#define Z_STEP_PIN                  I2SO(3)
#endif

// Laser pin set
#define SPINDLE_TYPE                SpindleType::PWM 
#ifdef USE_BOARD_V2_0
#define SPINDLE_OUTPUT_PIN          GPIO_NUM_32
#else 
#define SPINDLE_OUTPUT_PIN          GPIO_NUM_22
#endif

#define X_LIMIT_PIN                 GPIO_NUM_36
#define Y_LIMIT_PIN                 GPIO_NUM_35

#if defined(USR_Z_MOTOR)
#define Z_LIMIT_PIN                 GPIO_NUM_34
#endif

#ifdef USE_BOARD_V2_0
#define PROBE_PIN                   GPIO_NUM_22    
#else
#define PROBE_PIN                   GPIO_NUM_2    
#endif

#define LCD_SCK				        GPIO_NUM_18
#define LCD_MISO				    GPIO_NUM_19
#define LCD_MOSI				    GPIO_NUM_23
#define LCD_RS					    GPIO_NUM_33
#define LCD_EN					    GPIO_NUM_5     
#define LCD_RST					    GPIO_NUM_27     
#define LCD_CS					    GPIO_NUM_25
#define TOUCH_CS				    GPIO_NUM_26
#define BEEPER					    I2SO(7)

#define IIC_SCL                     GPIO_NUM_4
#define IIC_SDA                     GPIO_NUM_0

#define COOLANT_FLOOD_PIN           IIC_SCL

//sd card spi
#define GRBL_SPI_SCK 			    GPIO_NUM_14
#define GRBL_SPI_MISO 			    GPIO_NUM_12
#define GRBL_SPI_MOSI 			    GPIO_NUM_13
#define GRBL_SPI_SS 			    GPIO_NUM_15
#define SDCARD_DET_PIN 			    GPIO_NUM_39
#define GRBL_SPI_FREQ 			    40000000

// === Default settings
// #define DEFAULT_STEP_PULSE_MICROSECONDS I2S_OUT_USEC_PER_PULSE
#define DEFAULT_STEP_PULSE_MICROSECONDS     10

#define DEFAULT_STEPPER_IDLE_LOCK_TIME      255

#define DEFAULT_STEPPING_INVERT_MASK    	0 // uint8_t
#define DEFAULT_DIRECTION_INVERT_MASK   	0 
#define DEFAULT_INVERT_ST_ENABLE        	0 // boolean
#define DEFAULT_INVERT_LIMIT_PINS       	1 // boolean
#define DEFAULT_INVERT_PROBE_PIN        	1 // boolean

#define DEFAULT_STATUS_REPORT_MASK 			1

#define DEFAULT_JUNCTION_DEVIATION  		0.01        // mm       
#define DEFAULT_ARC_TOLERANCE       		0.002       // mm
#define DEFAULT_REPORT_INCHES       		0           // false

#define DEFAULT_SOFT_LIMIT_ENABLE 			0           // false
#define DEFAULT_HARD_LIMIT_ENABLE 			0           // false

#define DEFAULT_HOMING_CYCLE_0              (bit(X_AXIS) | bit(Y_AXIS))
#define DEFAULT_HOMING_CYCLE_1              0           // (bit(Z_AXIS))        /* If you want star Z axis, select (bit(Z_AXIS)) */
#define DEFAULT_HOMING_ENABLE           	1           // false
#define DEFAULT_HOMING_DIR_MASK         	3           // move positive dir Z,negative X,Y
#define DEFAULT_HOMING_FEED_RATE        	300.0       // mm/min
#define DEFAULT_HOMING_SEEK_RATE        	5000.0      // mm/min
#define DEFAULT_HOMING_DEBOUNCE_DELAY   	250         // msec (0-65k)
#define DEFAULT_HOMING_PULLOFF          	20.0         // mm

#ifdef USE_SPINDLE_RELAY
    #define DEFAULT_SPINDLE_RPM_MAX 1.0 // must be 1 so PWM duty is alway 100% to prevent relay damage
#else
    #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // can be change to your spindle max
#endif
#define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm

#define DEFAULT_LASER_MODE 0 // false

#define DEFAULT_X_STEPS_PER_MM 62.22 //treat 1 mm as 1 degree
#define DEFAULT_Y_STEPS_PER_MM 62.22  //treat 1 mm as 1 degree
#define DEFAULT_Z_STEPS_PER_MM 80.0

#define DEFAULT_X_MAX_RATE 15000.0 // mm/s
#define DEFAULT_Y_MAX_RATE 15000.0 // mm/s
#define DEFAULT_Z_MAX_RATE 6000.0 // mm/s

#define DEFAULT_X_ACCELERATION 1500.0 // mm/sec^2
#define DEFAULT_Y_ACCELERATION 1500.0 // mm/sec^2
#define DEFAULT_Z_ACCELERATION 500.0 // mm/sec^2

#define DEFAULT_X_MAX_TRAVEL 450.0 // mm NOTE: Must be a positive value.
#define DEFAULT_Y_MAX_TRAVEL 450.0 // mm NOTE: Must be a positive value.
#define DEFAULT_Z_MAX_TRAVEL 50.0 // mm NOTE: Must be a positive value.

#define DEFAULT_SPINDLE_FREQ        1920// 8000.0   // 1KHz
#define DEFAULT_LASER_FULL_POWER    1000
#define DEFAULT_SPINDLE_MAX_VALUE   1000    
#define DEFAULT_SPINDLE_MIN_VALUE   0


#define DEFAULT_BEEP_STATUS                 1
#define DEFAULT_LANGUAGE_STATUS             1       // default simple engliash

#define CUSTOM_CODE_FILENAME    "../Custom/clock_engine.cpp"