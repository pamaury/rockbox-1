#define CONFIG_RKNANO
#define IRAM_ORIG       0x1000000
#define IRAM_SIZE       0x8000
#define DRAM_ORIG       0x20000000
#define DRAM_SIZE       (MEMORYSIZE * 0x100000)
#define CPU_ARM
#define ARM_ARCH        7
#define USB_BASE            0x62000000
#define USB_NUM_ENDPOINTS   2