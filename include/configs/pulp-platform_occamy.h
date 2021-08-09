#ifndef __CONFIG_H
#define __CONFIG_H

#include <linux/sizes.h>

#ifdef CONFIG_SPL
#define CONFIG_SPL_MAX_SIZE		0x00020000

#define CONFIG_SPL_BSS_START_ADDR	0x70018000
#define CONFIG_SPL_BSS_MAX_SIZE		0x00008000

// We need more space than the SPM can provide, so put the heap into HBM.
// malloc should only be available _after_ the HBM is initialised.
#define CONFIG_SYS_SPL_MALLOC_START	0x84000000
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x00100000

#define CONFIG_SPL_STACK	0x70018000
#endif

#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_SDRAM_BASE + SZ_2M)

#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + SZ_2M)

#define CONFIG_SYS_MALLOC_LEN       SZ_8M
#define CONFIG_SYS_BOOTM_LEN        SZ_16M

#define CONFIG_SYS_MAX_FLASH_BANKS 1
#define CONFIG_SYS_FLASH_BASE 0x0
#endif
