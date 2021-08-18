#include <init.h>
#include <spl.h>
#include <misc.h>
#include <log.h>
#include <asm/spl.h>

#define SOC_CTRL_BASE 0x2000000
#define ISOLATE_0 0xa0
#define ISOLATE_1 0xa4
#define ISOLATED_0 0xa8
#define ISOLATED_1 0xac
#define SOC_CTRL(reg) *(int32_t*)(SOC_CTRL_BASE + reg)


int spl_board_init_f(void)
{
	/* disable AXI isolation */
	SOC_CTRL(ISOLATE_0) = 0;
	SOC_CTRL(ISOLATE_1) = 0;

	return 0;
}

u32 spl_boot_device(void)
{
	/* boot from SPI flash */
	return BOOT_DEVICE_SPI;
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* boot using first FIT config */
	return 0;
}
#endif
