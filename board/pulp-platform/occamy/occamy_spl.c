#include <init.h>
#include <spl.h>
#include <misc.h>
#include <log.h>
#include <asm/spl.h>


int spl_board_init_f(void)
{
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
