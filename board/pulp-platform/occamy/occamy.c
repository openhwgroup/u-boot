#include <init.h>

int board_init(void) {
  debug("%s: could not enable cache ways\n", __func__);
  return 0;
}