#ifndef __L3G4200D_H__
#define __L3G4200D_H__

#include <linux/ioctl.h>  /* For IOCTL macros */



#define L3GD20_ID 		0xD4 

struct l3g42xxd_platform_data {
        int gpio_drdy;
        int gpio_int1;
};

#endif  /* __L3G4200D_H__ */
