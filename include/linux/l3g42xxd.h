/* Copyright 2014 - 2015 CyberTech Labs Ltd.
 *
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, CyberTech Labs  SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*/

#ifndef __L3G4200D_H__
#define __L3G4200D_H__

#include <linux/ioctl.h>  /* For IOCTL macros */



#define L3GD20_ID 		0xD4 

struct l3g42xxd_platform_data {
        int gpio_drdy;
        int gpio_int1;
};

#endif  /* __L3G4200D_H__ */
