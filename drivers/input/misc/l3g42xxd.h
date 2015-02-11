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
 
#ifndef _L3G42XXD__INT_H_
#define _L3G42XXD__INT_H_

#include <linux/types.h>
#include <linux/l3g42xxd.h>

struct device;
struct l3g42xxd_chip;

struct l3g42xxd_bus_op{
	u16 bustype;
	int (*read)(struct device *, unsigned char);
	int (*read_block)(struct device *, unsigned char, int, void *);
	int (*write)(struct device *, unsigned char, unsigned char);
};

void l3g42xxd_suspend(struct l3g42xxd_chip *chip);
void l3g42xxd_resume(struct l3g42xxd_chip *chip);

struct l3g42xxd_chip * l3g42xxd_probe(  struct device *dev,
										const struct l3g42xxd_bus_op* bops);
void l3g42xxd_remove(struct l3g42xxd_chip *chip);


#endif  /*_L3G42XXD__INT_H_ */
