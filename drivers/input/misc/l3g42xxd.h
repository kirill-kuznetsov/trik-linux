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
