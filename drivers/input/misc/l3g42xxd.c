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

#include <linux/time.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/pm.h>
#include <linux/module.h>
#include "l3g42xxd.h"

#define L3G4200D_GYR_DEV_NAME     "L3G42xxd gyroscope"
#define RANGE_MAX      
/** Register map */
#define L3G4200D_WHO_AM_I               0x0f
#define L3G4200D_CTRL_REG1              0x20
#define L3G4200D_CTRL_REG2              0x21
#define L3G4200D_CTRL_REG3              0x22
#define L3G4200D_CTRL_REG4              0x23
#define L3G4200D_CTRL_REG5              0x24

#define L3G4200D_REF_DATA_CAP           0x25
#define L3G4200D_OUT_TEMP               0x26
#define L3G4200D_STATUS_REG             0x27

#define L3G4200D_STATUS_REG_EXPECTED    0x0f


#define L3G4200D_OUT_X_L                0x28
#define L3G4200D_OUT_X_H                0x29
#define L3G4200D_OUT_Y_L                0x2a
#define L3G4200D_OUT_Y_H                0x2b
#define L3G4200D_OUT_Z_L                0x2c
#define L3G4200D_OUT_Z_H                0x2d

#define L3G4200D_FIFO_CTRL              0x2e
#define L3G4200D_FIFO_SRC               0x2f

#define L3G4200D_INTERRUPT_CFG          0x30
#define L3G4200D_INTERRUPT_SRC          0x31
#define L3G4200D_INTERRUPT_THRESH_X_H   0x32
#define L3G4200D_INTERRUPT_THRESH_X_L   0x33
#define L3G4200D_INTERRUPT_THRESH_Y_H   0x34
#define L3G4200D_INTERRUPT_THRESH_Y_L   0x35
#define L3G4200D_INTERRUPT_THRESH_Z_H   0x36
#define L3G4200D_INTERRUPT_THRESH_Z_L   0x37
#define L3G4200D_INTERRUPT_DURATION     0x38

// struct l3g42xxd_bus_op{
//     u16 bustype;
//     int (*read)(struct device *, unsigned char);
//     int (*read_block)(struct device *, unsigned char, int, void *);
//     int (*write)(struct device *, unsigned char, unsigned char);
// };


struct l3g42xxd_axis {
    int x;
    int y;
    int z;
};

struct l3g42xxd_stat {
    u8 fifomode; // BYPASS;FIFO_MODE;STREAM_MODE;BYPASS_STREAM; STREAM_FIFO;
    
    int odr; // 95; 190; 380; 760;
    u8 fs_range;
    u32 sensitivity;


    bool polling_enabled;
    unsigned int poll_interval;


    atomic_t enabled;

};
struct l3g42xxd_chip {
    struct device *dev;
    struct input_dev *input_dev;
    struct mutex lock;

    struct l3g42xxd_platform_data pdata;
    struct workqueue_struct *l3g42xxd_workqueue;
    struct l3g42xxd_stat stat;
    unsigned model;

    const struct l3g42xxd_bus_op* bops;
};
// option enable/disable
// fs_range
// odr_range


struct l3g42xxd_chip * l3g42xxd_probe(  struct device *dev, const struct l3g42xxd_bus_op* bops)
{
    struct l3g42xxd_chip* chip;
    const struct l3g42xxd_platform_data *pdata;
    int err,revid;

    pdata = dev->platform_data;
    if(!pdata){
        pr_err("%s : no platform data\n",__func__);
        err = -EINVAL;
        goto exit_no_pdata;
    }
    chip = kzalloc(sizeof(*chip), GFP_KERNEL);
    if (!chip){
        pr_err("%s : failed to allocate memory for module(chip) data\n",__func__);
        err = -ENOMEM;
        goto exit_no_chip_memory;
    }
    chip->input_dev =  input_allocate_device();
    if (!chip->input_dev){
        err = -ENOMEM;
        pr_err("%s:input device allocate failed\n",__func__);
        goto exit_input_dev_alloc;
    }

    chip->pdata     = *pdata;
    pdata           = &chip->pdata;

    chip->dev       = dev;
    chip->bops      = bops;

    mutex_init(&chip->lock);

   
    revid = chip->bops->read(chip->dev, L3G4200D_WHO_AM_I);
    switch (revid){
        case 0xD4:
        case 0xD3:
            break;
        default:
            pr_err("%s:Unknown chip :0%02x \n", __func__, revid);
            err =  -ENODEV;
            goto exit_device_unknown;
    }
    
    chip->input_dev->name = L3G4200D_GYR_DEV_NAME;
    chip->input_dev->dev.parent = chip->dev;
    chip->input_dev->id.product = revid;
    chip->input_dev->id.bustype = chip->bops->bustype;

    // chip->input_dev->open = 
    // chip->input_dev->close = 

    input_set_drvdata(chip->input_dev, chip);
// init input dev
    set_bit(EV_ABS,chip->input_dev->evbit);
    set_bit(ABS_X, chip->input_dev->absbit);
    set_bit(ABS_Y, chip->input_dev->absbit);
    set_bit(ABS_Z, chip->input_dev->absbit);
    input_set_abs_params(chip->input_dev, ABS_X, SHRT_MIN, SHRT_MAX, 0, 0);
    input_set_abs_params(chip->input_dev, ABS_Y, SHRT_MIN, SHRT_MAX, 0, 0);
    input_set_abs_params(chip->input_dev, ABS_Z, SHRT_MIN, SHRT_MAX, 0, 0);


    pr_err("%s: Done!\n", __func__);
    return chip;
exit_device_unknown:
    input_free_device(chip->input_dev);
exit_input_dev_alloc:
    kfree(chip);
exit_no_chip_memory:
exit_no_pdata:
    return ERR_PTR(err);
}

EXPORT_SYMBOL_GPL(l3g42xxd_probe);

void l3g42xxd_suspend(struct l3g42xxd_chip *chip)
{
}
EXPORT_SYMBOL_GPL(l3g42xxd_suspend);

void l3g42xxd_resume(struct l3g42xxd_chip *ac)
{
}
EXPORT_SYMBOL_GPL(l3g42xxd_resume);

void l3g42xxd_remove(struct l3g42xxd_chip *chip)
{
    input_free_device(chip->input_dev);
    kfree(chip);
    pr_err("%s: Done!\n", __func__);
}
EXPORT_SYMBOL_GPL(l3g42xxd_remove);

MODULE_DESCRIPTION("L3g42xxd Gyroscope");
MODULE_AUTHOR("romik.momik@trikset.com");
MODULE_LICENSE("GPL v2");
