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

#define L3G42XXD_GYR_DEV_NAME     "L3G42xxd gyroscope"
#define RANGE_MAX      
/** Register map */
#define L3G42XXD_WHO_AM_I               0x0f
#define L3G42XXD_CTRL_REG1              0x20
#define L3G42XXD_CTRL_REG2              0x21
#define L3G42XXD_CTRL_REG3              0x22
#define L3G42XXD_CTRL_REG4              0x23
#define L3G42XXD_CTRL_REG5              0x24

#define L3G42XXD_REF_DATA_CAP           0x25
#define L3G42XXD_OUT_TEMP               0x26
#define L3G42XXD_STATUS_REG             0x27

#define L3G42XXD_STATUS_REG_EXPECTED    0x0f


#define L3G42XXD_OUT_X_L                0x28
#define L3G42XXD_OUT_X_H                0x29
#define L3G42XXD_OUT_Y_L                0x2a
#define L3G42XXD_OUT_Y_H                0x2b
#define L3G42XXD_OUT_Z_L                0x2c
#define L3G42XXD_OUT_Z_H                0x2d

#define L3G42XXD_FIFO_CTRL              0x2e
#define L3G42XXD_FIFO_SRC               0x2f

#define L3G42XXD_INTERRUPT_CFG          0x30
#define L3G42XXD_INTERRUPT_SRC          0x31
#define L3G42XXD_INTERRUPT_THRESH_X_H   0x32
#define L3G42XXD_INTERRUPT_THRESH_X_L   0x33
#define L3G42XXD_INTERRUPT_THRESH_Y_H   0x34
#define L3G42XXD_INTERRUPT_THRESH_Y_L   0x35
#define L3G42XXD_INTERRUPT_THRESH_Z_H   0x36
#define L3G42XXD_INTERRUPT_THRESH_Z_L   0x37
#define L3G42XXD_INTERRUPT_DURATION     0x38

// struct l3g42xxd_bus_op{
//     u16 bustype;
//     int (*read)(struct device *, unsigned char);
//     int (*read_block)(struct device *, unsigned char, int, void *);
//     int (*write)(struct device *, unsigned char, unsigned char);
// };


#define L3G42XXD_SENSITIVITY_250     8750        /*  udps/LSB */
#define L3G42XXD_SENSITIVITY_500     17500       /*  udps/LSB */
#define L3G42XXD_SENSITIVITY_2000    70000       /*  udps/LSB */

#define MASK_EMPTY      (0x00)

//      CTRL_REG1 masks

#define MASK_PD_OFF             (0x00)
#define MASK_PD_NORMAL          (0x08)
#define MASK_ALL_AXES_ENABLE    (0x07)
#define MASK_ALL_AXES_DISABLE   (0x00)

#define MASK_BW00               (0x00)
#define MASK_BW01               (0x10)
#define MASK_BW10               (0x20)
#define MASK_BW11               (0x30)

#define MASK_ODR095             (0x00)
#define MASK_ODR190             (0x40)
#define MASK_ODR380             (0x80)
#define MASK_ODR760             (0xC0)
#define MASK_ODR                (0xC0)
//     CTRL_REG3 masks

#define MASK_I2_DRDY            (0x08)
#define MASK_I2_WTM             (0x04)
#define MASK_I2_OVRUN           (0x02)
#define MASK_I2_EMPTY           (0x01)
#define MASK_I2_NONE            (0x00)

#define MASK_FIFO_I2            (0x07)

//     CTRL_REG4 masks

#define L3G42XXD_FS_250DPS        (0x00)
#define L3G42XXD_FS_500DPS        (0x10)
#define L3G42XXD_FS_2000DPS       (0x30)

#define MASK_FS_MASK            (0x30)
#define MASK_BDU_ENABLE         (0x80)
#define MASK_BLE                (0x40)

//     CTRL_REG5 masks

#define MASK_FIFO_EN            (0x40)

// STATUS_REG mask

// FIFO_CTRL_REG mask
#define MASK_FIFO_MODE_MASK             (0xE0)
#define MASK_FIFO_MODE_BYPASS           (0x00)
#define MASK_FIFO_MODE_FIFO             (0x20)
#define MASK_FIFO_MODE_STREAM           (0x40)
#define MASK_FIFO_MODE_STR2FIFO         (0x60)
#define MASK_FIFO_MODE_BYPASS2STR       (0x80)
#define MASK_FIFO_WATERMARK             (0x1F)

#define RES_CTRL_REG1       0
#define RES_CTRL_REG2       1
#define RES_CTRL_REG3       2
#define RES_CTRL_REG4       3
#define RES_CTRL_REG5       4
#define RES_FIFO_CTRL_REG   5


struct l3g42xxd_axis {
    s32 x;
    s32 y;
    s32 z;
};

struct l3g42xxd_stat {
    u8 fifomode; // BYPASS;FIFO_MODE;STREAM_MODE;BYPASS_STREAM; STREAM_FIFO;
    u8 watermark;


    int odr; // 95; 190; 380; 760 Hz
    u8 fs_range; // 250; 500; 2000 dps
    u32 sensitivity; // 8700; 17500; 70000 udps/LSB

    bool polling_enabled;
    unsigned int poll_interval;
    
    atomic_t enabled;
    u8 reg_state[6];
    bool hw_initialized;
};
static struct l3g42xxd_stat l3g42xxd_stat_default = {
    .fifomode           = MASK_FIFO_MODE_BYPASS,
    .watermark          = 0,
    .odr                = MASK_ODR095,
    .fs_range           = L3G42XXD_FS_250DPS,
    .sensitivity        = L3G42XXD_SENSITIVITY_250,
    .polling_enabled    = false,
    .poll_interval      = 0,
    .enabled            = ATOMIC_INIT(0),
    .reg_state = {
        (MASK_EMPTY|MASK_ALL_AXES_ENABLE|MASK_PD_NORMAL),
        (MASK_EMPTY),
        (MASK_EMPTY),
        (MASK_EMPTY|MASK_BDU_ENABLE),
        (MASK_EMPTY),
        (MASK_EMPTY),
    },
    .hw_initialized = false,

};
struct l3g42xxd_chip {
    struct device *dev;
    struct input_dev *input_dev;
    struct mutex lock;

    struct l3g42xxd_platform_data pdata;
    struct workqueue_struct *l3g42xxd_workqueue;
    struct l3g42xxd_stat* stat;
    unsigned model;
    const struct l3g42xxd_bus_op* bops;

};
// option enable/disable
// fs_range
// odr_range
static int l3g42xxd_input_init(struct l3g42xxd_chip* chip ){
    int err = -1;
    int revid;
    chip->input_dev =  input_allocate_device();
    if (!chip->input_dev){
        err = -ENOMEM;
        pr_err("%s:input device allocate failed\n",__func__);
        goto exit_input_dev_alloc;
    }
    revid = chip->bops->read(chip->dev, L3G42XXD_WHO_AM_I);
    switch (revid){
        case 0xD4:
        case 0xD3:
            break;
        default:
            pr_err("%s:Unknown chip :0%02x \n", __func__, revid);
            err =  -ENODEV;
            goto exit_input_dev_unknown;
    }
   
    chip->input_dev->name = L3G42XXD_GYR_DEV_NAME;
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


    err = input_register_device(chip->input_dev);
    if (err) {
        pr_err("%s: unable to register input device \n", __func__);
        goto exit_input_dev_register;
    }
    return 0;
exit_input_dev_register:
exit_input_dev_unknown:
    input_free_device(chip->input_dev);
exit_input_dev_alloc:
    return err;
}
static void l3g42xxd_input_finit(struct l3g42xxd_chip* chip ){
    input_unregister_device(chip->input_dev);
    input_free_device(chip->input_dev);
}

static int l3g42xxd_hardware_init(struct l3g42xxd_chip* chip) {
    int err;
    //TODO: write block
    err = chip->bops->write(chip->dev,L3G42XXD_CTRL_REG1, chip->stat->reg_state[RES_CTRL_REG1]);
    if (err < 0){
         pr_err("%s:can't write into CTRL_REG1 \n", __func__);
         return err;
    }    
    err = chip->bops->write(chip->dev,L3G42XXD_CTRL_REG2, chip->stat->reg_state[RES_CTRL_REG2]);
    if (err < 0){
         pr_err("%s:can't write into CTRL_REG1 \n", __func__);
         return err;
    }  
    err = chip->bops->write(chip->dev,L3G42XXD_CTRL_REG3, chip->stat->reg_state[RES_CTRL_REG3]);
    if (err < 0){
         pr_err("%s:can't write into CTRL_REG1 \n", __func__);
         return err;
    }  
    err = chip->bops->write(chip->dev,L3G42XXD_CTRL_REG4, chip->stat->reg_state[RES_CTRL_REG4]);
    if (err < 0){
         pr_err("%s:can't write into CTRL_REG1 \n", __func__);
         return err;
    }  
    err = chip->bops->write(chip->dev,L3G42XXD_CTRL_REG5, chip->stat->reg_state[RES_CTRL_REG5]);
    if (err < 0){
         pr_err("%s:can't write into CTRL_REG1 \n", __func__);
         return err;
    }  
    err =  chip->bops->write(chip->dev,L3G42XXD_FIFO_CTRL, chip->stat->reg_state[RES_FIFO_CTRL_REG]);
    if (err < 0){
         pr_err("%s:can't write into CTRL_REG1 \n", __func__);
         return err;
    }  
    chip->stat->hw_initialized = true;
    return err;   
}
static int l3g42xxd_update_odr( struct l3g42xxd_chip* chip, u8 new_odr){
    int err = -1;
    switch (new_odr){
        case MASK_ODR095:
        case MASK_ODR190:
        case MASK_ODR380:
        case MASK_ODR760:
            break;
        default:
            pr_err("%s:invalid odr value regueted: %u \n", __func__, new_odr);
            return  -EINVAL;
    }
    //todo:
    if (!atomic_read(&chip->stat->enabled)){
        u8 newvalue  = ( (chip->stat->reg_state[RES_CTRL_REG1] & (~MASK_ODR)) |
                                                    ( MASK_ODR & new_odr ));
        err = chip->bops->write(chip->dev,L3G42XXD_CTRL_REG1,newvalue);
        if (err < 0) {
            return err;
        }
        chip->stat->reg_state[RES_CTRL_REG1] = newvalue;
        chip->stat->odr = new_odr;
    }
    return 0;
}
static int l3g42xxd_update_fs_range(struct l3g42xxd_chip* chip,u8 new_fs) {
    int err;
    u32 sensitivity;
    switch(new_fs) {
        case L3G42XXD_FS_250DPS:
            sensitivity = L3G42XXD_SENSITIVITY_250;
            break;
        case L3G42XXD_FS_500DPS:
            sensitivity = L3G42XXD_SENSITIVITY_500;
            break;
        case L3G42XXD_FS_2000DPS:
            sensitivity = L3G42XXD_SENSITIVITY_2000;
            break;
        default:
            pr_err("%s:invalid g range requested: %u\n", __func__, new_fs);
            return -EINVAL;
    }
    if (!atomic_read(&chip->stat->enabled)){
        u8 newvalue  = ((chip->stat->reg_state[RES_CTRL_REG4] & (~MASK_FS_MASK)) |
                                                      (new_fs & MASK_FS_MASK));
        err = chip->bops->write(chip->dev,L3G42XXD_CTRL_REG4,newvalue);
        if (err < 0) {
            return err;
        }
        chip->stat->sensitivity = sensitivity;
        chip->stat->fs_range = new_fs;
        chip->stat->reg_state[RES_CTRL_REG4] = newvalue;
    }
    return 0;
}
static int l3g42xxd_get_data(struct l3g42xxd_chip* chip,struct l3g42xxd_axis* axis){
    int err;
    u8 buf[6];
    err = chip->bops->read_block(chip->dev, L3G42XXD_OUT_X_L, sizeof(buf)/sizeof(buf[0]), buf);
    if (err < 0)
        return err;
    axis->x  = (s32) ((s16)((buf[1]) << 8) | buf[0]);
    axis->y = (s32) ((s16)((buf[3]) << 8) | buf[2]);
    axis->z = (s32) ((s16)((buf[5]) << 8) | buf[4]);
    return 0;
}
static void l3g42xxd_report_values (struct l3g42xxd_chip* chip,struct l3g42xxd_axis * axis ){
    input_report_abs(chip->input_dev, ABS_X, axis->x);
    input_report_abs(chip->input_dev, ABS_Y, axis->y);
    input_report_abs(chip->input_dev, ABS_Z, axis->z);
    input_sync(chip->input_dev);
}
struct l3g42xxd_chip * l3g42xxd_probe( struct device *dev, const struct l3g42xxd_bus_op* bops)
{
    struct l3g42xxd_chip* chip;
    const struct l3g42xxd_platform_data *pdata;
    int err = -1;

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

    chip->pdata     = *pdata;
    pdata           = &chip->pdata;
    chip->dev       = dev;
    chip->bops      = bops;

    mutex_init(&chip->lock);
    
    chip->stat = &l3g42xxd_stat_default;
    atomic_set(&chip->stat->enabled, 0);
    //
    err = l3g42xxd_hardware_init (chip);
    if (err) {
         pr_err("%s : failed to init registers \n",__func__);
         goto exit_hw_init;
    }

    // 
   //l3g42xxd_input_init(chip);

    pr_err("%s: Done!\n", __func__);
    return chip;
exit_hw_init:
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
  //l3g42xxd_input_finit(chip);
    kfree(chip);
    pr_err("%s: Done!\n", __func__);
}
EXPORT_SYMBOL_GPL(l3g42xxd_remove);

MODULE_DESCRIPTION("L3g42xxd Gyroscope");
MODULE_AUTHOR("romik.momik@trikset.com");
MODULE_LICENSE("GPL v2");
