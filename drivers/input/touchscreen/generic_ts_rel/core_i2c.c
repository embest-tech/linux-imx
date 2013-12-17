/* 
 * drivers/input/touchscreen/generic_ts.c
 *
 * generic touchscreen driver. 
 *
 * Copyright (c) 2012 ~ 2013.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * George Chen, 2012-06-15
 */

 /*
	TODO: 
		xxxx_data validation check
		file system access
*/

// ****************************************************************************
// Includes
// ****************************************************************************
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include "core_i2c.h"

#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_DEBUG_CORE
#define	PRINT_CORE_MSG(msg...) printk(msg)
#else
#define	PRINT_CORE_MSG(msg...)
#endif

// ****************************************************************************
// Globel or static variables
// ****************************************************************************
static
char bindata[] = {
#include "CT363_V01_0607_130603.dat"
};

//static
struct struct_core_i2c_var *core_data = NULL;

static 
struct i2c_device_id i2c_dev_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};


static int core_probe(struct i2c_client *client, const struct i2c_device_id *id);
static void core_shutdown(struct i2c_client *client);
static int core_suspend(struct i2c_client *client, pm_message_t mesg);
static int core_resume(struct i2c_client *client);
static int __devexit core_remove(struct i2c_client *client);

static
struct i2c_driver generic_ts_driver  = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME
	},
	.id_table	= i2c_dev_id,
	.probe		= core_probe,
	.shutdown	= core_shutdown,
	.suspend	= core_suspend,
	.resume		= core_resume,
	.remove 	= __devexit_p(core_remove),
};

// ****************************************************************************
// Function declaration
// ****************************************************************************
static 
struct struct_core_i2c_var* core_var_init(void)
{
	struct struct_core_i2c_var* local_core_data = NULL;

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	local_core_data = kzalloc(sizeof(struct struct_core_i2c_var), GFP_KERNEL);
	if ( local_core_data ) {
		local_core_data->ops.platform_var_init = platform_var_init;
		local_core_data->ops.platform_var_exit = platform_var_exit;

		local_core_data->ops.chip_var_init = chip_var_init;
		local_core_data->ops.chip_var_exit = chip_var_exit;

		local_core_data->ops.touch_var_init = touch_var_init;
		local_core_data->ops.touch_var_exit = touch_var_exit;

		local_core_data->ops.fsutils_var_init = fsutils_var_init;
		local_core_data->ops.fsutils_var_exit = fsutils_var_exit;

	#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_ENHANCE
		local_core_data->ops.enhance_var_init = enhance_var_init;
		local_core_data->ops.enhance_var_exit = enhance_var_exit;
	#endif
	}

	return local_core_data;
}

static 
void core_var_exit(struct struct_core_i2c_var* core_data)
{
	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( core_data ) {
		if ( core_data->ops.enhance_var_exit )
			core_data->ops.enhance_var_exit(core_data->edata);

		if ( core_data->ops.fsutils_var_exit )
			core_data->ops.fsutils_var_exit(core_data->fdata);
		
		if ( core_data->ops.touch_var_exit )
			core_data->ops.touch_var_exit(core_data->tdata);
		
		if ( core_data->ops.chip_var_exit )
			core_data->ops.chip_var_exit(core_data->cdata);
		
		if ( core_data->ops.platform_var_exit )
			core_data->ops.platform_var_exit(core_data->pdata);

		kfree(core_data);
	}
}

static 
irqreturn_t core_irq_handler(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;
	struct struct_core_i2c_var *core_data = (struct struct_core_i2c_var *)dev;

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( core_data->pdata->i2c->ops.chk_irq ) 
	if ( core_data->pdata->i2c->ops.chk_irq(core_data->pdata->i2c) ) 
	{
		// touch device is ready??
		if ( core_data->state == CORE_STATE_NORMAL ) {
			// Disable ts interrupt
			disable_irq_nosync(core_data->pdata->i2c->irq);
			queue_work(core_data->workqueue, &core_data->event_work);
		}
		ret = IRQ_HANDLED;
	}

	return ret;
}

static 
void core_workfunc(struct work_struct *work)
{
	struct struct_core_i2c_var *core_data = container_of(work, struct struct_core_i2c_var, event_work);
	char *buf = (char*)core_data->cdata->pts_data;

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);
	//printk("Get Touch Points\n");

	/* read touch points */
	buf[0] = 0x00;	// i2c offset
	if ( core_data->pdata->i2c->ops.i2c_read )
		core_data->pdata->i2c->ops.i2c_read(
			core_data->pdata->i2c->client, 
			core_data->pdata->i2c->i2c_addr, 
			buf, 
			core_data->cdata->pt_sz * core_data->cdata->pts_num);

	if ( core_data->tdata->ops.obtain_pts )
		core_data->tdata->ops.obtain_pts(
			core_data->tdata, 
			core_data->cdata->pts_data, 
			core_data->cdata->ops.cpy_valid_pt, 
			core_data->cdata->pt_sz);

#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_ENHANCE
#if ENHANCE_PREPROC_EN
	/* preprocessing */
	if ( core_data->edata->preproc->ops.obtain_pts )
		core_data->edata->preproc->ops.obtain_pts(
			core_data->edata->preproc, 
			core_data->tdata->pts, 
			core_data->tdata->ops.cpy_valid_pt, 
			core_data->tdata->pt_sz, 
			core_data->tdata->valid_cur_pts_num);

	if ( core_data->edata->preproc->avg_dnsty_en ) {
		if ( core_data->edata->preproc->ops.avg_pts_dnsty )
			core_data->edata->preproc->ops.avg_pts_dnsty(core_data->edata->preproc);
	}

	if ( core_data->edata->preproc->ops.update_pts )
		core_data->edata->preproc->ops.update_pts(
			core_data->tdata->pts, 
			core_data->edata->preproc, 
			core_data->tdata->ops.upd_valid_pt, 
			core_data->tdata->pt_sz);
#endif

#if ENHANCE_TRACK_EN
	/* tracking */
	if ( core_data->edata->track->ops.backup_pts )
		core_data->edata->track->ops.backup_pts(core_data->edata->track);

	if ( core_data->edata->track->ops.obtain_pts )
		core_data->edata->track->ops.obtain_pts(
			core_data->edata->track, 
			core_data->tdata->pts, 
			core_data->tdata->ops.cpy_valid_pt, 
			core_data->tdata->pt_sz, 
			core_data->tdata->valid_cur_pts_num);

	if ( core_data->edata->track->ops.do_track )
		core_data->edata->track->ops.do_track(core_data->edata->track);

	if ( core_data->edata->track->ops.update_pts )
		core_data->edata->track->ops.update_pts(
			core_data->tdata->pts, 
			core_data->edata->track, 
			core_data->tdata->ops.upd_valid_pt, 
			core_data->tdata->pt_sz);
#endif

#if ENHANCE_PREPROC_EN
	/* postprocessing */
	if ( core_data->edata->postproc->ops.obtain_pts )
		core_data->edata->postproc->ops.obtain_pts(
			core_data->edata->postproc, 
			core_data->tdata->pts, 
			core_data->tdata->ops.cpy_valid_pt, 
			core_data->tdata->pt_sz, 
			core_data->tdata->valid_cur_pts_num);

	if ( core_data->edata->postproc->pwrns_pts_corr_en ) {
		if ( core_data->edata->postproc->ops.pwrns_pts_corr )
			core_data->edata->postproc->ops.pwrns_pts_corr(core_data->edata->postproc, core_data->edata->track);
	}

	if ( core_data->edata->postproc->ops.update_pts )
		core_data->edata->postproc->ops.update_pts(
			core_data->tdata->pts, 
			core_data->edata->postproc, 
			core_data->tdata->ops.upd_valid_pt, 
			core_data->tdata->pt_sz);
#endif

#if ENHANCE_CURVE_EN
	/* curve fitting */
	if ( core_data->edata->curve->ops.obtain_pts )
		core_data->edata->curve->ops.obtain_pts(
			core_data->edata->curve, 
			core_data->tdata->pts, 
			core_data->tdata->ops.cpy_valid_pt, 
			core_data->tdata->pt_sz, 
			core_data->tdata->valid_cur_pts_num);

	if ( core_data->edata->curve->fltr_en ) {
		if ( core_data->edata->curve->ops.init_fltr )
			core_data->edata->curve->ops.init_fltr(core_data->edata->curve, core_data->edata->track->dist_pts);
		if ( core_data->edata->curve->ops.do_fltr )
			core_data->edata->curve->ops.do_fltr(core_data->edata->curve, core_data->edata->track->dist_pts);
	}

	if ( core_data->edata->curve->ops.update_pts )
		core_data->edata->curve->ops.update_pts(
			core_data->tdata->pts, 
			core_data->edata->curve, 
			core_data->tdata->ops.upd_valid_pt, 
			core_data->tdata->pt_sz);
#endif

#if ENHANCE_SCALE_EN
	/* scaling */
	if ( core_data->edata->scale->ops.obtain_pts )
		core_data->edata->scale->ops.obtain_pts(
			core_data->edata->scale, 
			core_data->tdata->pts, 
			core_data->tdata->ops.cpy_valid_pt, 
			core_data->tdata->pt_sz, 
			core_data->tdata->valid_cur_pts_num);

	if ( core_data->edata->scale->scale_en ) {
		if ( core_data->edata->scale->ops.do_scale )
			core_data->edata->scale->ops.do_scale(core_data->edata->scale);
	}

	if ( core_data->edata->scale->ops.update_pts )
		core_data->edata->scale->ops.update_pts(
			core_data->tdata->pts, 
			core_data->edata->scale, 
			core_data->tdata->ops.upd_valid_pt, 
			core_data->tdata->pt_sz);
#endif
#endif

	/* report touch points */
	if ( core_data->tdata->ops.report_pts ) {
		core_data->tdata->ops.report_pts(core_data->tdata);
	}
	//printk("Report Touch Points\n");

	// Enable ts interrupt
	enable_irq(core_data->pdata->i2c->irq);

}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void core_early_suspend(struct early_suspend *handler);
static void core_early_resume(struct early_suspend *handler);
#endif

static
int core_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = -1;
	int vendor;
	int binchksum, fwchksum;
	int updcnt;
	struct device *dev = &client->dev;

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	core_data->state = CORE_STATE_INIT;

	i2c_set_clientdata(client, core_data);

	/* Check I2C Functionality */
	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if ( !err ) {
		dev_err(dev, "Check I2C Functionality Failed.\n");
		goto ERR_I2C_CHK;
	}

	/* Request platform resources (gpio/interrupt pins) */
	if ( core_data->pdata->i2c->ops.get_rsrc ) {
		err = core_data->pdata->i2c->ops.get_rsrc(core_data->pdata->i2c);
		if ( err ) {
			dev_err(dev, "Unable to request platform resource for device %s.\n", DRIVER_NAME);
			goto ERR_PLAT_RSC;
		}
	}

	/* Hardware reset */
	if ( core_data->pdata->i2c->ops.hw_reset ) {
		core_data->pdata->i2c->ops.hw_reset(core_data->pdata->i2c);
	}

	/* chip data allocation */
#ifndef CONFIG_TOUCHSCREEN_GENERIC_TS_MISC_AUTO_CHIP_SELECT
	/* manually chip select */
	if ( core_data->ops.chip_var_init ) {
		struct struct_chip_param param;
	#if defined(CONFIG_TOUCHSCREEN_GENERIC_TS_CHIP_CT360)
		param.chip = CHIP_CT360;
		param.chn_x_num = CT360_CHN_X_MAX;
		param.chn_y_num = CT360_CHN_Y_MAX;
		param.chn_res = CT360_CHN_RES;
		param.x_max = CT360_ABS_X_MAX;
		param.y_max = CT360_ABS_Y_MAX;
		param.xy_swap = CHIP_XY_SWAP;
		param.pts_num = CT360_POINT_NUM;
	#elif defined(CONFIG_TOUCHSCREEN_GENERIC_TS_CHIP_CT365)
		param.chip = CHIP_CT365;
		param.chn_x_num = CT365_CHN_X_MAX;
		param.chn_y_num = CT365_CHN_Y_MAX;
		param.chn_res = CT365_CHN_RES;
		param.x_max = CT365_ABS_X_MAX;
		param.y_max = CT365_ABS_Y_MAX;
		param.xy_swap = CHIP_XY_SWAP;
		param.pts_num = CT365_POINT_NUM;
	#else
		#error No Chip Parameter defined
	#endif
		core_data->cdata = core_data->ops.chip_var_init(&param);

		/* Get chip id */
		if ( core_data->cdata->ops.get_vendor ) {
			vendor = core_data->cdata->ops.get_vendor(client, core_data->cdata->pts_data);
			printk("Chip ID: 0x%x\n", vendor);
		}

		if ( core_data->cdata->chip != vendor ) {
			printk("Chip data allocation failed: 0x%x\n", vendor);
			goto ERR_CDATA_ALLOC;
		}
	}
#else
	/* automatic chip select */
	if ( core_data->ops.chip_var_init && core_data->ops.chip_var_exit ) {
		int iter;
		struct struct_chip_param param;
		
		for ( iter = CHIP_CT365; iter < CHIP_MAX; iter++ ) {
			switch ( iter ) {
			case CHIP_CT360:
				param.chip = CHIP_CT360;
				param.chn_x_num = CT360_CHN_X_MAX;
				param.chn_y_num = CT360_CHN_Y_MAX;
				param.chn_res = CT360_CHN_RES;
				param.x_max = CT360_ABS_X_MAX;
				param.y_max = CT360_ABS_Y_MAX;
				param.xy_swap = CHIP_XY_SWAP;
				param.pts_num = CT360_POINT_NUM;
				break;
				
			case CHIP_CT365:
				param.chip = CHIP_CT365;
				param.chn_x_num = CT365_CHN_X_MAX;
				param.chn_y_num = CT365_CHN_Y_MAX;
				param.chn_res = CT365_CHN_RES;
				param.x_max = CT365_ABS_X_MAX;
				param.y_max = CT365_ABS_Y_MAX;
				param.xy_swap = CHIP_XY_SWAP;
				param.pts_num = CT365_POINT_NUM;
				break;

			default:
				param.chip = -1;	// skip
				printk("No Chip Parameter defined for id: 0x%x\n", param.chip);
				break;
			}

			if ( param.chip == -1 )
				continue;

			core_data->cdata = core_data->ops.chip_var_init(&param);
			/* Get chip id */
			if ( core_data->cdata->ops.get_vendor ) {
				vendor = core_data->cdata->ops.get_vendor(client, core_data->cdata->pts_data);
				printk("Chip ID: 0x%x\n", vendor);
			}
			if ( core_data->cdata->chip != vendor ) {
				core_data->ops.chip_var_exit(core_data->cdata);
			} else {
				break;
			}
		}

		if ( iter == CHIP_MAX ) {
			printk("Chip data allocation failed: 0x%x\n", vendor);
			goto ERR_CDATA_ALLOC;
		}
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
//	/* register early suspend */
	core_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	core_data->early_suspend.suspend = core_early_suspend;
	core_data->early_suspend.resume = core_early_resume;
	register_early_suspend(&core_data->early_suspend);
#endif

#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_MISC_AUTO_FW_UPDATE
	/* fw bootloader */
	memcpy(core_data->cdata->bin_data, bindata, core_data->cdata->bin_sz);
	// Get binary Checksum
	if ( core_data->cdata->ops.get_bin_chksum )
		binchksum = core_data->cdata->ops.get_bin_chksum(core_data->cdata->pts_data, core_data->cdata->bin_data);
	printk("Bin checksum: 0x%x\n", binchksum);

	// Get firmware Checksum
	if ( core_data->cdata->ops.get_fw_chksum )
		fwchksum = core_data->cdata->ops.get_fw_chksum(client, core_data->cdata->pts_data);
	printk("Fw checksum: 0x%x\n", fwchksum);

	/* Hardware reset */
	if ( core_data->pdata->i2c->ops.hw_reset )
		core_data->pdata->i2c->ops.hw_reset(core_data->pdata->i2c);

	updcnt = 1;
	while ( binchksum != fwchksum && updcnt-- ) {
		/* Update Firmware */
		if ( core_data->cdata->ops.update_fw )
			core_data->cdata->ops.update_fw(client, core_data->cdata->pts_data);

		/* Hardware reset */
		if ( core_data->pdata->i2c->ops.hw_reset )
			core_data->pdata->i2c->ops.hw_reset(core_data->pdata->i2c);

		// Get firmware Checksum
		if ( core_data->cdata->ops.get_fw_chksum )
		fwchksum = core_data->cdata->ops.get_fw_chksum(client, core_data->cdata->pts_data);
		printk("Fw checksum: 0x%x\n", fwchksum);

		/* Hardware reset */
		if ( core_data->pdata->i2c->ops.hw_reset )
			core_data->pdata->i2c->ops.hw_reset(core_data->pdata->i2c);
	}

	printk("Fw update %s. 0x%x, 0x%x\n", binchksum != fwchksum ? "Failed" : "Success", binchksum, fwchksum);
#endif

	/* Create work queue */
	INIT_WORK(&core_data->event_work, core_workfunc);
	core_data->workqueue = create_singlethread_workqueue(dev_name(&client->dev));

	/* Init irq */
	err = request_irq(core_data->pdata->i2c->irq, core_irq_handler, IRQF_TRIGGER_FALLING, DRIVER_NAME, core_data);
	if ( err ) {
		dev_err(dev, "Unable to request irq for device %s.\n", DRIVER_NAME);
		goto ERR_IRQ_REQ;
	}

	/* create touch data */
	if ( core_data->ops.touch_var_init ) {
		// parameters for touch
		struct struct_touch_param param;
		param.valid_max_pts_num = core_data->cdata->pts_num;
		param.x_max = core_data->cdata->x_max;
		param.y_max = core_data->cdata->y_max;
		param.x_rvs = TOUCH_X_REVERSE;
		param.y_rvs = TOUCH_Y_REVERSE;
		param.xy_swp = core_data->cdata->xy_swap;

		core_data->tdata = core_data->ops.touch_var_init(&param);

		if ( core_data->tdata == NULL )
			goto ERR_TDATA_ALLOC;
	}

	/* create fsutils data */
	if ( core_data->ops.fsutils_var_init ) {

		// parameters for fsutils
		struct struct_fsutils_param param;
		//param.cmd_num = strlen(fsutils_cmd_str);
		//param.cmd_ind = fsutils_cmd_ind;
		//param.cmd_str = fsutils_cmd_str;

		core_data->fdata = core_data->ops.fsutils_var_init(&param);

		if ( core_data->fdata == NULL )
			goto ERR_FDATA_ALLOC;
	}

	/* create enhance data */
	if ( core_data->ops.enhance_var_init ) {
		// better smoothness, worse tracking
		int fltr_coef[] = {
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		};
		struct struct_fltr_data fltr[] = {
			{
				.type = CURVE_FLTR_TYPE_FIR,
				.num = ENHANCE_CURVE_FILTER_PTS_MAX,
				.min_thrshld = 0,
				.max_thrshld = 0,
				.coef = fltr_coef,
			},
		};

		struct struct_enhance_param param;
		// parameters for preprocessing
		param.preproc_valid_max_pts_num = core_data->cdata->pts_num;
		param.avg_dnsty_en = ENHANCE_PREPROC_AVG_DNSTY_EN;
		param.avg_dnsty_chn_res = core_data->cdata->chn_res;
		param.avg_dnsty_delta = ENHANCE_PREPROC_AVG_DNSTY_DELTA;
		
		// parameters for tracking
		param.track_valid_max_pts_num = core_data->cdata->pts_num;
		param.dist_en = ENHANCE_TRACK_DIST_EN;
		param.track_min_thrshld_first = ENHANCE_TRACK_DIST_THRSHLD_FIRST;
		param.track_min_thrshld_track = ENHANCE_TRACK_DIST_THRSHLD_TRACK;
		param.track_valid_thrshld_min = ENHANCE_TRACK_VALID_THRSHLD_MIN;
		param.track_valid_thrshld_max = ENHANCE_TRACK_VALID_THRSHLD_MAX;
		
		// parameters for postprocessing
		param.postproc_valid_max_pts_num = core_data->cdata->pts_num;
		param.pwrns_pts_corr_en = ENHANCE_POSTPROC_PWRNS_PTS_CORR_EN;
		param.pwrns_pts_corr_stb_cnt = ENHANCE_POSTPROC_PWRNS_PTS_STB_CNT;
		param.pwrns_pts_corr_chn_res = core_data->cdata->chn_res;

		// parameters for curve fitting (denoise)
		param.curve_valid_max_pts_num = core_data->cdata->pts_num;
		param.fltr_en = ENHANCE_CURVE_FILTER_EN;
		param.fltr_lvl_num = sizeof(fltr)/sizeof(struct struct_fltr_data);
		param.fltr_pts_num = ENHANCE_CURVE_FILTER_PTS_MAX;
		param.fltr = fltr;

		// parameters for scaling
		param.scale_en = ENHANCE_SCALE_EN;
		param.scale_valid_max_pts_num = core_data->cdata->pts_num;
		param.scale_chn_x_num = core_data->cdata->chn_x_num;
		param.scale_chn_y_num = core_data->cdata->chn_y_num;
		param.scale_chn_res = core_data->cdata->chn_res;
		param.scale_x_max = core_data->cdata->x_max;
		param.scale_y_max = core_data->cdata->y_max;

		core_data->edata = core_data->ops.enhance_var_init(&param);
	}

	/* Set device is ready */
	core_data->state = CORE_STATE_NORMAL;

	return 0;

ERR_FDATA_ALLOC:
ERR_TDATA_ALLOC:
ERR_IRQ_REQ:
	cancel_work_sync(&core_data->event_work);
	destroy_workqueue(core_data->workqueue);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&core_data->early_suspend);
#endif

ERR_CDATA_ALLOC:

ERR_PLAT_RSC:
	if ( core_data->pdata->i2c->ops.put_rsrc )
		core_data->pdata->i2c->ops.put_rsrc(core_data->pdata->i2c);

ERR_I2C_CHK:
	i2c_set_clientdata(client, NULL);

	/* free allocated memory (MUST be the last call ) */
	core_var_exit(core_data);

	return err;
}

static
void core_shutdown(struct i2c_client *client)
{
	struct struct_core_i2c_var *core_data = i2c_get_clientdata(client);

        if ( core_data == NULL)
                return ;

	char *buf = (char*)core_data->cdata->pts_data;

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( core_data->state == CORE_STATE_NORMAL ) {
		if ( core_data->cdata->ops.go_sleep )
			core_data->cdata->ops.go_sleep(client, buf);
	}
}

static
int core_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct struct_core_i2c_var *core_data = i2c_get_clientdata(client);
	char *buf = (char*)core_data->cdata->pts_data;

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( core_data->state == CORE_STATE_NORMAL ) {
		core_data->state = CORE_STATE_SLEEP;
		disable_irq(core_data->pdata->i2c->irq);
		if ( core_data->cdata->ops.go_sleep )
			core_data->cdata->ops.go_sleep(client, buf);
	#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_ENHANCE
		core_data->edata->track->valid_cur_pts_num = 0;
		core_data->edata->curve->valid_cur_pts_num = 0;
		core_data->edata->curve->ops.init_fltr(core_data->edata->curve, NULL);
	#endif
		if ( core_data->tdata->ops.report_pts ) {
			core_data->tdata->valid_cur_pts_num = 0;
			core_data->tdata->ops.report_pts(core_data->tdata);
		}
	}

	return 0;
}

static
int core_resume(struct i2c_client *client)
{
	struct struct_core_i2c_var *core_data = i2c_get_clientdata(client);

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( core_data->state == CORE_STATE_SLEEP ) {
		/* Hardware reset */
		if ( core_data->pdata->i2c->ops.hw_reset )
			core_data->pdata->i2c->ops.hw_reset(core_data->pdata->i2c);
		enable_irq(core_data->pdata->i2c->irq);
		core_data->state = CORE_STATE_NORMAL;
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static 
void core_early_suspend(struct early_suspend *handler)
{
	struct struct_core_i2c_var *core_data = container_of(handler, struct struct_core_i2c_var, early_suspend);

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	core_suspend(core_data->pdata->i2c->client, PMSG_SUSPEND);
}

static 
void core_early_resume(struct early_suspend *handler)
{
	struct struct_core_i2c_var *core_data = container_of(handler, struct struct_core_i2c_var, early_suspend);

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	core_resume(core_data->pdata->i2c->client);
}
#endif

static
int __devexit core_remove(struct i2c_client *client)
{
	struct struct_core_i2c_var *core_data = i2c_get_clientdata(client);

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

        if ( core_data == NULL)
                return -1;

	free_irq(core_data->pdata->i2c->irq, core_data);
	cancel_work_sync(&core_data->event_work);
	destroy_workqueue(core_data->workqueue);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&core_data->early_suspend);
#endif
	if ( core_data->pdata->i2c->ops.put_rsrc )
		core_data->pdata->i2c->ops.put_rsrc(core_data->pdata->i2c);
	i2c_set_clientdata(client, NULL);
	core_var_exit(core_data);

	return 0;
}

int __init core_init(void)
{
	int err = -1;

	printk("generic touchscreen driver, <@>.\n");

	core_data = core_var_init();
	if ( core_data == NULL )
		goto ERR_VAR;

	core_data->state = CORE_STATE_UNKNOWN;

	if ( core_data->ops.platform_var_init ) {
                struct struct_platform_param param;
                param.bustype = PLATFORM_BUS_I2C;
		core_data->pdata = core_data->ops.platform_var_init(&param);
	}

	if ( core_data->pdata->i2c->ops.get_cfg ) {
		core_data->pdata->i2c->ops.get_cfg(core_data->pdata->i2c);
	}

	if ( core_data->pdata->i2c->ops.set_dev ) {
		err = core_data->pdata->i2c->ops.set_dev(core_data->pdata->i2c);
		if ( err ) goto ERR_INIT;
	}

	err = i2c_add_driver(&generic_ts_driver);
	if ( err ) goto ERR_INIT;

	return 0;

ERR_INIT:
ERR_VAR:
	core_var_exit(core_data);

	return err;
}

void __exit core_exit(void)
{
	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	core_var_exit(core_data);
	i2c_del_driver(&generic_ts_driver);
}

module_init(core_init);
module_exit(core_exit);

MODULE_AUTHOR("<@>");
MODULE_DESCRIPTION("generic touchscreen driver");
MODULE_LICENSE("GPL");


