#include <linux/kernel.h>
#include <linux/slab.h>
// for android 4.x only
#include <linux/input/mt.h>

#include "core_i2c.h"
#include "core_usb.h"
#include "touch.h"


#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_DEBUG_TOUCH
#define	PRINT_TOUCH_MSG(msg...) printk(msg)
#else
#define	PRINT_TOUCH_MSG(msg...)
#endif

// ****************************************************************************
// preprocessing 
// ****************************************************************************
void touch_obtain_pts(struct struct_touch_var *touch, void *pts_data, int (*cpy)(int *x, int *y, char *id, void *pts_data), int pt_sz)
{
	int iter;

	PRINT_TOUCH_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	touch->valid_cur_pts_num = 0;
	for ( iter = 0; iter < touch->valid_max_pts_num; iter++ ) {
		if ( cpy ) {
			if ( cpy(
				&touch->pts[touch->valid_cur_pts_num].x, 
				&touch->pts[touch->valid_cur_pts_num].y, 
				&touch->pts[touch->valid_cur_pts_num].id, 
				(char*)pts_data+(pt_sz*iter)) ) {
					touch->valid_cur_pts_num++;
			}
		}
	}
}

#if 0
void touch_update_pts(struct struct_touch_var *touch, void *pts_data, int (*upd)(int *x, int *y, char *id, void *pts_data), int pt_sz)
{
	int iter;

	PRINT_TOUCH_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);
}
#endif

int touch_cpy_valid_pt(int *x, int *y, char *id, void *pt_data)
{
	int ret = 0;

	struct struct_touch_point_data *pt = (struct struct_touch_point_data*)pt_data;
	
	PRINT_TOUCH_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	*x = pt->x;
	*y = pt->y;
	*id = pt->id;
	
	return ret;
}

int touch_upd_valid_pt(int *x, int *y, char *id, void *pt_data)
{
	int ret = 0;

	struct struct_touch_point_data *pt = (struct struct_touch_point_data*)pt_data;
	
	PRINT_TOUCH_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	pt->x = *x;
	pt->y = *y;
	pt->id = *id;
	
	return ret;
}

void touch_report_pts(struct struct_touch_var *touch)
{
	int iter;
	int temp;

	/* report points of down */
	touch->sync = 0; touch->press = 0;
	for ( iter = 0; iter < touch->valid_cur_pts_num; iter++ ) {	
		if ( touch->pts[iter].x > touch->x_max || touch->pts[iter].y > touch->y_max )
			continue;

		if ( touch->x_rvs )
			touch->pts[iter].x = touch->x_max - touch->pts[iter].x;
		if ( touch->y_rvs )
			touch->pts[iter].y = touch->y_max - touch->pts[iter].y;
		if ( touch->xy_swp ) {
			temp = touch->pts[iter].x;
			touch->pts[iter].x = touch->pts[iter].y;
			touch->pts[iter].y = temp;
		}

                PRINT_TOUCH_MSG("ID:    %d\n", touch->pts[iter].id);
                PRINT_TOUCH_MSG("X:     %d\n", touch->pts[iter].x);
                PRINT_TOUCH_MSG("Y:     %d\n", touch->pts[iter].y);

#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_SINGLE_TOUCH
                input_report_abs(touch->input, ABS_X, touch->pts[iter].x);
                input_report_abs(touch->input, ABS_Y, touch->pts[iter].y);
                input_event(touch->input, EV_KEY, BTN_TOUCH, 1);
                input_report_abs(touch->input, ABS_PRESSURE, 1);		
#else
	#if 1	// android 4.x
		input_mt_slot(touch->input, touch->pts[iter].id);
		input_mt_report_slot_state(touch->input, MT_TOOL_FINGER, true);
		input_report_abs(touch->input, ABS_MT_POSITION_X, touch->pts[iter].x);
		input_report_abs(touch->input, ABS_MT_POSITION_Y, touch->pts[iter].y);
		input_report_abs(touch->input, ABS_MT_TOUCH_MAJOR, 30);
		input_report_abs(touch->input, ABS_MT_WIDTH_MAJOR, 128);
	#else	// android 2.x / others
		input_report_abs(touch->input, ABS_MT_TRACKING_ID, touch->pts[iter].id);
		input_report_abs(touch->input, ABS_MT_POSITION_X,  touch->pts[iter].x);
		input_report_abs(touch->input, ABS_MT_POSITION_Y,  touch->pts[iter].y);
		input_report_abs(touch->input, ABS_MT_TOUCH_MAJOR, 30);
		input_report_abs(touch->input, ABS_MT_WIDTH_MAJOR, 128);
			
		input_mt_sync(ts->input);
	#endif
#endif	
		touch->sync = 1;
		touch->press |= (0x01 << touch->pts[iter].id);
	}

	/* report points of up */
	//touch->sync = 0;
	touch->release &= touch->release ^ touch->press;
	for ( iter = 0; iter < touch->valid_max_pts_num; iter++ ) {
		if ( touch->release & (0x01<<iter) ) {
#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_SINGLE_TOUCH
                input_event(touch->input, EV_KEY, BTN_TOUCH, 0);
                input_report_abs(touch->input, ABS_PRESSURE, 0);
#else
	#if 1	// android 4.x
			input_mt_slot(touch->input, iter);
			input_mt_report_slot_state(touch->input, MT_TOOL_FINGER, false);
	#else	// android 2.x / others
			input_report_abs(core_data->input, ABS_MT_TRACKING_ID, iter);
			input_report_abs(core_data->input, ABS_MT_POSITION_X,  touch->pts[iter].x);
			input_report_abs(core_data->input, ABS_MT_POSITION_Y,  touch->pts[iter].y);
			input_report_abs(core_data->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(core_data->input, ABS_MT_WIDTH_MAJOR, 0);
	
			input_mt_sync(core_data->input);
	#endif
#endif
			touch->sync = 1;
		}
	}
	touch->release = touch->press;
	
	if ( touch->sync ) input_sync(touch->input);
}

struct struct_touch_var* touch_var_init(struct struct_touch_param *param)
{
	struct struct_touch_var *touch = NULL;

	touch = kzalloc(sizeof(struct struct_touch_var), GFP_KERNEL);
	if ( touch ) {
		PRINT_TOUCH_MSG("%s: kzalloc OK for variable touch.\n", __FUNCTION__);
		touch->valid_max_pts_num = param->valid_max_pts_num;
		touch->pt_sz = sizeof(struct struct_touch_point_data);
		touch->valid_cur_pts_num = 0;
		touch->x_max = param->x_max;
		touch->y_max = param->y_max;
		touch->x_rvs = param->x_rvs;
		touch->y_rvs = param->y_rvs;
		touch->xy_swp = param->xy_swp;

		touch->pts = kzalloc(touch->pt_sz*touch->valid_max_pts_num, GFP_KERNEL);

		touch->ops.obtain_pts = touch_obtain_pts;
	#if 0
		touch->ops.update_pts = NULL;
	#endif
		touch->ops.cpy_valid_pt = touch_cpy_valid_pt;
		touch->ops.upd_valid_pt = touch_upd_valid_pt;
		touch->ops.report_pts = touch_report_pts;

		/* allocate input device */
		touch->input = input_allocate_device();
		if ( !touch->input ) {
			printk("Unable to allocate input device for device %s.\n", DRIVER_NAME);
			goto ERR_INPUT_ALLOC;
		}
		
		/* config input device */
		//__set_bit(EV_SYN, touch->input->evbit);
		__set_bit(EV_KEY, touch->input->evbit);
		__set_bit(EV_ABS, touch->input->evbit);

#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_SINGLE_TOUCH
        	__set_bit(BTN_TOUCH, touch->input->keybit);
        	input_set_abs_params(touch->input, ABS_X, 0, 1024, 0, 0);
        	input_set_abs_params(touch->input, ABS_Y, 0, 768, 0, 0);
        	input_set_abs_params(touch->input, ABS_PRESSURE, 0, 1, 0, 0);		
#else		
		// For android 4.x only
		__set_bit(INPUT_PROP_DIRECT, touch->input->propbit);
		
		// For android 4.x only
		input_mt_init_slots(touch->input, touch->valid_max_pts_num);
		input_set_abs_params(touch->input, ABS_MT_POSITION_X, 0, param->x_max, 0, 0);
		input_set_abs_params(touch->input, ABS_MT_POSITION_Y, 0, param->y_max, 0, 0);
		input_set_abs_params(touch->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(touch->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
#endif		
		touch->input->name = DRIVER_NAME;
		touch->input->id.bustype = BUS_I2C;
		
		/* register input device */
		if ( input_register_device(touch->input) ) {
			printk("Unable to register input device for device %s.\n", DRIVER_NAME);
			goto ERR_INPUT_REGIS;
		}
	}

	return touch;

ERR_INPUT_REGIS:
	input_free_device(touch->input);
ERR_INPUT_ALLOC:
	if ( touch->pts )
		kfree(touch->pts);
	if ( touch )
		kfree(touch);
	return NULL;
}

void touch_var_exit(struct struct_touch_var *touch)
{
	if ( touch ) {
		if ( touch->input ) {
			input_unregister_device(touch->input);
			input_free_device(touch->input);
		}
		if ( touch->pts )
			kfree(touch->pts);

		kfree(touch);
	}
}
