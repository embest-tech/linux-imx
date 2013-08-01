#ifndef TOUCH_H
#define TOUCH_H

#define TOUCH_X_REVERSE		0
#define TOUCH_Y_REVERSE		0


// ****************************************************************************
// touch 
// ****************************************************************************
struct struct_touch_point_data {
	char id;
	int x;
	int y;
};

struct struct_touch_var;
struct struct_touch_ops {
	//
	void (*obtain_pts)(struct struct_touch_var *touch, void *pts_data, int (*cpy)(int *x, int *y, char *id, void *pts_data), int pt_sz);
#if 0
	void (*update_pts)(struct struct_touch_var *touch, void *pts_data, int (*upd)(int *x, int *y, char *id, void *pts_data), int pt_sz);
#endif

	//
	int (*cpy_valid_pt)(int *x, int *y, char *id, void *pts_data);
	int (*upd_valid_pt)(int *x, int *y, char *id, void *pts_data);


	//
	void (*report_pts)(struct struct_touch_var *touch);
};

struct struct_touch_var {
	int valid_max_pts_num;                   // max point number support.
	int valid_cur_pts_num;                   // current point number.
	int pt_sz;                               // point data size in byte.

	int x_max;
	int y_max;
	int x_rvs;
	int y_rvs;
	int xy_swp;

	struct struct_touch_point_data *pts;      // current points been tracked.

	/* input devices */
	struct input_dev		*input;

	int sync;
	int press;
	int release;

	struct struct_touch_ops ops;
};

struct struct_touch_param {
	/* parameters for touch */
	char valid_max_pts_num;
	int x_max;
	int y_max;
	int x_rvs;
	int y_rvs;
	int xy_swp;
};

// ****************************************************************************
// Function prototypes
// ****************************************************************************
struct struct_touch_var* touch_var_init(struct struct_touch_param *param);
void touch_var_exit(struct struct_touch_var *touch);
#endif
