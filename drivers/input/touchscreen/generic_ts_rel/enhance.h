#ifndef ENHANCE_H
#define ENHANCE_H


/* Enhance performance Config */
//#define ENHANCE_EN				1
/* Enhance Preprocess Config */
#define ENHANCE_PREPROC_EN				0
#define ENHANCE_PREPROC_AVG_DNSTY_EN	1
#define ENHANCE_PREPROC_AVG_DNSTY_DELTA	8

/* Enhance Tracking Config */
#define ENHANCE_TRACK_EN				1
#define ENHANCE_TRACK_DIST_EN			1
#define ENHANCE_TRACK_DIST_THRSHLD_FIRST		128//64
#define ENHANCE_TRACK_DIST_THRSHLD_TRACK		32//8
#define ENHANCE_TRACK_VALID_THRSHLD_MIN	250
#define ENHANCE_TRACK_VALID_THRSHLD_MAX	5000
#define ENHANCE_TRACK_PTS_STB_CNT_DISCHARGING		0//3//5
#define ENHANCE_TRACK_PTS_STB_CNT_CHARGING		3//1//3//5//7

/* Enhance Postprocess Config */
#define ENHANCE_POSTPROC_EN				0

/* Enhance Curve-fitting Config */
#define ENHANCE_CURVE_EN				1
#define ENHANCE_CURVE_FILTER_EN			1
#define ENHANCE_CURVE_FILTER_PTS_MAX	10

/* Enhance Scaling Config */
#define ENHANCE_SCALE_EN				1


// ****************************************************************************
// pre-processing
// ****************************************************************************
struct struct_preproc_point_data {
	int id;
	int x;
	int y;
};

struct struct_preproc_var;
struct struct_preproc_ops {
	void (*obtain_pts)(struct struct_preproc_var *preproc, void *pts_data, int (*cpy)(int *x, int *y, char *id, void *pts_data), int pt_sz, int num_pts);
	void (*update_pts)(void *pts_data, struct struct_preproc_var *preproc, int (*upd)(int *x, int *y, char *id, void *pts_data), int pt_sz);

	void (*avg_pts_dnsty)(struct struct_preproc_var *preproc);
};

struct struct_preproc_var {
	/* global data for preprocessing */
	int valid_max_pts_num;                   // max point number support.
	int valid_cur_pts_num;                   // current point number.
	struct struct_preproc_point_data *pts;  // current points been tracked.

	/* average pts density */
	char avg_dnsty_en;
	char chn_res;                             // channel resolution
	char delta;

	struct struct_preproc_ops ops;
};


// ****************************************************************************
// tracking 
// ****************************************************************************
struct struct_track_point_data {
	int id;
	int x;
	int y;
};

struct struct_track_dist_data {
	int dist;
	int path;
	int prv_ind;
	int cur_ind;
};

struct struct_track_thrshld_data {
	int min_track_dist;
	int max_track_dist;
};

struct struct_track_var;
struct struct_track_ops {
	void (*obtain_pts)(struct struct_track_var *track, void *pts_data, int (*cpy)(int *x, int *y, char *id, void *pts_data), int pt_sz, int num_pts);
	void (*update_pts)(void *pts_data, struct struct_track_var *track, int (*upd)(int *x, int *y, char *id, void *pts_data), int pt_sz);

	void (*backup_pts)(struct struct_track_var *track);
	void (*do_track)(struct struct_track_var *track);
	
};

struct struct_track_var {
	/* global data for preprocessing */
	int valid_max_pts_num;                   // max point number support.
	int valid_prv_pts_num;                   // previous point number detected.
	int valid_cur_pts_num;                   // current point number detected.

	int dist_en;
	int id_map;                               // record used/unused IDs of point.
	int row_num;                              // point number of row of tracking map
	int col_num;                              // point number of col of tracking map
	struct struct_track_point_data *row_pts;  // points of row (could be prv_pts,or cur_pts) of tracking map
	struct struct_track_point_data *col_pts;  // points of col (could be prv_pts,or cur_pts) of tracking map
	struct struct_track_point_data *prv_pts;  // previous points been tracked.
	struct struct_track_point_data *cur_pts;  // current points been tracked.
	struct struct_track_dist_data *dist_map;     // distance & path map between previous points and current points & minimum path map of all valid distance.

	int track_min_thrshld_first;             // first mininum tracking threshold
	int track_min_thrshld_track;             // normal mininum tracking threshold
	struct struct_track_thrshld_data track_thrshld_valid;	// tracking validation threshold config
	struct struct_track_thrshld_data *track_thrshld_sel;	// dynamic tracking threshold config selected for each id
	int *dist_pts;                             // distance map of pts located by id

	//int stb_fac;                              // stable count factor
	int stb_cnt;                              // stable count
	int chn_res;                             // channel resolution
	int *grid_ind_x;
	int *grid_ind_y;
	int *grid_ind_x_cnt;
	int *grid_ind_y_cnt;

	struct struct_track_ops ops;
};


// ****************************************************************************
// post-processing
// ****************************************************************************
struct struct_postproc_point_data {
	int id;
	int x;
	int y;
};

struct struct_postproc_var;
struct struct_postproc_ops {
	void (*obtain_pts)(struct struct_postproc_var *postproc, void *pts_data, int (*cpy)(int *x, int *y, char *id, void *pts_data), int pt_sz, int num_pts);
	void (*update_pts)(void *pts_data, struct struct_postproc_var *postproc, int (*upd)(int *x, int *y, char *id, void *pts_data), int pt_sz);

	void (*do_postproc)(struct struct_postproc_var *postproc);
};

struct struct_postproc_var {
	/* global data for postprocessing */
	int valid_max_pts_num;                   // max point number support.
	int valid_cur_pts_num;                   // current point number.
	struct struct_postproc_point_data *pts;  // current points been tracked.

	struct struct_postproc_ops ops;
};


// ****************************************************************************
// curve fitting
// ****************************************************************************
enum enum_curve_fltr_type {
	CURVE_FLTR_TYPE_IIR,
	CURVE_FLTR_TYPE_FIR,
};

struct struct_curve_point_data {
	char id;
	int x;
	int y;
};

struct struct_fltr_data {
	int type;			// filter type
	int num;           // number of weight
	int min_thrshld;
	int max_thrshld;
	int *coef;       // filter coef
};

struct struct_curve_var;
struct struct_curve_ops {
	void (*obtain_pts)(struct struct_curve_var *curve, void *pts_data, int (*cpy)(int *x, int *y, char *id, void *pts_data), int pt_sz, int num_pts);
	void (*update_pts)(void *pts_data, struct struct_curve_var *curve, int (*upd)(int *x, int *y, char *id, void *pts_data), int pt_sz);

	void (*do_fltr)(struct struct_curve_var* curve, int *dist_pts);
};

struct struct_curve_var {
	/* global data of curve fitting */
	int valid_max_pts_num;
	int valid_cur_pts_num;

	struct struct_curve_point_data *pts;

	/* filtering */
	int fltr_en;
	int fltr_lvl_num;            // max filter level
	int fltr_pts_num;            // max pts buffer
	int *fltr_coef_acc;               // number of points used in iir calculation
	int *fltr_pts_x;
	int *fltr_pts_y;
	struct struct_fltr_data *fltr;

	struct struct_curve_ops ops;
};


// ****************************************************************************
// scaling
// ****************************************************************************
struct struct_scale_point_data {
	char id;
	int x;
	int y;
};

struct struct_scale_var;
struct struct_scale_ops {
	void (*obtain_pts)(struct struct_scale_var *scale, void *pts_data, int (*cpy)(int *x, int *y, char *id, void *pts_data), int pt_sz, int num_pts);
	void (*update_pts)(void *pts_data, struct struct_scale_var *scale, int (*upd)(int *x, int *y, char *id, void *pts_data), int pt_sz);

	void (*do_scale)(struct struct_scale_var* scale);
};

struct struct_scale_var {
	/* global data of scaling */
	int valid_max_pts_num;                   // max point number support.
	int valid_cur_pts_num;                   // current point number detected.
	//int pt_sz;
	struct struct_scale_point_data *pts;

	/* */
	int scale_en;
	int real_x_max;
	int real_y_max;
	int scal_x_max;
	int scal_y_max;

	struct struct_scale_ops ops;
};

// ****************************************************************************
// enhance
// ****************************************************************************
struct struct_enhance_param {
	/* parameters for enhance global */

	/* parameters for pre-processing */
	int preproc_valid_max_pts_num;
	int avg_dnsty_en;
	int avg_dnsty_chn_res;
	int avg_dnsty_delta;

	/* parameters for tracking */
	int track_valid_max_pts_num;
	int dist_en;
	int track_min_thrshld_first;
	int track_min_thrshld_track;
	int track_valid_thrshld_min;
	int track_valid_thrshld_max;
	//int track_stb_fac;
	int track_stb_cnt;
	int track_chn_res;

	/* parameters for post-processing */
	int postproc_valid_max_pts_num;
	//char pwrns_pts_corr_en;
	//char pwrns_pts_corr_stb_cnt;
	//char pwrns_pts_corr_chn_res;

	/* parameters for curve fitting (iir) */
	int curve_valid_max_pts_num;
	int fltr_en;
	int fltr_lvl_num;
	int fltr_pts_num;
	struct struct_fltr_data *fltr;

	/* parameters for scaling */
	int scale_valid_max_pts_num;
	int scale_en;
	int scale_chn_x_num;
	int scale_chn_y_num;
	int scale_chn_res;
	int scale_x_max;
	int scale_y_max;
};


struct struct_enhance_ops {
	struct struct_preproc_var* (*preproc_var_init)(struct struct_enhance_param *param);
	void (*preproc_var_exit)(struct struct_preproc_var *preproc);

	struct struct_track_var* (*track_var_init)(struct struct_enhance_param *param);
	void (*track_var_exit)(struct struct_track_var *track);

	struct struct_postproc_var* (*postproc_var_init)(struct struct_enhance_param *param);
	void (*postproc_var_exit)(struct struct_postproc_var *postproc);

	struct struct_scale_var* (*scale_var_init)(struct struct_enhance_param *param);
	void (*scale_var_exit)(struct struct_scale_var *scale);

	struct struct_curve_var* (*curve_var_init)(struct struct_enhance_param *param);
	void (*curve_var_exit)(struct struct_curve_var *curve);
};

struct struct_enhance_var {
	// preprocessing
	struct struct_preproc_var *preproc;
	// Tracking
	struct struct_track_var *track;
	// postprocessing
	struct struct_postproc_var *postproc;
	// Scaling
	struct struct_scale_var *scale;
	// Curve fitting
	struct struct_curve_var *curve;

	struct struct_enhance_ops ops;
};

// ****************************************************************************
// Globel or static variables
// ****************************************************************************


// ****************************************************************************
// Function prototypes
// ****************************************************************************
struct struct_enhance_var* enhance_var_init(struct struct_enhance_param *param);
void enhance_var_exit(struct struct_enhance_var *enhance_data);
#endif
