#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>



#include "core_i2c.h"
#include "core_usb.h"
#include "fsutils.h"


#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_DEBUG_FSUTILS
#define	PRINT_FSUTILS_MSG(msg...) printk(msg)
#else
#define	PRINT_FSUTILS_MSG(msg...)
#endif


extern struct struct_core_i2c_var *core_data;

enum enum_fsutils_cmds {
	FSUTILS_CMD_LIST,

	FSUTILS_CHIP_ID,
	FSUTILS_CHIP_RESET,

	FSUTILS_FW_VER,
	FSUTILS_FW_CHKSUM,
	FSUTILS_UPDATE_FW,

	FSUTILS_BIN_VER,
	FSUTILS_BIN_CHKSUM,
	FSUTILS_BIN_LOAD,

	FSUTILS_UPDATE_CHIP,
	FSUTILS_UPDATE_TOUCH,

	FSUTILS_UPDATE_DEBUG,
};

static
struct struct_fsutils_cmd def_cmd_list[] = {
	{
		.major_str = '?',
		.minor_str = 0,
		.help = "list all commands.",
	},
	{
		.major_str = 'i',
		.minor_str = 0,
		.help = "get touch ic chip id.",
	},
	{
		.major_str = 'r',
		.minor_str = 0,
		.help = "do touch ic hardware reset.",
	},
	{
		.major_str = 'V',
		.minor_str = 0,
		.help = "get touch ic firmware version.",
	},
	{
		.major_str = 'C',
		.minor_str = 0,
		.help = "get touch ic firmware checksum.",
	},
	{
		.major_str = 'u',
		.minor_str = 'f',
		.help = "do flash touch ic firmware.",
	},
	{
		.major_str = 'v',
		.minor_str = 0,
		.help = "get touch ic binary version.",
	},
	{
		.major_str = 'c',
		.minor_str = 0,
		.help = "get touch ic binary checksum.",
	},
	{
		.major_str = 'l',
		.minor_str = 0,
		.help = "do load touch ic binary from ext-sd.",
	},
	{
		.major_str = 'u',
		.minor_str = 'c',
		.help = "apply chip binary config.",
	},
	{
		.major_str = 'u',
		.minor_str = 't',
		.help = "apply touch binary config.",
	},
	{
		.major_str = 'd',
		.minor_str = 'b',
		.help = "debug ...",
	},
};

static int fsutils_open(struct inode *inode, struct file *file);
static int fsutils_close(struct inode *inode, struct file *file);
static ssize_t fsutils_write(struct file *file, const char __user *buffer, size_t count, loff_t *offset);
static ssize_t fsutils_read(struct file *file, char __user *buf, size_t count, loff_t *offset);


static
struct file_operations fsutils_fops = {
	.owner = THIS_MODULE,
	.open = fsutils_open,
	.release = fsutils_close,
	.write = fsutils_write,
	.read = fsutils_read,
};

static
int fsutils_cmd_interpreter(struct struct_fsutils_cmd *cmd_list, int cmd_num, const char *cmd_str)
{
	int iter = 0;
	int major = 0, minor = 0;

	// search cmd
	PRINT_FSUTILS_MSG("%s(): cmd_str[0]=%c, cmd_str[1]=%c\n", __FUNCTION__, cmd_str[0], cmd_str[1]);
	for ( iter = 0; iter < cmd_num; iter++ ) {		
		major = 0; minor = 0;
		if ( cmd_list[iter].major_str == cmd_str[0] ) 
			major = 1;
		if ( cmd_list[iter].minor_str == cmd_str[1] || cmd_list[iter].minor_str == 0 )
			minor = 1;

		if ( major && minor ) {
			PRINT_FSUTILS_MSG("%s(): major_str=%c, minor_str=%c\n", __FUNCTION__, cmd_list[iter].major_str, cmd_list[iter].minor_str);
			return iter;
		}
	}

	return -1;
}

static 
int fsutils_load_bin(char *buffer, size_t count)
{
	int ret = -1;
	struct file* filp = NULL;
	loff_t offset = 0;

	mm_segment_t old_fs = get_fs();
	set_fs(get_ds());
	//fd = sys_open("/etc/firmware/ct36x_ts.bin", O_RDONLY, 0);
	filp = filp_open("/mnt/external_sd/firmware/ct36x_ts.bin", O_RDONLY, 0);
	if ( !IS_ERR(filp) ) {
		ret = vfs_read(filp, buffer, count, &offset);
		filp_close(filp, NULL);
	}
	set_fs(old_fs);

	return ret;
}

static
int fsutils_cmd_handler(int cmd_ind)
{
	int iter;
	int rslt;
	char *buf = (char*)core_data->cdata->pts_data;

	PRINT_FSUTILS_MSG(">>>>> %s() called <<<<< \n", __FUNCTION__);

	PRINT_FSUTILS_MSG("%s(): cmd_ind: %d\n", __FUNCTION__, cmd_ind);
	if (cmd_ind >= 0 && cmd_ind < core_data->fdata->def_cmd_num )
		PRINT_FSUTILS_MSG("%s(): %s\n", __FUNCTION__, core_data->fdata->def_cmd_list[cmd_ind].help);

	switch ( cmd_ind ) {
		case FSUTILS_CMD_LIST:	// help
			printk("****************************************************************************\n");
			for ( iter = 0; iter < core_data->fdata->def_cmd_num; iter++ )
				printk("** %s\n", core_data->fdata->def_cmd_list[iter].help);
			printk("****************************************************************************\n");
			break;

		case FSUTILS_CHIP_ID:	// chip id
			if ( core_data->cdata->ops.get_vendor)
				rslt = core_data->cdata->ops.get_vendor(core_data->pdata->i2c->client, buf);
				printk("Chip ID: 0x%x\n", rslt);
			break;

		case FSUTILS_CHIP_RESET:	// chip reset
			if ( core_data->pdata->i2c->ops.hw_reset )
				core_data->pdata->i2c->ops.hw_reset(core_data->pdata->i2c);
			break;

		case FSUTILS_FW_VER:	// get version
			break;

		case FSUTILS_FW_CHKSUM:	// get fw checksum
			if ( core_data->cdata->ops.get_fw_chksum )
			rslt = core_data->cdata->ops.get_fw_chksum(core_data->pdata->i2c->client, buf);
			printk("%s(): Fw checksum: 0x%x\n", __FUNCTION__, rslt);
			break;

		case FSUTILS_UPDATE_FW:	// flash firmware
			if ( core_data->cdata->ops.update_fw )
			core_data->cdata->ops.update_fw(core_data->pdata->i2c->client, buf);
			break;

		case FSUTILS_BIN_VER:	// get binary version
			break;

		case FSUTILS_BIN_CHKSUM:	// get binary checksum
			if ( core_data->cdata->ops.get_bin_chksum )
			rslt = core_data->cdata->ops.get_bin_chksum(buf, core_data->cdata->bin_data);
			printk("%s(): bin checksum: 0x%x\n", __FUNCTION__, rslt);
			break;

		case FSUTILS_BIN_LOAD:	// load binary from ext-sd
			//if ( core_data->fdata->ops.load_bin )
			//rslt = core_data->fdata->ops.load_bin(core_data->cdata->bin_data, sizeof(core_data->cdata->bin_data));
			rslt = fsutils_load_bin(core_data->cdata->bin_data, core_data->cdata->bin_sz);
			//for ( iter = 0; iter < 16; iter++ ) {
			//	PRINT_FSUTILS_MSG("%x, ", core_data->cdata->bin_data[iter]);
			//}
			//PRINT_FSUTILS_MSG("\n");
			printk("%s(): load firmware binary %s\n", __FUNCTION__, rslt == -1 ? "failed" : "success");
			break;

		case FSUTILS_UPDATE_CHIP: {	// apply binary config to chip
			struct struct_chip_param *param = core_data->cdata->bin_data;

			PRINT_FSUTILS_MSG("%s(): param->pts_num=%d \n", __FUNCTION__, param->pts_num);
			PRINT_FSUTILS_MSG("%s(): param->chip=%d \n", __FUNCTION__, param->chip);
			PRINT_FSUTILS_MSG("%s(): param->xy_swap=%d \n", __FUNCTION__, param->xy_swap);
			PRINT_FSUTILS_MSG("%s(): param->x_max=%d \n", __FUNCTION__, param->x_max);
			PRINT_FSUTILS_MSG("%s(): param->y_max=%d \n", __FUNCTION__, param->y_max);
			PRINT_FSUTILS_MSG("%s(): param->chn_x_num=%d \n", __FUNCTION__, param->chn_x_num);
			PRINT_FSUTILS_MSG("%s(): param->chn_y_num=%d \n", __FUNCTION__, param->chn_y_num);
			PRINT_FSUTILS_MSG("%s(): param->chn_res=%d \n", __FUNCTION__, param->chn_res);

			if ( core_data->ops.chip_var_exit && core_data->ops.chip_var_init ) {
				core_data->ops.chip_var_exit(core_data->cdata);
				core_data->cdata = core_data->ops.chip_var_init(param);
			}
			}
			break;

		case FSUTILS_UPDATE_TOUCH: {	// apply binary config to touch
			struct struct_touch_param *param = core_data->cdata->bin_data;

			PRINT_FSUTILS_MSG("%s(): param->valid_max_pts_num=%d \n", __FUNCTION__, param->valid_max_pts_num);
			PRINT_FSUTILS_MSG("%s(): param->x_max=%d \n", __FUNCTION__, param->x_max);
			PRINT_FSUTILS_MSG("%s(): param->y_max=%d \n", __FUNCTION__, param->y_max);
			PRINT_FSUTILS_MSG("%s(): param->x_rvs=%d \n", __FUNCTION__, param->x_rvs);
			PRINT_FSUTILS_MSG("%s(): param->y_rvs=%d \n", __FUNCTION__, param->y_rvs);
			PRINT_FSUTILS_MSG("%s(): param->xy_swp=%d \n", __FUNCTION__, param->xy_swp);

		#if 0
			if ( core_data->ops.touch_var_exit && core_data->ops.touch_var_init ) {
				core_data->ops.touch_var_exit(core_data->tdata);
				core_data->tdata = core_data->ops.touch_var_init(param);
			}
		#else
			if ( core_data->tdata ) {
				core_data->tdata->valid_max_pts_num = param->valid_max_pts_num;
				core_data->tdata->x_rvs = param->x_rvs;
				core_data->tdata->y_rvs = param->y_rvs;
				core_data->tdata->xy_swp = param->xy_swp;
			}
		#endif
			}
			break;

		case FSUTILS_UPDATE_DEBUG: {	// debug mode
			struct struct_fsutils_cmd_param *param = NULL;
			param = core_data->cdata->bin_data;
			param->param = core_data->cdata->bin_data + (sizeof(struct struct_fsutils_cmd_param) - sizeof(param->param));
			
			printk("param->lenhi:%x, param->lenlo:%x \n", param->lenhi, param->lenlo);
			printk("param->crc:%x \n", param->crc);

			//for ( iter = 0; iter < (param->lenhi<<8) | param->lenlo; iter++ )
			//{
			//	if ( iter % 0xF ) printk("\n");
			//	printk("param->param[%x]:%x ", iter, param->param[iter]);
			//	
			//}
			}
			break;

		default:
			printk("%s(): command is not supported.\n", __FUNCTION__);
			break;
	}
	
	return 0;
}

static
int fsutils_open(struct inode *inode, struct file *file)
{
	PRINT_FSUTILS_MSG(">>>>> %s() called <<<<< \n", __FUNCTION__);
	
	return 0;
}

static
int fsutils_close(struct inode *inode, struct file *file)
{
	PRINT_FSUTILS_MSG(">>>>> %s() called <<<<< \n", __FUNCTION__);
	
	return 0;
}

static
ssize_t fsutils_write(struct file *file, const char __user *buffer, size_t count, loff_t *offset)
{
	int cmd_ind;

	PRINT_FSUTILS_MSG(">>>>> %s() called <<<<< \n", __FUNCTION__);

	/* execute cmd */
	if ( core_data->state == CORE_STATE_NORMAL ) {
		/* interpret default cmds */
		if ( core_data->fdata->ops.def_cmd_intpt ) {
			cmd_ind = core_data->fdata->ops.def_cmd_intpt(core_data->fdata->def_cmd_list, core_data->fdata->def_cmd_num, buffer);	
		}

		/* execute default cmds */
		if ( core_data->fdata->ops.def_cmd_hndlr ) {
			core_data->fdata->ops.def_cmd_hndlr(cmd_ind);
		}
	}

	return count;
}

static
ssize_t fsutils_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	PRINT_FSUTILS_MSG(">>>>> %s() called <<<<< \n", __FUNCTION__);

	if ( core_data->state == CORE_STATE_NORMAL ) {
	
	}

	return 0;//count;
}

struct struct_fsutils_var* fsutils_var_init(struct struct_fsutils_param *param)
{
	struct struct_fsutils_var *fsutils = NULL;

	fsutils = kzalloc(sizeof(struct struct_fsutils_var), GFP_KERNEL);
	if ( fsutils ) {
		/* Create Proc Entry File */
		fsutils->entry = create_proc_entry(DRIVER_NAME, 0666/*S_IFREG | S_IRUGO | S_IWUSR*/, NULL);
		if ( fsutils->entry == NULL ) {
			printk("Failed creating proc dir entry file.\n");
			goto ERR_CREATE_ENTRY;
		} else {
			fsutils->entry->proc_fops = &fsutils_fops;
		}

		/* default command list */
		fsutils->def_cmd_num = sizeof(def_cmd_list)/sizeof(struct struct_fsutils_cmd);
		fsutils->def_cmd_list = def_cmd_list;

		fsutils->ops.def_cmd_intpt = fsutils_cmd_interpreter;
		fsutils->ops.def_cmd_hndlr = fsutils_cmd_handler;
		//fsutils->ops.load_bin = fsutils_load_bin;
	}

	return fsutils;

ERR_CREATE_ENTRY:
	if ( fsutils )
		kfree(fsutils);

	return NULL;
}

void fsutils_var_exit(struct struct_fsutils_var *fsutils)
{
	if ( fsutils ) {
		//if ( fsutils->cmd_str )
		//	kfree(fsutils->cmd_str);
		//if ( fsutils->cmd_ind )
		//	kfree(fsutils->cmd_ind);
		if ( fsutils->entry )
			remove_proc_entry(DRIVER_NAME, NULL);

		kfree(fsutils);
	}
}
