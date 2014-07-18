#ifndef FSUTILS_H
#define FSUTILS_H


#include <linux/proc_fs.h>


// ****************************************************************************
// file system utilities 
// ****************************************************************************
struct struct_fsutils_cmd {
	char major_str;
	char minor_str;
	char *help;
};

struct struct_fsutils_var;
struct struct_fsutils_ops {
	int (*def_cmd_intpt)(struct struct_fsutils_cmd *cmd_list, int cmd_num, const char *cmd_str);
	int (*def_cmd_hndlr)(int cmd_ind);
};

struct struct_fsutils_var {
	struct proc_dir_entry *entry;

	int def_cmd_num;
	struct struct_fsutils_cmd *def_cmd_list;	// default commnad list
	int cus_cmd_num;
	struct struct_fsutils_cmd *cus_cmd_list;	// custom commnad list

	struct struct_fsutils_ops ops;
};

struct struct_fsutils_cmd_param {
	char lenhi;
	char lenlo;
	char crc;
	char *param;
};

struct struct_fsutils_param {
	/* parameters for fsutils */
	int cus_cmd_num;
	struct struct_fsutils_cmd *cus_cmd_list;
};

// ****************************************************************************
// Function prototypes
// ****************************************************************************
struct struct_fsutils_var* fsutils_var_init(struct struct_fsutils_param *param);
void fsutils_var_exit(struct struct_fsutils_var *fsutils);
#endif
