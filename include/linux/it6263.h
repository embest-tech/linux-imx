/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _IT6263_H_
#define _IT6263_H_

#ifdef __cplusplus
extern "C" {

#endif				/*  */
	struct it6263_platform_data {
		struct i2c_client *hdmi_client;
		struct i2c_client *LVDS_client;
	};
	struct it6263_data {
		struct task_struct *it6263_timer_task;
		int dev_inited;
		int irq;
		struct mutex lock;
		wait_queue_head_t it6263_wq;
	};

#ifdef __cplusplus
}
#endif				/*  */
#endif				/*  */
