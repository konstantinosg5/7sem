/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Gkavogiannis Konstantinos (el21820)
 * Megalios Giorgos (el21118)
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	/* ? */

	/* Check the last update time, so that you know if you should refresh */
	if (state->buf_timestamp != sensor->msr_data[BATT]->last_update) return 1;
	else return 0;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	debug("leaving\n");

	WARN_ON( !(sensor = state->sensor));

	uint32_t time , newData;
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	/* ? */
	debug("Spin lock in state update");
	spin_lock_irq(&sensor->lock);
	time = sensor->msr_data[BATT]->last_update;
	newData = sensor->msr_data[state->type]->values[0];
	spin_unlock_irq(&sensor->lock);
	debug("Spin unlock in state update");

	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	* Any new data available?
	*/
	/* ? */
	if (lunix_chrdev_state_needs_refresh(state)) {

	/*
	* Now we can take our time to format them,
	* holding only the private state semaphore
	*/

	/* ? */
		long cooked_value;
		state->buf_timestamp = time; //save new update time
		
		switch (state->type) {
			case BATT:
				cooked_value = lookup_voltage[newData];
				break;
			case TEMP:
				cooked_value = lookup_temperature[newData];
				break;
			case LIGHT:
				cooked_value = lookup_light[newData];
				break;
			default:
				return -EAGAIN; // fail
		}


		debug("New data: %ld\n", cooked_value);
		state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%ld.%ld\n", cooked_value/1000, cooked_value%1000);
		debug("New data came and specifically %d bytes\n", state->buf_lim);
		
	}
	else {
		debug("No new data\n");
		return -EAGAIN;
	}

	debug("Leaving state update\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	struct lunix_chrdev_state_struct *state;
	int ret, Minor, DeviseID, TypeOfMesurement;

	debug("\n======================================\nHello from the kernel!\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	Minor = iminor(inode);

	// Sensor and Type of Mesurement can be dirived from Minor number
	TypeOfMesurement = Minor % 8; //Type of measurement (0 Battery, 1 temperature, 2 Light)
	DeviseID = Minor / 8; 

	/* Allocate a new Lunix character device private state structure */
	/* ? */
	state = kzalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	// The GFP_KERNEL flag means that the process can go to sleep until we find memory

	if (!state) {
		ret = -ENOMEM;
		printk(KERN_ERR "Failed to allocate memory for Lunix driver state");
		goto out;
	}

	// If we passed check we can start loading Initializations
	state->sensor = &lunix_sensors[DeviseID]; 
	state->type = TypeOfMesurement;
	
	state->buf_lim = 0; 		// Length of state->buf_data
	state->buf_timestamp = 0; 

	sema_init(&state->lock, 1); //Initialize semaphore for state

	filp->private_data = state; 
	ret = 0;

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	struct lunix_chrdev_state_struct *state;
	state = filp->private_data;
	WARN_ON(!state);

	kfree(state);
	debug("Lunix Char Driven closed successfully!\n======================================\n");

	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

        /* Lock? */
	if(down_interruptible(&state->lock))
		return -ERESTARTSYS;
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {// Empty Buffer
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			up(&state->lock); // Unlock so some other process progress

			debug("Sleepy time\n");

			// Sleep queue
			if(wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
				return -ERESTARTSYS;
			
			// Lock again
			if(down_interruptible(&state->lock))
				return -ERESTARTSYS;

		}
	}

        debug("I am awake!");
	/* End of file */
	/* ? */
	if (*f_pos + cnt > (size_t) state->buf_lim) {// If more is asked than max, we just give max
		cnt = (size_t) state->buf_lim - *f_pos;
	}

	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
	if(copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)) {
		ret = -EFAULT;
		goto out;
	}

	/* Auto-rewind on EOF mode? */
	/* ? */
	*f_pos = *f_pos + cnt;
	ret = cnt;

	if(state->buf_lim == *f_pos) 
		*f_pos = 0;

out:
	/* Unlock? */
	up(&state->lock);
	return ret;
}


static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}


static struct file_operations lunix_chrdev_fops = 
{
	.owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};


int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("Initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	/* ? */
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	
	/* register_chrdev_region? */
	/* ? */
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "Lunix-TNG");
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	

	/* cdev_add? */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("Initialization completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
