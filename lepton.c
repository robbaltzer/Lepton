/*
 * Lepton Device Driver
 *
 * Copyright (c) 2013, Rob Baltzer All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * 3. The name of Rob Baltzer may not be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Rob Baltzer ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include <linux/moduleparam.h>
#include <linux/slab.h>         /* kmalloc() */
#include <linux/types.h>        /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <asm/system.h>         /* cli(), *_flags */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include "lepton.h"

#define MS_TO_NS(x) (x * 1E6L)

#define SPI_BUS 1
#define SPI_BUS_CS1 1
#define SPI_BUS_SPEED 15 * 1000000

#define VOSPI_PKT_SZ	(164)
#define LINES_OF_VIDEO	(60)
#define TELEMETRY_LINES	(3)
#define TOTAL_LINES		(LINES_OF_VIDEO + TELEMETRY_LINES)

#define MAX_RETRIES 10
#define SPI_MAX_XFER 12*1024

const char this_driver_name[]="lepton";

struct lepton_dev {
	struct semaphore spi_sem;
	struct semaphore fop_sem;
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct spi_device *spi_device;

	int num_transfers;	
	int transfer_size;
	bool loopback_mode;
	bool quiet;

	u8* read_buff;
	u8 packets;
};

struct lepton_dev lepton_dev;

static u8 * spi_tx_buf = NULL;
static u8 * spi_rx_buf = NULL;

static int lepton_transfer(struct spi_device *spi, int num_bytes)
{
	struct spi_message	msg;
	struct spi_transfer	xfer;
	int			ret;
	int 		i;
	int 		packets = 0, ret_val = 0, retry_ctr = 0, num_packets = 0, status = 0; 


	num_packets = num_bytes / VOSPI_PKT_SZ;
	while (retry_ctr < MAX_RETRIES && packets < num_packets) {
		if (lepton_dev.loopback_mode) {
			for (i = 0 ; i < VOSPI_PKT_SZ; i++) {
				spi_tx_buf[i] = i;
			}
		}
	
		/* Initialize the SPI message and transfer data structures */
		spi_message_init(&msg);
		memset(&xfer, 0, sizeof(xfer));
		xfer.tx_buf = spi_tx_buf;
		xfer.rx_buf = spi_rx_buf;
		xfer.len = VOSPI_PKT_SZ;
	
		/* Add our only transfer to the message */
		spi_message_add_tail(&xfer, &msg);
	
		/* Send the message and wait for completion */
		ret = spi_sync(spi, &msg);

		if (ret) {
			printk(KERN_ALERT "lepton: SPI xfer status %d\n", ret);
			retry_ctr++;
			continue;
		}

		if ( (spi_rx_buf[0]   == 0) &&
				(spi_rx_buf[1] == 0) &&
				(spi_rx_buf[2] == 0) &&
				(spi_rx_buf[3] == 0)) {
			printk(KERN_ALERT "lepton: Zero packets found\n");
			msleep(200);	// Give the lepton some time to wake up
			retry_ctr++;
			continue;
		}

		ret_val = copy_to_user((void*) &lepton_dev.read_buff[packets*VOSPI_PKT_SZ], (void*) &spi_rx_buf[0], VOSPI_PKT_SZ);
		if (ret_val != 0) {
			status = -EFAULT;
			printk(KERN_ALERT "copy_to_user fault, %d bytes not copied!\n", ret_val);
			goto exit;
		}

		/* seems ok, reset retries and move to next part of data */

		retry_ctr = 0;
		packets++;
	}

	if (retry_ctr >= MAX_RETRIES) {
		printk(KERN_ALERT "lepton: Retries exceeded\n");
		status = -EIO;
	}
	else {
		/* managed to get through the whole transfer */
		status = num_bytes;
	}

exit:

	return status;
}

static ssize_t lepton_read(struct file *filp, char __user *buff, size_t byte_count,
	loff_t *offp)
{
	ssize_t status = -EFAULT;

	if (!buff)
		return -EFAULT;

	if ((byte_count % VOSPI_PKT_SZ) != 0) {
		return -EINVAL;
	}	

	if (*offp > 0)
		return 0;

	if (down_interruptible(&lepton_dev.fop_sem))
		return -ERESTARTSYS;

	lepton_dev.read_buff = (u8*) buff;

	status = lepton_transfer(lepton_dev.spi_device, byte_count);

	up(&lepton_dev.fop_sem);

	return status;	
}

static int lepton_open(struct inode *inode, struct file *filp)
{	
	int status = 0;

	if (down_interruptible(&lepton_dev.fop_sem))
		return -ERESTARTSYS;

	// do open stuff here

	up(&lepton_dev.fop_sem);

	return status;
}

static int __devinit lepton_probe(struct spi_device *spi)
{
	int ret = 0;

	/* Add per-device initialization code here */
	printk(KERN_ALERT "lepton_probe\n");

	dev_notice(&spi->dev, "probe() called, value: %d\n", 4);

	if (down_interruptible(&lepton_dev.spi_sem))
		return -EBUSY;

	lepton_dev.spi_device = spi;

	up(&lepton_dev.spi_sem);

	lepton_dev.num_transfers = 1;
	lepton_dev.transfer_size = 2;
	lepton_dev.loopback_mode = false;
	lepton_dev.quiet = false;

	if (!spi_rx_buf) {
		printk(KERN_ALERT "lepton.ko: init rx_buf.\n");
		spi_rx_buf = kmalloc( SPI_MAX_XFER, GFP_ATOMIC || GFP_DMA );
		if (!spi_rx_buf) 
			return -ENOMEM;
	}

	if (!spi_tx_buf) {
		printk(KERN_ALERT "lepton.ko: init tx_buf.\n");
		spi_tx_buf = kmalloc( SPI_MAX_XFER, GFP_ATOMIC || GFP_DMA );
		if (!spi_tx_buf) 
			return -ENOMEM;
		memset(spi_tx_buf, 0, SPI_MAX_XFER);
	}

	return ret;
}

#ifdef CONFIG_PM
static int lepton_suspend(struct spi_device *spi, pm_message_t state)
{
	/* Add code to suspend the device here */

	dev_notice(&spi->dev, "suspend() called\n");

	return 0;
}

static int lepton_resume(struct spi_device *spi)
{
	/* Add code to resume the device here */

	dev_notice(&spi->dev, "resume() called\n");

	return 0;
}
#else
/* No need to do suspend/resume if power management is disabled */
#define lepton_suspend	NULL
#define lepton_resume	NULL
#endif

static int lepton_remove(struct spi_device *spi_device)
{
	if (down_interruptible(&lepton_dev.spi_sem))
		return -EBUSY;

	kfree(spi_rx_buf);
	kfree(spi_tx_buf);

	lepton_dev.spi_device = NULL;

	up(&lepton_dev.spi_sem);

	return 0;
}

static struct spi_driver lepton_driver = {
	.driver	= {
		.name		= this_driver_name,
		.owner		= THIS_MODULE,
	},
	.probe		= lepton_probe,
	.remove		= __devexit_p(lepton_remove),
	.suspend	= lepton_suspend,
	.resume		= lepton_resume,
};

static int __init lepton_init_spi(void)
{
	int error;

	error = spi_register_driver(&lepton_driver);
	if (error < 0) {
		printk(KERN_ALERT "spi_register_driver() failed %d\n", error);
		return error;
	}
	return 0;
}

long lepton_unlocked_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg) {
	lepton_iotcl_t q;

	switch (cmd) {
	case QUERY_GET_VARIABLES:
		q.num_transfers = lepton_dev.num_transfers;
		q.transfer_size = lepton_dev.transfer_size;
		q.loopback_mode = lepton_dev.loopback_mode;
		q.quiet = lepton_dev.quiet;
		if (copy_to_user((lepton_iotcl_t *) arg, &q, sizeof(lepton_iotcl_t))) {
			return -EACCES;
		}
		break;
	case QUERY_CLR_VARIABLES:
		lepton_dev.num_transfers = 0;
		lepton_dev.transfer_size = 0;
		lepton_dev.loopback_mode = 0;
		lepton_dev.quiet = 0;
		break;
	case QUERY_SET_VARIABLES:
		if (copy_from_user(&q, (lepton_iotcl_t *) arg,
				sizeof(lepton_iotcl_t))) {
			return -EACCES;
		}
		lepton_dev.num_transfers = q.num_transfers;
		lepton_dev.transfer_size = q.transfer_size;
		lepton_dev.loopback_mode = q.loopback_mode;
		lepton_dev.quiet = q.quiet;
		break;
	default:
		return -EINVAL;
		break;
	}

	return 0;
}

static const struct file_operations lepton_fops = {
	.owner =	THIS_MODULE,
	.read = lepton_read,
	.open =	lepton_open,	
	.unlocked_ioctl = lepton_unlocked_ioctl,
};

static int __init lepton_init_cdev(void)
{
	int error;

	lepton_dev.devt = MKDEV(0, 0);
	printk(KERN_ALERT "lepton_init_cdev()\n");
	error = alloc_chrdev_region(&lepton_dev.devt, 0, 1, this_driver_name);
	if (error < 0) {
		printk(KERN_ALERT "alloc_chrdev_region() failed: %d \n",
			error);
		return -1;
	}

	cdev_init(&lepton_dev.cdev, &lepton_fops);
	lepton_dev.cdev.owner = THIS_MODULE;

	error = cdev_add(&lepton_dev.cdev, lepton_dev.devt, 1);
	if (error) {
		printk(KERN_ALERT "cdev_add() failed: %d\n", error);
		unregister_chrdev_region(lepton_dev.devt, 1);
		return -1;
	}	
	else {
		printk(KERN_ALERT "cdev_add() succeeded: %d\n", error);
	}

	return 0;
}

static int __init lepton_init_class(void)
{
	lepton_dev.class = class_create(THIS_MODULE, this_driver_name);

	if (!lepton_dev.class) {
		printk(KERN_ALERT "class_create() failed\n");
		return -1;
	}

	if (!device_create(lepton_dev.class, NULL, lepton_dev.devt, NULL,
		this_driver_name)) {
		printk(KERN_ALERT "device_create(..., %s) failed\n",
			this_driver_name);
	class_destroy(lepton_dev.class);
	return -1;
}

return 0;
}

static int __init lepton_init(void)
{

	printk(KERN_ALERT "lepton_init\n");

	memset(&lepton_dev, 0, sizeof(lepton_dev));

	sema_init(&lepton_dev.spi_sem, 1);
	sema_init(&lepton_dev.fop_sem, 1);

	if (lepton_init_cdev() < 0)
		goto fail_1;

	if (lepton_init_class() < 0)
		goto fail_2;

	if (lepton_init_spi() < 0)
		goto fail_3;

	return 0;

fail_3:
	device_destroy(lepton_dev.class, lepton_dev.devt);
	class_destroy(lepton_dev.class);

fail_2:
	cdev_del(&lepton_dev.cdev);
	unregister_chrdev_region(lepton_dev.devt, 1);

fail_1:
	return -1;
}
module_init(lepton_init);

static void __exit lepton_exit(void)
{
	spi_unregister_device(lepton_dev.spi_device);
	spi_unregister_driver(&lepton_driver);

	device_destroy(lepton_dev.class, lepton_dev.devt);
	class_destroy(lepton_dev.class);

	cdev_del(&lepton_dev.cdev);
	unregister_chrdev_region(lepton_dev.devt, 1);

	printk(KERN_ALERT "lepton_exit\n");
	

}
module_exit(lepton_exit);

/* Information about this module */
MODULE_DESCRIPTION("Lepton Camera VoSPI Driver");
MODULE_AUTHOR("Rob Baltzer");
MODULE_LICENSE("Dual BSD/GPL");
