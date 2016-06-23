/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef             LSM9DS0_H
#define             LSM9DS0_H

/* Includes ------------------------------------------------------------------*/
#include <linux/init.h> 
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>

/*Define----------------------------------------------------------------------*/
#define LSM9DS0_GYR_DEV_NAME	"lsm9ds0_gyr_spi"	   /* Input file name ----*/

#define IMX_GPIO_NR(bank, nr)           (((bank) - 1) * 32 + (nr))

#define LSM9DS0_GYR_DEFAULT_FIFO_READY_GPIO				(IMX_GPIO_NR(3,30))
#define LSM9DS0_GYR_DEFAULT_DATA_GYR_ENABLE				(IMX_GPIO_NR(3,31))		


/* typedef -------------------------------------------------------------------*/

typedef struct{
  
	struct gpio FifoReadyGPIO;									/* Pin signal FIFO ready - IN */
	struct gpio DataGyrEnableGPIO;								/* Pin signal Data enable gyroscope - Out */

	int irq_num;
}Gyroscope_type;

/*extern variable ------------------------------------------------------------*/


/*Function prototypes---------------------------------------------------------*/  

#endif
