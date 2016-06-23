/**
  ******************************************************************************
  * @file    lsm9ds0_gyr_spi.c
  * @author  Nikishyn Roman
  * @version V1.0.0
  * @date    25-05-2016
  * @brief   kernel module for CM-FX6, Linux Ubuntu
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lsm9ds0_spi.h"
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

#define READREG		(0x80)

/* lsm9ds0 gyroscope registers */
#define WHO_AM_I	(0x0F)


#define CTRL_REG1	(0x20)    			/* CTRL REG1 */
#define CTRL_REG2	(0x21)    			/* CTRL REG2 */
#define CTRL_REG3	(0x22)    			/* CTRL_REG3 */
#define CTRL_REG4	(0x23)    			/* CTRL_REG4 */
#define CTRL_REG5	(0x24)    			/* CTRL_REG5 */
#define	REFERENCE	(0x25)    			/* REFERENCE REG */
#define	FIFO_CTRL_REG	(0x2E)    		/* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG	(0x2F)    		/* FIFO SOURCE REGISTER */
#define	OUT_X_L		(0x28)    			/* 1st AXIS OUT REG of 6 */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
Gyroscope_type Gyroscope;									/*gyroscope object*/						 


	
static struct task_struct *task;
static int data = 0x55;

static struct spi_device *spi_dev;


unsigned char mtx[2];
unsigned char mrx[2];



int spi_drv_probe(struct spi_device *spi);
int spi_drv_remove(struct spi_device *spi);
 

struct spi_driver spi_drv = {
    .driver = {
        .name = "spilsm",
        .owner = THIS_MODULE,
    },
    .probe = spi_drv_probe,
    .remove = spi_drv_remove,
};



static int WriteRegister_G(struct spi_device *spi,unsigned char Adr, unsigned char data)
{
	int ret;
	struct spi_transfer t = {
		.tx_buf		= mtx,
		.rx_buf 	= mrx,
		.len		= 2,
	};
	
	struct spi_message	m;
	mtx[0]= Adr;
	mtx[1] = data;
	
	mrx[0]=0;
    mrx[0]=0;
	
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	if((ret=spi_sync(spi, &m))<0)
		return ret;
/*
	printk(KERN_ALERT "Отправил1#%d.\n", mtx[0]);
	printk(KERN_ALERT "Отправил2#%d.\n", mtx[1]);
	
	printk(KERN_ALERT "Прочитал1#%d.\n", mrx[0]);
	printk(KERN_ALERT "Прочитал2#%d.\n", mrx[1]);
*/	
	return ret;
}


static int ReadRegister_G(struct spi_device *spi,unsigned char Adr)
{
	int ret;
	struct spi_transfer t = {
		.tx_buf		= mtx,
		.rx_buf 	= mrx,
		.len		= 2,
	};	
	
	struct spi_message	m;
	
	mtx[0] = Adr | READREG ;
	mtx[1] = 0xAA;
	
	mrx[0]=0;
    mrx[1]=0;
	
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	if((ret=spi_sync(spi, &m))<0)
		return ret;
/*
	printk(KERN_ALERT "Отправил1#%d.\n", mtx[0]);
	printk(KERN_ALERT "Отправил2#%d.\n", mtx[1]);
	
	printk(KERN_ALERT "Прочитал1#%d.\n", mrx[0]);
	printk(KERN_ALERT "Прочитал2#%d.\n", mrx[1]);
*/
	return ret;
}




int ToggleGPIO_Thread(void *data)
{
	static int GpioValue  = 1;


	while(!kthread_should_stop()){          
		if(GpioValue){
			GpioValue = 0;
		}
		else{
			GpioValue = 1;
		}
		
		ReadRegister_G(spi_dev,WHO_AM_I);
		//gpio_set_value(Gyroscope.DataGyrEnableGPIO.gpio,GpioValue);
		//gpio_set_value(Gyroscope.Spi4clk.gpio,GpioValue);
		msleep(20);	
    }
 
    return 0;
}

static irqreturn_t FifoReady_isr(int irq, void *data)
{
	printk("Semph");
    return IRQ_HANDLED;
}



static int initFifoReadyGPIO(Gyroscope_type *pntGyro)
{

	int ret;
	
	
	pntGyro->DataGyrEnableGPIO.gpio 	= LSM9DS0_GYR_DEFAULT_DATA_GYR_ENABLE;
    pntGyro->DataGyrEnableGPIO.flags 	= GPIOF_OUT_INIT_LOW;
    pntGyro->DataGyrEnableGPIO.label 	= "Output Gpio";
 

/*
	pntGyro->FifoReadyGPIO.gpio		= 	LSM9DS0_GYR_DEFAULT_FIFO_READY_GPIO;
	pntGyro->FifoReadyGPIO.flags 	= 	GPIOF_IN;
	pntGyro->FifoReadyGPIO.label 	= 	"In Gpio";

*/


    ret = gpio_request(pntGyro->DataGyrEnableGPIO.gpio, pntGyro->DataGyrEnableGPIO.label);
    if (ret)
    {
        printk(KERN_ALERT "OUT Gpio request failed");
		printk(KERN_INFO "GPIOEN#%d.\n", pntGyro->DataGyrEnableGPIO.gpio);
        return -1;
    }
    gpio_direction_output(pntGyro->DataGyrEnableGPIO.gpio, 0);
    gpio_set_value(pntGyro->DataGyrEnableGPIO.gpio, 0);
	


/*	
	ret = gpio_request(pntGyro->FifoReadyGPIO.gpio,pntGyro->FifoReadyGPIO.label);
	
	if (ret)
	{
		printk(KERN_ALERT "In Gpio request failed");
		printk(KERN_INFO "GPIOEXT#%d.\n", pntGyro->FifoReadyGPIO.gpio);
		return -2;
	}

	ret = gpio_to_irq(pntGyro->FifoReadyGPIO.gpio);
	if (ret < 0)
	{
		printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
		return -3;
	}
	
	pntGyro->irq_num = ret;
	printk(KERN_INFO "Successfully requested IRQ#%d.\n", pntGyro->irq_num);
	ret = request_irq(pntGyro->irq_num,FifoReady_isr,IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_DISABLED, "gpio_irq", NULL);
	
	if (ret)
	{
		printk(KERN_ERR "Unable to request irq\n");
		return -4;
	}
	
*/
	return 0;
}




int spi_drv_probe(struct spi_device *spi)
{
    int ret;
 
    printk(KERN_NOTICE "spi drv probe!\n" );
 
    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_0 ;
    spi->max_speed_hz = 1000000;
     
    ret = spi_setup(spi);
    if (ret < 0){
		printk(KERN_ALERT "SPI NO SET UP.\n");
        return ret;
	}
    spi_dev = spi;
 
	
	WriteRegister_G(spi_dev,CTRL_REG1, 0x0F);	
    return 0;
}
 
 
int spi_drv_remove(struct spi_device *spi)
{
    printk(KERN_NOTICE "spi drv remove!\n" );
    return 0;
}



static int __init lsm9ds0_gyr_init(void)
{

	int ret;
	ret = initFifoReadyGPIO(&Gyroscope);
	if (ret){
		printk("Error driver set up");
		return -1;
	}
	else{
		
		ret = spi_register_driver(&spi_drv);
		if (ret < 0)
		{
			printk(KERN_ALERT "Cant register spi dev.\n");
			return ret;
		}
		else
		{
			printk(KERN_ALERT "SPI oK.\n");
		}
		
		task = kthread_run(&ToggleGPIO_Thread,(void *)data,"ToogleGPIO");
		
	/*	mtx=kzalloc(2, GFP_KERNEL);
		mrx=kzalloc(2, GFP_KERNEL);
	*/	
		
		pr_info("%s: gyroscope sysfs driver init\n", LSM9DS0_GYR_DEV_NAME);		
		return 0;	
	}
	
	return 0;
	
	
}

static int deinitFifoReadyGPIO(Gyroscope_type *pntGyro)
{
	
	gpio_set_value(pntGyro->DataGyrEnableGPIO.gpio,0);
	//free_irq(pntGyro->irq_num,NULL);
	gpio_free(pntGyro->DataGyrEnableGPIO.gpio);
    //gpio_free(pntGyro->FifoReadyGPIO.gpio);

	
	return 0;
}




static void __exit lsm9ds0_gyr_exit(void)
{
	deinitFifoReadyGPIO(&Gyroscope);
	kthread_stop(task);
	
	/*kfree(mtx);
	kfree(mrx);*/
	spi_unregister_driver(&spi_drv);

	pr_info("%s: exit\n", LSM9DS0_GYR_DEV_NAME);
	return;
}

module_init(lsm9ds0_gyr_init);
module_exit(lsm9ds0_gyr_exit);

MODULE_DESCRIPTION("lsm9ds0 gyroscope driver use SPI");
MODULE_AUTHOR("Roman Nikishyn, TRIOL");
MODULE_LICENSE("GPL");



