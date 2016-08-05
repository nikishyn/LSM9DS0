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

#define READREG										(0x80)
#define LSM9DS0_MULTIPLE_BYTE						(1 << 6)

/* lsm9ds0 gyroscope registers */
#define WHO_AM_I	(0x0F)


#define CTRL_REG1_G			(0x20)    			/* CTRL REG1 */
#define CTRL_REG2_G			(0x21)    			/* CTRL REG2 */
#define CTRL_REG3_G			(0x22)    			/* CTRL_REG3 */
#define CTRL_REG4_G			(0x23)    			/* CTRL_REG4 */
#define CTRL_REG5_G			(0x24)    			/* CTRL_REG5 */
#define	REFERENCE_G			(0x25)    			/* REFERENCE REG */
#define	FIFO_CTRL_REG_G		(0x2E)    			/* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG_G		(0x2F)    			/* FIFO SOURCE REGISTER */


#define LSM9DS0_OUT_X_L_G  							(0x28)
#define LSM9DS0_OUT_X_H_G  							(0x29)
#define LSM9DS0_OUT_Y_L_G 							(0x2A)
#define LSM9DS0_OUT_Y_H_G 							(0x2B)
#define LSM9DS0_OUT_Z_L_G    						(0x2C)
#define LSM9DS0_OUT_Z_H_G 							(0x2D)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
Gyroscope_type Gyroscope;									/*gyroscope object*/						 


	
static struct task_struct *task;
static int data = 0x55;

static struct spi_device *spi_dev;


unsigned char mtx[3];
unsigned char mrx[3];



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

static int16_t LSM9DS0_Gyro_Read(struct spi_device *spi,int16_t *x, int16_t *y, int16_t *z)
{
	int16_t ret = 				0;
	uint8_t TxBuf[7] = {0};
	uint8_t RxBuf[7] = {0};
	
	
	
	struct spi_transfer Transmit = {
		.tx_buf		= TxBuf,
		.rx_buf 	= RxBuf,
		.len		= sizeof(TxBuf),
	};	
	
	struct spi_message	messege;
	TxBuf[0] = READREG | LSM9DS0_MULTIPLE_BYTE | LSM9DS0_OUT_X_L_G;
	
	spi_message_init(&messege);
	spi_message_add_tail(&Transmit, &messege);
	if((ret=spi_sync(spi, &messege))<0)
		return ret;

	*x = ((int16_t) ((RxBuf[2]) << 8) | RxBuf[1]);
	*y = ((int16_t) ((RxBuf[4]) << 8) | RxBuf[3]);
	*z = ((int16_t) ((RxBuf[6]) << 8) | RxBuf[5]);
	
	return ret;

}


static int16_t WriteRegister_G(struct spi_device *spi,unsigned char Adr, unsigned char data)
{
	int16_t ret;
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
	
	return ret;
}


static int16_t ReadRegister_G(struct spi_device *spi,unsigned char Adr/*, int16_t *ReturnData*/)
{
	int16_t ret;
	unsigned char TxBuf[2] = {0};
	unsigned char RxBuf[2] = {0};
	
	struct spi_transfer Transmit = {
		.tx_buf		= TxBuf,
		.rx_buf 	= RxBuf,
		.len		= 2,
	};	
	
	struct spi_message	messege;
	
	TxBuf[0] = Adr | READREG ;
	TxBuf[1] = 0;
	

	
	
	spi_message_init(&messege);
	spi_message_add_tail(&Transmit, &messege);
	if((ret=spi_sync(spi, &messege))<0)
		return ret;

	//printk(KERN_ALERT "Отправил1#%d.\n", mtx[0]);
	//printk(KERN_ALERT "Отправил2#%d.\n", mtx[1]);
	
	//*ReturnData =
	printk(KERN_ALERT "Прочитал1#%d.\n", RxBuf[0]);
	printk(KERN_ALERT "Прочитал2#%d.\n", RxBuf[1]);
	
	return ret;
}




int16_t ToggleGPIO_Thread(void *data)
{
	static int16_t GpioValue  = 1;
	int16_t	Xaxis = 0, Yaxis = 0,Zaxis = 0;

	while(!kthread_should_stop()){          
		if(GpioValue){
			GpioValue = 0;
		}
		else{
			GpioValue = 1;
		}
		
		
		LSM9DS0_Gyro_Read(spi_dev, &Xaxis,&Yaxis,&Zaxis);
		printk(KERN_ALERT "--------------------------------------------------- \n");
		printk(KERN_ALERT "X =#%d.\n", Xaxis);
		printk(KERN_ALERT "Y =#%d.\n", Yaxis);
		printk(KERN_ALERT "Z =#%d.\n", Zaxis);
		//ReadRegister_G(spi_dev,)
		//gpio_set_value(Gyroscope.DataGyrEnableGPIO.gpio,GpioValue);
		//gpio_set_value(Gyroscope.Spi4clk.gpio,GpioValue);
		msleep(2000);	
    }
 
    return 0;
}

static irqreturn_t FifoReady_isr(int16_t irq, void *data)
{
	printk("Semph");
    return IRQ_HANDLED;
}



static int16_t initFifoReadyGPIO(Gyroscope_type *pntGyro)
{

	int16_t ret;
	
	
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

	ReadRegister_G(spi_dev,WHO_AM_I);
	WriteRegister_G(spi_dev,
					CTRL_REG1_G,
					(LSM9DS0_CTRL_REG1_G_Yen | LSM9DS0_CTRL_REG1_G_Xen | LSM9DS0_CTRL_REG1_G_Zen | LSM9DS0_CTRL_REG1_G_PD));
    
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



