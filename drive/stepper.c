#include <linux/fs.h>      /* file stuff */
#include <linux/kernel.h>  /* printk() */
#include <linux/errno.h>   /* error codes */
#include <linux/module.h>  /* THIS_MODULE */
#include <linux/device.h>  /* Header to support the kernel Driver Model */
#include <linux/cdev.h>    /* char device stuff */
#include <linux/init.h>    /* module_init, module_exit */
#include <linux/module.h>  /* version info, MODULE_LICENSE, MODULE_AUTHOR, printk() */
#include <linux/compiler.h> /* __must_check */
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/ioport.h>
#include <asm/io.h>

// sudo mknod /dev/stepper c 240 1
// 1 seconds = 102 jiffies
// Pinout web: https://pinout.xyz/pinout/1_wire#
//
// BOT:  BCM0  - Connector 27 - WiringPi 30 
//    GPFSEL0 bits 2,1,0    Linux addr: 0x20200000 (real addr 0x7E200000)
//    GPLEV0  bit 0         Linux addr: 0x20200034 (real addr 0x7E200034)
//
// EOT:  BCM5  - Connector 29 - WiringPi 21
//    GPFSEL0 bits 17,16,15 Linux addr: 0x20200000 (real addr 0x7E200000)
//    GPLEV0  bit 5         Linux addr: 0x20200034 (real addr 0x7E200034)
//
// dir:  BCM6  - Connector 31 - WiringPi 22
//    GPFSEL0 bits 20,19,18 Linux addr: 0x20200000 (real addr 0x7E200000)
//    GPSET0  bit 6         Linux addr: 0x2020001c (real addr 0x7E20001C)
//    GPCLR0  bit 6         Linux addr: 0x20200028 (real addr 0x7E200028)
#define DIR_SEL_OFSET 0
#define DIR_SEL_VAL (1 << 18) // 001 << 18
#define DIR_SET_OFFSET 0x1c
#define DIR_CLR_OFFSET 0x28
#define DIR_BIT (1 << 6)
//
// step: BCM13 - Connector 33 - WiringPi 23
//    GPFSEL1 bits 11,10,9  Linux addr: 0x20200004 (real addr 0x7E200004)
//    GPSET0  bit 13        Linux addr: 0x2020001c (real addr 0x7E20001C)
//    GPCLR0  bit 13        Linux addr: 0x20200028 (real addr 0x7E200028)
//
#define STEP_SEL_OFFSET 0x04
#define STEP_SEL_VAL (1 << 9) // 001 << 9
#define STEP_SET_OFFSET 0x1c
#define STEP_CLR_OFFSET 0x28
#define STEP_BIT (1 << 13)

//#define  PORT  0x20200000  // Raspberry whatever
#define  PORT  0xff200000  // Raspberry PI 3B - see: cat /proc/iomem
#define  RANGE   0x40

#define NUM_PULSES 10
#define TM	100       // 10 milliseconds

static void timer_handler(struct timer_list * timerlist);
static unsigned long tm = TM;
static unsigned long tm_counter = NUM_PULSES;

DEFINE_TIMER(mTimer, timer_handler);

#define PROMPT "stepper_drv was open %d times\n"

static const char DEVICE_NAME[] = "stepper_drv";

__must_check int register_device(void); /* 0 if Ok*/

void  unregister_device(void); 

MODULE_LICENSE("Apache-2.0");
MODULE_AUTHOR("Marcelo Varanda");

static int   nOpenTimes = 0;
static char  prompt[64];
static int   prompt_len = 0;
static void * io_base_addr;
static int   numberOpens = 0;
static int   phase = 0;

static void setTimer(unsigned long ms)
{
  int ret;
  ret = mod_timer(&mTimer, jiffies + msecs_to_jiffies(ms));
  printk( KERN_NOTICE "stepper_drv: mod_timer ret = %d", ret);
  
}

static void timer_handler(struct timer_list * timerlist)
{
  unsigned long j = jiffies;
  printk( KERN_NOTICE "stepper_drv: timer_handler");
  if (tm_counter == 0) {
     printk( KERN_NOTICE "stepper_drv: tm_counter = 0");
    return;
  }
  
   if (request_mem_region(PORT, RANGE, DEVICE_NAME) == NULL) {
     printk( KERN_NOTICE "stepper_drv (timer): could not reserve I/O area");
     return;
   }

   //----- set STEP and DIR as outputs
   if ((io_base_addr = ioremap(PORT, RANGE)) == NULL) {
     printk( KERN_NOTICE "stepper_drv (timer): ioremap fail");
     release_mem_region(PORT, RANGE);
     return;
   }
   if (phase++ & 1) {
     writel(STEP_BIT, io_base_addr + STEP_SET_OFFSET);  
   }
   else {
     writel(STEP_BIT, io_base_addr + STEP_CLR_OFFSET);  
   }

   release_mem_region(PORT, RANGE);

   printk( KERN_NOTICE "timer_handler expired at %u jiffies\n", (unsigned)j);
  setTimer(tm);
  tm_counter--;
}

static int device_open(struct inode *inodep, struct file *file_ptr){
   unsigned char d;

   if (numberOpens != 0) {
     printk( KERN_NOTICE "stepper_drv: file already open");
     return -EBUSY;
   }

   if (request_mem_region(PORT, RANGE, DEVICE_NAME) == NULL) {
     printk( KERN_NOTICE "stepper_drv (open): could not reserve I/O area");
     return -EBUSY;
   }

   //----- set STEP and DIR as outputs
   if ((io_base_addr = ioremap(PORT, RANGE)) == NULL) {
     printk( KERN_NOTICE "stepper_drv: ioremap fail");
     release_mem_region(PORT, RANGE);
     return -EBUSY;
   }
   printk( KERN_NOTICE "stepper_drv: config I/O pins");
   d = readl(io_base_addr + STEP_SEL_OFFSET);
   d |= STEP_SEL_VAL;
   writel(d, io_base_addr + STEP_SEL_OFFSET);

   writel(STEP_BIT, io_base_addr + STEP_CLR_OFFSET);  

   release_mem_region(PORT, RANGE);

   printk( KERN_NOTICE "stepper_drv: device open fine");
   numberOpens++;
   

   return 0;
}

static int     device_close(struct inode *inodep, struct file *file_ptr)
{
   printk( KERN_NOTICE "stepper_drv: device close fine");
   numberOpens = 0;
   return 0;
}

/*=======================================================================*/
static ssize_t device_file_read(
                           struct file *file_ptr
                        , char __user *user_buffer
                        , size_t count
                        , loff_t *possition)
{
   printk( KERN_NOTICE "stepper_drv: Device file is read at offset = %i, read bytes count = %u"
            , (int)*possition
            , (unsigned int)count );

   if( *possition >= prompt_len )
      return 0;

   prompt_len = sprintf(prompt, PROMPT, nOpenTimes++);

   if( *possition + count > prompt_len )
      count = prompt_len - *possition;

   if( copy_to_user(user_buffer, prompt + *possition, count) != 0 )
      return -EFAULT;   

   setTimer(TM);
   tm_counter = NUM_PULSES;

   *possition += count;
  return count;
}
/*==========================================================================*/
static struct file_operations simple_driver_fops = 
{
   .owner   = THIS_MODULE,
   .read    = device_file_read,
   .open    = device_open,
   .release = device_close,
};

static int device_file_major_number = 0;


/*========================================================================*/
int register_device(void)
{
      int result = 0;

      printk( KERN_NOTICE "stepper_drv: register_device() is called." );

      result = register_chrdev( 0, DEVICE_NAME, &simple_driver_fops );
      if( result < 0 )
      {
         printk( KERN_WARNING "stepper_drv:  can\'t register character device with errorcode = %i", result );
         return result;
      }

      device_file_major_number = result;
      printk( KERN_NOTICE "stepper_drv: registered character device with major number = %i and minor numbers 0...255"
                  , device_file_major_number );

      prompt_len = sprintf(prompt, PROMPT, nOpenTimes++);
      return 0;
}
/*--------------------------------------------------------------------------*/
void unregister_device(void)
{
   del_timer(&mTimer);
   
   printk( KERN_NOTICE "stepper_drv: unregister_device() is called" );
   if(device_file_major_number != 0)
   {
      unregister_chrdev(device_file_major_number, DEVICE_NAME);
   }
}

/*==========================================================================*/
static int simple_driver_init(void)
{
      int result = 0;
    printk( KERN_NOTICE "stepper_drv: Initialization started" );

      result = register_device();
    return result;
}
/*--------------------------------------------------------------------------*/
static void simple_driver_exit(void)
{
   printk( KERN_NOTICE "stepper_drv: Exiting" );
    unregister_device();
}
/*==========================================================================*/

module_init(simple_driver_init);
module_exit(simple_driver_exit);

