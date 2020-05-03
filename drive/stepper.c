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

// sudo mknod /dev/stepper c 240 1
// 1 seconds = 102 jiffies

static void timer_handler(struct timer_list * timerlist);
static unsigned long tm = 1000;
static unsigned long tm_counter;

DEFINE_TIMER(mTimer, timer_handler);

static void setTimer(unsigned long ms)
{
   mod_timer(&mTimer, jiffies + msecs_to_jiffies(ms));
}

static void timer_handler(struct timer_list * timerlist)
{
  unsigned long j = jiffies;
  if (tm_counter == 0) return;
  
  printk( KERN_NOTICE "timer_handler expired at %u jiffies\n", (unsigned)j);
  setTimer(tm);
  //mod_timer(&mTimer, jiffies + tm);
  tm_counter--;
}

#define PROMPT "stepper_drv was open %d times\n"

__must_check int register_device(void); /* 0 if Ok*/

void  unregister_device(void); 

MODULE_LICENSE("Apache-2.0");
MODULE_AUTHOR("Marcelo Varanda");

static int   nOpenTimes = 0;
static char  prompt[64];
static int   prompt_len = 0;
static int   numberOpens = 0;

static int device_open(struct inode *inodep, struct file *file_ptr){
   if (numberOpens != 0) {
     printk( KERN_NOTICE "stepper_drv: file already open");
     return -EBUSY;
   }
   printk( KERN_NOTICE "stepper_drv: device open fine");
   numberOpens++;
   
   tm = 1000;

   //mod_timer(&mTimer, jiffies + tm);
   setTimer(tm);
   tm_counter = 5;

   return 0;
}

static int     device_close(struct inode *inodep, struct file *file_ptr)
{
   printk( KERN_NOTICE "stepper_drv: device close fine");
   numberOpens = 0;
   //tm_counter = 0;
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

static const char device_name[] = "stepper_drv";

/*========================================================================*/
int register_device(void)
{
      int result = 0;

      printk( KERN_NOTICE "stepper_drv: register_device() is called." );

      result = register_chrdev( 0, device_name, &simple_driver_fops );
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
      unregister_chrdev(device_file_major_number, device_name);
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

