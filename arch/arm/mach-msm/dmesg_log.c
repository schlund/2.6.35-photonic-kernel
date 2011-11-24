#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

static int pk_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
        char string[256];
        count = count < 255 ? count : 255;

        if(copy_from_user(string, buffer, count))
                return -EFAULT;

        string[count] = '\0';        
        printk(string);
        return count;
}


static int __init printk_init(void)
{
        struct proc_dir_entry *pk_file;

        pk_file = create_proc_entry("printk", 0222, NULL);
        if(pk_file == NULL)
                return -ENOMEM;

        pk_file->write_proc = pk_write;
        //pk_file->owner = THIS_MODULE;

        return 0;
}

static void __exit printk_cleanup(void)
{
        remove_proc_entry("printk", NULL);
}

module_init(printk_init);
module_exit(printk_cleanup);
MODULE_LICENSE("GPL");
