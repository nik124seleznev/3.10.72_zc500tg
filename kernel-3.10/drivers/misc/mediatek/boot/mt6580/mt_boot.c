#define pr_fmt(fmt) "["KBUILD_MODNAME"] " fmt
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/kfifo.h>

#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/of.h>

#include <mach/mt_boot.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>
#include <asm/setup.h>

/* this vairable will be set by mt_fixup.c */

/*[FACTORY_TEST_BY_APK] modify start*/
BOOTMODE g_boot_mode_ex __nosavedata = UNKNOWN_BOOT;//Huangyisong_add 20130823
static ssize_t show_boot_status(struct device *dev,struct device_attribute *attr, char *buf)
{
   unsigned int ret_value = 1;
        printk("songjiangchao-->g_boot_mode-->%d \n", g_boot_mode);
        printk("songjiangchao-->g_boot_mode_ex-->%d \n", g_boot_mode_ex);
	if(11==g_boot_mode_ex)
//		ret_value = sprintf(buf, "%s", "1"); 	
		*buf = 1 ;	
	else
//		ret_value = sprintf(buf, "%s", "0");
  		*buf = 0 ;
	  return ret_value;
}
static  DEVICE_ATTR(boot_status, 0444,show_boot_status, NULL);
static int hq_boot_status(struct platform_device *dev)
{
		int ret_device_file = 0;
    printk("** hq_boot_status_probe!! **\n" );
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_boot_status)) != 0) goto exit_error;
exit_error:	
    return ret_device_file;
}
static struct platform_driver Hq_boot_status_driver = {
        .probe		= hq_boot_status,
        .driver     = {
        .name = "hq_boot_status",
    },
};

static struct platform_device Hq_boot_status_device = {
    .name   = "hq_boot_status",
    .id	    = -1,
};
static int __init Hq_boot_status_mod_init(void)
{
    int ret = 0;


    ret = platform_device_register(&Hq_boot_status_device);
    if (ret) {
        printk("**hq_boot_status_mod_init  Unable to driver register(%d)\n", ret);
        platform_driver_unregister(&Hq_boot_status_driver);
    }
    

    ret = platform_driver_register(&Hq_boot_status_driver);
    if (ret) {
        printk("**hq_boot_status_mod_init  Unable to driver register(%d)\n", ret);
        platform_device_unregister(&Hq_boot_status_device);
    }

    return ret;   
}

static void __exit Hq_boot_status_mod_exit(void)
{
		
        platform_driver_unregister(&Hq_boot_status_driver);
	platform_device_unregister(&Hq_boot_status_device);
}
module_init(Hq_boot_status_mod_init);
module_exit(Hq_boot_status_mod_exit);
/*[FACTORY_TEST_BY_APK] modify start*/
BOOTMODE get_boot_mode_ex(void)
{
    return g_boot_mode_ex;
}
/*[FACTORY_TEST_BY_APK] modify end*/
/*[FACTORY_TEST_BY_APK] modify end*/

META_COM_TYPE g_meta_com_type = META_UNKNOWN_COM;
unsigned int g_meta_com_id = 0;
unsigned int g_meta_uart_port = 0;

struct meta_driver {
    struct device_driver driver;
    const struct platform_device_id *id_table;
};

static struct meta_driver meta_com_type_info = {
    .driver  = {
        .name = "meta_com_type_info",
        .bus = &platform_bus_type,
        .owner = THIS_MODULE,
    },
    .id_table = NULL,
};

static struct meta_driver meta_com_id_info = {
    .driver = {
        .name = "meta_com_id_info",
        .bus = &platform_bus_type,
        .owner = THIS_MODULE,
    },
    .id_table = NULL,
};

static struct meta_driver meta_uart_port_info = {
    .driver = {
        .name = "meta_uart_port_info",
        .bus = &platform_bus_type,
        .owner = THIS_MODULE,
    },
    .id_table = NULL,
};



#ifdef CONFIG_OF
struct boot_tag_meta_com {
	u32 size;
	u32 tag;
	u32 meta_com_type; /* identify meta via uart or usb */
	u32 meta_com_id;  /* multiple meta need to know com port id */
	u32 meta_uart_port; /* identify meta uart port number */
};
#endif

bool com_is_enable(void)  // usb android will check whether is com port enabled default. in normal boot it is default enabled.
{
    if (get_boot_mode() == NORMAL_BOOT) {
        return false;
	} else {
        return true;
	}
}

void set_meta_com(META_COM_TYPE type, unsigned int id)
{
    g_meta_com_type = type;
    g_meta_com_id = id;
}

META_COM_TYPE get_meta_com_type(void)
{
    return g_meta_com_type;
}

unsigned int get_meta_com_id(void)
{
    return g_meta_com_id;
}

unsigned int get_meta_uart_port(void)
{
    return g_meta_uart_port;
}


static ssize_t meta_com_type_show(struct device_driver *driver, char *buf)
{
    return sprintf(buf, "%d\n", g_meta_com_type);
}

static ssize_t meta_com_type_store(struct device_driver *driver, const char *buf, size_t count)
{
    /*Do nothing*/
    return count;
}

DRIVER_ATTR(meta_com_type_info, 0644, meta_com_type_show, meta_com_type_store);


static ssize_t meta_com_id_show(struct device_driver *driver, char *buf)
{
    return sprintf(buf, "%d\n", g_meta_com_id);
}

static ssize_t meta_com_id_store(struct device_driver *driver, const char *buf, size_t count)
{
    /*Do nothing*/
    return count;
}

DRIVER_ATTR(meta_com_id_info, 0644, meta_com_id_show, meta_com_id_store);
static ssize_t meta_uart_port_show(struct device_driver *driver, char *buf)
{
    return sprintf(buf, "%d\n", g_meta_uart_port);
}

static ssize_t meta_uart_port_store(struct device_driver *driver, const char *buf, size_t count)
{
    /*Do nothing*/
    return count;
}

DRIVER_ATTR(meta_uart_port_info, 0644, meta_uart_port_show, meta_uart_port_store);


static int __init create_sysfs(void)
{
    int ret;
    BOOTMODE bm = get_boot_mode();
    pr_notice("[%s] songjiangchao-->create_sysfs() ,,,bm -->%d,,\n", __func__ , bm);   //songjiangchao,2015-03-31,add
    printk("[%s] songjiangchao-->create_sysfs() ,,,bm -->%d,,\n", __func__ , bm); //songjiangchao,2015-03-31,add
#ifdef CONFIG_OF
    if (of_chosen) {
        struct boot_tag_meta_com *tags;
        tags = (struct boot_tag_meta_com *)of_get_property(of_chosen, "atag,meta", NULL);
        if (tags) {
            g_meta_com_type = tags->meta_com_type;
            g_meta_com_id = tags->meta_com_id;
			g_meta_uart_port = tags->meta_uart_port;
            pr_notice("[%s] g_meta_com_type = %d, g_meta_com_id = %d, g_meta_uart_port=%d. \n", __func__, g_meta_com_type, g_meta_com_id,g_meta_uart_port);
        }
        else
            pr_notice("[%s] No atag,meta found !\n", __func__);
    }
    else
        pr_notice("[%s] of_chosen is NULL !\n", __func__);
#endif

    if(bm == META_BOOT || bm == ADVMETA_BOOT || bm == ATE_FACTORY_BOOT || bm == FACTORY_BOOT) {
        /* register driver and create sysfs files */
        ret = driver_register(&meta_com_type_info.driver);
        if (ret) {
            pr_warn("fail to register META COM TYPE driver\n");
        }
        ret = driver_create_file(&meta_com_type_info.driver, &driver_attr_meta_com_type_info);
        if (ret) {
            pr_warn("fail to create META COM TPYE sysfs file\n");
        }

        ret = driver_register(&meta_com_id_info.driver);
        if (ret) {
            pr_warn("fail to register META COM ID driver\n");
        }
        ret = driver_create_file(&meta_com_id_info.driver, &driver_attr_meta_com_id_info);
        if (ret) {
            pr_warn("fail to create META COM ID sysfs file\n");
        }
		ret = driver_register(&meta_uart_port_info.driver);
        if (ret) {
            pr_warn("fail to register META UART PORT driver\n");
        }
        ret = driver_create_file(&meta_uart_port_info.driver, &driver_attr_meta_uart_port_info);
        if (ret) {
            pr_warn("fail to create META UART PORT sysfs file\n");
        }
    }
//songjiangchao,2015-03-31,add start	
    /*[FACTORY_TEST_BY_APK] modify start*/
//     if ( g_boot_mode == 11 ) {
     if ( bm == 11 ) {
            g_boot_mode = NORMAL_BOOT;
            g_boot_mode_ex = 11;
    }
    /*[FACTORY_TEST_BY_APK] modify   end */
//songjiangchao,2015-03-31,add  end	
    return 0;
}

static int __init boot_mod_init(void)
{
    create_sysfs();
    return 0;
}

static void __exit boot_mod_exit(void)
{
}

module_init(boot_mod_init);
module_exit(boot_mod_exit);
MODULE_DESCRIPTION("MTK Boot Information Querying Driver");
MODULE_LICENSE("GPL");
/*[FACTORY_TEST_BY_APK] modify start*/
EXPORT_SYMBOL(get_boot_mode_ex);//Huangyisong_add 20130823
/*[FACTORY_TEST_BY_APK] modify end*/
