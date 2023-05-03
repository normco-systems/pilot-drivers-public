#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <asm/unaligned.h>

#define DEVICE_NAME "nectar_temperature"
#define REF_Z 1600
#define GAIN 16
#define GPIO_RESET "reset"
#define GPIO_INTERRUPT "int"
#define READ_PERIOD 1

/* Commands */
#define CMD_RREAD(reg) ((1 << 5) | (reg & 0x1F))
#define CMD_WREG(reg) ((1<<6) | (reg & 0x1F))
#define CMD_NOP 0x00
#define CMD_WAKEUP 0x02
#define CMD_POWERDOWN 0x04
#define CMD_RESET 0x06
#define CMD_START 0x08
#define CMD_STOP 0x0A
#define CMD_SYOCAL 0x16
#define CMD_SYGCAL 0x17
#define CMD_SFOCAL 0x19
#define CMD_RDATA 0x12

/* Device register maps */
#define REG_ID 0x00
#define REG_STATUS 0x01
#define REG_INPMUX 0x02  
#define REG_PGA 0x03
#define REG_DATARATE 0x04
#define REG_REF 0x05
#define REG_IDACMAG 0x06
#define REG_IDACMUX 0x07
#define REG_VBIAS 0x08
#define REG_SYS 0x09
#define REG_OFCAL0 0x0A
#define REG_OFCAL1 0x0B
#define REG_OFCAL2 0x0C
#define REG_FSCAL0 0x0D
#define REG_FSCAL1 0x0E
#define REG_FSCAL2 0x0F
#define REG_GPIODAT 0x10
#define REG_GPIOCON 0x11

/* Values for the registers */
/* INPMUX */
#define AIN0 0x00
#define AIN1 0x01
#define AIN2 0x02
#define AIN3 0x03
#define AIN4 0x04
#define AIN5 0x05
#define AIN6 0x06
#define AIN7 0x07
#define AIN8 0x08
#define AIN9 0x09
#define AIN10 0x0A
#define AIN11 0x0B
#define MX_NEG(pin) pin
#define MX_POS(pin) (pin << 4)
/* PGA */
#define DELAY_14 0x00
#define PGA_ENABLE 0x08
#define GAIN_1 0x00
#define GAIN_2 0x01
#define GAIN_4 0x02
#define GAIN_8 0x03
#define GAIN_16 0x04
#define GAIN_32 0x05
/* DATARATE */
#define GCHOP_DISABLE 0x00
#define GCHOP_ENABLE 0x80
#define CLOCK_INTERNAL 0x00
#define CLOCK_EXTERNAL 0x40
#define CMODE_CONT 0x00
#define CMODE_SINGLE 0x20
#define DF_SINC 0x00
#define DF_LOWL 0x10
#define NDR_2 0x00
#define NDR_5 0x01
#define NDR_10 0x02
#define NDR_16 0x03
#define NDR_20 0x04
/* REF */
#define REF_MON_DISABLE 0x00
#define REFP_BUF_ENABLE 0x00
#define REFP_BUF_DISABLE 0x20
#define REFN_BUF_ENABLE 0x00
#define REFN_BUF_DISABLE 0x10
#define REF_SEL0 0x00
#define REF_SEL1 0x04
#define INTL_V_OFF 0x00
#define INTLV_ON 0x01
#define INTLV_ON_A 0x02
/* IDACMAG */
#define PGA_RAIL_FLAG_ENABLE 0x80
#define PGA_RAIL_FLAG_DISABLE 0x00
#define PSW_CLOSED 0x40
#define PSW_OPEN 0x00
#define IMAG_OFF 0x00
#define IMAG_10 0x01
#define IMAG_50 0x02
#define IMAG_100 0x03
#define IMAG_250 0x04
#define IMAG_1000 0x07
#define IDAC_OFF 0x0F
/* IDACMUX - for 'pin' see INPMUX */
#define MX_IDAC1(pin) pin
#define MX_IDAC2(pin) (pin << 4)

/** Macros to help change a single value in the register 
 * reg_mask will mask off the registers based in start and end bits
 * change_only will take a mask of what bits to change, old reg value and new bit values for the property to be changed
*/
#define reg_mask(i, j) (((1 << i) - 1) ^ ((1 << (j+1)) - 1))
#define change_only(mask, current, new) ((current & ~mask) | (new & mask))

/* ADC register settings for read-only
*  It is the programmers responsibility to assign the correct register values
*  changing these values have no effect on register values 
*/
struct adc_settings {
#define SETTINGS_INIT {     \
    .gain = GAIN,           \
    .ref_z = REF_Z,         \
}
    unsigned int gain;      //gain of the pga
    unsigned int ref_z;     // value of the reference resitor to compare value against in ohms
};

/* Private data structure for the device */
struct nectar_temp {
    spinlock_t lock;
    dev_t dev_id;
    struct device *parent_dev;
    struct class *dev_class;
    struct gpio_desc *gpio_interrupt;
    struct gpio_desc *gpio_reset;
    int irq;
    struct adc_settings settings;
    u8 reg_values[0x12];
    int active_probe;
    int num_probe;
    struct temp_probe *probes[];    
};

/* Data structure for an indivudaul temperature probe device */
struct temp_probe {
    struct nectar_temp *priv;
    spinlock_t lock;
    dev_t dev_id;
    int id;
    u32 rtd_z;
};

/* Convert from 24bit array intom unsigned 32bit number */
static inline u32 get_unaligned_be24(u8 *buf)
{
	return 0xffffff & (u32) get_unaligned_be32(buf - 1);
}

/** @brief read the value of the ADC register
 * @param ref the address of the register to read the value from
 * @param buf a byte buffer to hold the read value 
*/
static int read_reg(struct device *dev, u8 reg, u8 *buf){
    int ret;
    u8 cmd[2];
    struct spi_transfer t[2];
    struct spi_device *spi = to_spi_device(dev);

    memset(t, 0, sizeof(t));

    cmd[0] = CMD_RREAD(reg);
    cmd[1] = 0x00;

    t[0].tx_buf = cmd;
    t[0].len = 2;
    
    t[1].rx_buf = buf;
    t[1].len = 1;

    ret = spi_sync_transfer(spi, t, 2);
    if(ret != 0){
        pr_err("spi register read failed: %d\n", ret);
    }
    
    return ret;
}

/** @brief write a value to a register of the ADC
 *  @param reg address of the register to write to
 *  @param value the new register value to write to the register 
 */
static int write_reg(struct device *dev, u8 reg, u8 *value){
    int status;
    u8 data[3];
    struct spi_device *spi = to_spi_device(dev);

    data[0] = CMD_WREG(reg);
    data[1] = 0x00;
    data[2] = *value;

    status = spi_write(spi, data, 3);

    return status;
}

/* Sends a command to the ADC */
static int send_command(struct device *dev, u8 cmd){
    int status;
    struct spi_device *spi = to_spi_device(dev);
    status = spi_write(spi, &cmd, 1);

    return status;
}

/** @brief Read the converted value from the ADC
 * @param dev The device struct of the SPI
 * @param buf a byte array of the appropriate size (3 bytes) to hold the returned value
*  
*/
static int read_value(struct device *dev, u8 *buf){
    int ret;
    u8 cmd;
    struct spi_transfer t[2];
    struct spi_device *spi = to_spi_device(dev);

    memset(t, 0, sizeof(t));

    cmd = CMD_RDATA;

    t[0].tx_buf = &cmd;
    t[0].len = 1;
    
    t[1].rx_buf = buf;
    t[1].len = 3;

    ret = spi_sync_transfer(spi, t, 2);
    if(ret != 0){
        pr_err("spi register read failed: %d\n", ret);
    }
    
    return ret;
}

/* Interrupt handlers of the driver */
static irqreturn_t irq_handler(int irq, void *handle){

    return IRQ_WAKE_THREAD;
}

static irqreturn_t irq_threaded(int irq, void *handle){
    u8 buf[3];
    int ret;
    struct nectar_temp *priv = handle;

    /* Read the adc value */
    ret = read_value(priv->parent_dev, buf);

    switch(priv->active_probe)
    {
        case 0:
            spin_lock(&priv->probes[0]->lock);
            priv->probes[0]->rtd_z = get_unaligned_be24(buf);
            spin_unlock(&priv->probes[0]->lock);
            priv->reg_values[REG_INPMUX] = (MX_POS(AIN5) | MX_NEG(AIN6));
            priv->reg_values[REG_IDACMUX] = (MX_IDAC1(AIN4) | MX_IDAC2(AIN7));
            priv->active_probe = 1;
            break;
        
        case 1:
            spin_lock(&priv->probes[1]->lock);
            priv->probes[1]->rtd_z = get_unaligned_be24(buf);
            spin_unlock(&priv->probes[1]->lock);
            priv->reg_values[REG_INPMUX] = (MX_POS(AIN9) | MX_NEG(AIN10));
            priv->reg_values[REG_IDACMUX] = (MX_IDAC1(AIN8) | MX_IDAC2(AIN11));
            priv->active_probe = 2;
            break;
        
        case 2:
            spin_lock(&priv->probes[2]->lock);
            priv->probes[2]->rtd_z = get_unaligned_be24(buf);
            spin_unlock(&priv->probes[2]->lock);
            priv->reg_values[REG_INPMUX] = (MX_POS(AIN1) | MX_NEG(AIN2));
            priv->reg_values[REG_IDACMUX] = (MX_IDAC1(AIN0) | MX_IDAC2(AIN3));
            priv->active_probe = 0;
            break;
    }
    
    ret = write_reg(priv->parent_dev, REG_INPMUX, &priv->reg_values[REG_INPMUX]);
    ret = write_reg(priv->parent_dev, REG_IDACMUX, &priv->reg_values[REG_IDACMUX]);

    return IRQ_HANDLED;
}

/**
 * @brief Setup up code to initialise the ADC with the default settings and check that it is functioning correctly.
 * @param spi the spi device connected the ADC
*/
static int setup_adc(struct spi_device *spi){
    int ret;
    u8 reg_check;
    int i;
    struct device *dev;
    struct nectar_temp *priv;

    dev = &spi->dev;

    priv = spi_get_drvdata(spi);

    /* Get the device id. Mask first 5 bits as these can change*/
    ret = read_reg(dev, REG_ID, &reg_check);
    priv->reg_values[REG_ID] = (0x07 & reg_check);   

    /* Send reset command */
    ret = send_command(dev, CMD_RESET);

    /* Map default values */
    for(i = 0; i < ARRAY_SIZE(priv->reg_values); i++){
        ret = read_reg(dev, i, &priv->reg_values[i]);
    }

    /* Update the private data with required settings */
    priv->reg_values[REG_INPMUX] = (MX_POS(AIN1) | MX_NEG(AIN2));
    priv->reg_values[REG_PGA] = (DELAY_14 | PGA_ENABLE | GAIN_16);
    priv->reg_values[REG_DATARATE] = (GCHOP_DISABLE | CLOCK_INTERNAL | CMODE_CONT | DF_SINC | NDR_20);
    priv->reg_values[REG_REF] = (REF_MON_DISABLE | REFP_BUF_ENABLE | REFN_BUF_ENABLE | REF_SEL0 | INTLV_ON_A);
    priv->reg_values[REG_IDACMAG] = (PGA_RAIL_FLAG_DISABLE | PSW_OPEN | IMAG_1000);
    priv->reg_values[REG_IDACMUX] = (MX_IDAC1(AIN0) | MX_IDAC2(AIN3));
    
    /* Write the settings to the adc registers */
    ret = write_reg(dev, REG_INPMUX, &priv->reg_values[REG_INPMUX]);
    ret = write_reg(dev, REG_PGA, &priv->reg_values[REG_PGA]);
    ret = write_reg(dev, REG_DATARATE, &priv->reg_values[REG_DATARATE]);
    ret = write_reg(dev, REG_REF, &priv->reg_values[REG_REF]);
    ret = write_reg(dev, REG_IDACMAG, &priv->reg_values[REG_IDACMAG]);
    ret = write_reg(dev, REG_IDACMUX, &priv->reg_values[REG_IDACMUX]);

    /* Check the registers value to make sure write was successful staring from INMUX*/
    for(i = REG_INPMUX; i < ARRAY_SIZE(priv->reg_values); i++){
        ret = read_reg(dev, i, &reg_check);
        if(reg_check != priv->reg_values[i]){
            pr_err("write error at: %x, trying again\n", i);
            //try again
            ret = write_reg(dev, i, &priv->reg_values[i]);
            ret = read_reg(dev, i, &reg_check);
            if(reg_check == priv->reg_values[i]){
                pr_info("register write successful\n");
            }
            else{
                pr_err("could not write to register: %x\n", i);
            }
        }
    }

    return 0;
}

/* Define the RTD attributes */
ssize_t rtd_impedence_show(struct device *dev, struct device_attribute *attr, char *buf){
    u32 value;
    struct temp_probe *probe;
    probe = dev_get_drvdata(dev);

    spin_lock(&probe->lock);
    value = probe->rtd_z;
    spin_unlock(&probe->lock);

    return sprintf(buf, "%x\n", value);
}
static DEVICE_ATTR_RO(rtd_impedence);

/* Define the ADC attributes */
ssize_t ref_impedence_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct temp_probe *probe = dev_get_drvdata(dev);
    struct nectar_temp *priv = probe->priv;
    u32 ref_z = priv->settings.ref_z;

    return sprintf(buf, "%u\n", ref_z);
}
static DEVICE_ATTR_RO(ref_impedence);

ssize_t pga_gain_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct temp_probe *probe = dev_get_drvdata(dev);
    struct nectar_temp *priv = probe->priv;
    u32 pga_gain = priv->settings.gain;

    return sprintf(buf, "%u\n", pga_gain);
}
static DEVICE_ATTR_RO(pga_gain);

static struct attribute *probe_attrs[] = {
    &dev_attr_rtd_impedence.attr,
    &dev_attr_ref_impedence.attr,
    &dev_attr_pga_gain.attr,
    NULL,
};

static const struct attribute_group probe_attr_group = {
    .attrs = probe_attrs,
};

static const struct attribute_group *probe_attr_groups[] = {
    &probe_attr_group,
    NULL,
};

/**
 * @brief Register an rtd device with the system
 * @param priv The private data strucutre of the driver
*/
struct temp_probe *register_probe(struct nectar_temp *priv){
    struct temp_probe *probe;
    struct device *dev;
    
    probe = devm_kzalloc(priv->parent_dev, sizeof(*probe), GFP_KERNEL);
    if(!probe){
        pr_err("failed to allocate probe memory\n");
        return ERR_PTR(-ENOMEM);
    }

    /* get probe id and increment number of probes*/
    probe->id = priv->num_probe++;

    /* get probe device number */
    probe->dev_id = MKDEV(MAJOR(priv->dev_id), (probe->id + 1));

    /* create device */
    dev = device_create(priv->dev_class, priv->parent_dev, probe->dev_id, probe, "probe%d", probe->id);
    if(IS_ERR(dev)){
        pr_err("failed to create probe device\n");
        return ERR_CAST(dev);
    }

    /* intialise device */
    spin_lock_init(&probe->lock);
    probe->priv = priv;
    probe->rtd_z = 0;

    return probe;
}

/* Create the probe and remove functions */
static int nectar_temp_probe(struct spi_device *spi){
    int ret;
    struct nectar_temp *priv;
    struct device *dev;
    int size;

    dev = &spi->dev;

    /* Allocate memory for private data strucutre */
    size = sizeof(*priv) + sizeof(struct temp_probe);
    priv = devm_kzalloc(dev, size, GFP_KERNEL);
    if(!priv){
        pr_err("failed to allocate device memory\n");
    }

    /* Allocate character driver region */
    ret = alloc_chrdev_region(&priv->dev_id, 0, 4, DEVICE_NAME);
    if(ret){
        pr_err("falied to allocate device region\n");
    }

    /* Create the class */
    priv->dev_class = class_create(THIS_MODULE, DEVICE_NAME);

    /* Set the attribute groups to the created class */
    priv->dev_class->dev_groups = probe_attr_groups;

    /* set the device inside private data */
    priv->parent_dev = dev;

    /* Setup the GPIOs */
    priv->gpio_interrupt = devm_gpiod_get_index(priv->parent_dev, GPIO_INTERRUPT, 0, GPIOD_IN);
    if(IS_ERR(priv->gpio_interrupt)){
        pr_err("failed to aquire interrupt gpio\n");
    }

    priv->gpio_reset = devm_gpiod_get_index(priv->parent_dev, GPIO_RESET, 0, GPIOD_OUT_LOW);
    if(IS_ERR(priv->gpio_reset)){
        pr_err("failed to aquire reset gpio\n");
    }

    /* Setup the IRQ */
    priv->irq = gpiod_to_irq(priv->gpio_interrupt);
    if(priv->irq < 0){
        pr_err("failed to aquire irq\n");
    }

    ret = devm_request_threaded_irq(priv->parent_dev, priv->irq, irq_handler, irq_threaded, 
                                    IRQF_TRIGGER_NONE, dev_name(dev), priv);

    /* Register a temp probe device as a character device */
    priv->probes[0] = register_probe(priv);
    priv->probes[1] = register_probe(priv);
    priv->probes[2] = register_probe(priv);

    /* Initialise ADC settings */
    priv->settings.gain = GAIN;
    priv->settings.ref_z = REF_Z;
    priv->active_probe = 0;

    /* Set the spi device private data */
    spi_set_drvdata(spi, priv);

    /* Setup the ADC with the required settings */
    ret = setup_adc(spi);

    /* Start the ADC conversions */
    ret = send_command(priv->parent_dev, CMD_START);

    return 0;
}

/* Create the remove function */
static int nectar_temp_remove(struct spi_device *spi){
    /* TODO - Add cleanup code */
    struct nectar_temp *priv = spi_get_drvdata(spi);
    
    /* Remove probe devices */
    device_destroy(priv->dev_class, priv->probes[0]->dev_id);
    device_destroy(priv->dev_class, priv->probes[1]->dev_id);
    device_destroy(priv->dev_class, priv->probes[2]->dev_id);

    /* destroy the class */
    class_destroy(priv->dev_class);

    /* unregister the character device region */
    unregister_chrdev_region(priv->dev_id, 1);
    
    return 0;
}

/* Create the compatibility string id */
static const struct of_device_id nectar_ids[] = {
    { .compatible = DEVICE_NAME, },
    { }
};
MODULE_DEVICE_TABLE(of, nectar_ids);

/* Create the spi device id strings */
static const struct spi_device_id nectar_temperature_ids[] = {
    { .name = DEVICE_NAME, },
    { }
};
MODULE_DEVICE_TABLE(spi, nectar_temperature_ids);

/* Create the device */
static struct spi_driver nectar_temperature_driver = {
    .probe = nectar_temp_probe,
    .remove = nectar_temp_remove,
    .id_table = nectar_temperature_ids,
    .driver = {
        .name = DEVICE_NAME,
        .owner = THIS_MODULE,
        .of_match_table = nectar_ids,
    },
};

static int __init nectar_temp_init(void){
    int err;
    err = spi_register_driver(&nectar_temperature_driver);
    return 0;
}

static void __exit nectar_temp_exit(void){
    spi_unregister_driver(&nectar_temperature_driver);
} 

module_init(nectar_temp_init);
module_exit(nectar_temp_exit);

MODULE_AUTHOR("Tom Norman");
MODULE_DESCRIPTION("Diver code for the temperature device");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
