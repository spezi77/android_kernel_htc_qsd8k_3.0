/* linux/arch/arm/mach-msm/board-bravo-wifi.c
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>

#include "board-bravo.h"

int bravo_wifi_power(int on);
int bravo_wifi_reset(int on);
int bravo_wifi_set_carddetect(int on);

#if defined(CONFIG_DHD_USE_STATIC_BUF) || defined(CONFIG_BCM4329_DHD_USE_STATIC_BUF)

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *bravo_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}
#endif

int __init bravo_init_wifi_mem(void)
{
#if defined(CONFIG_DHD_USE_STATIC_BUF) || defined(CONFIG_BCM4329_DHD_USE_STATIC_BUF)
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
#endif
	return 0;
}

static struct resource bravo_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= MSM_GPIO_TO_INT(BRAVO_GPIO_WIFI_IRQ),
		.end		= MSM_GPIO_TO_INT(BRAVO_GPIO_WIFI_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

#define COUNTRY_BUF_SZ	4
struct cntry_locales_custom {
	char iso_abbrev[COUNTRY_BUF_SZ];
	char custom_locale[COUNTRY_BUF_SZ];
	int custom_locale_rev;
};

/* Customized Locale table */
static struct cntry_locales_custom wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XV", 17},	/* Universal if Country code is unknown or empty */
	{"IR", "XV", 16},	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"SD", "XV", 16},	/* Universal if Country code is SUDAN */
	{"SY", "XV", 16},	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"GL", "XV", 16},	/* Universal if Country code is GREENLAND */
	{"PS", "XV", 16},	/* Universal if Country code is PALESTINIAN TERRITORY, OCCUPIED */
	{"TL", "XV", 16},	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"MH", "XV", 16},	/* Universal if Country code is MARSHALL ISLANDS */
	{"PK", "XV", 16},	/* Universal if Country code is PAKISTAN */
	{"CK", "XV", 16},	/* Universal if Country code is Cook Island (13.4.27)*/
	{"CU", "XV", 16},	/* Universal if Country code is Cuba (13.4.27)*/
	{"FK", "XV", 16},	/* Universal if Country code is Falkland Island (13.4.27)*/
	{"FO", "XV", 16},	/* Universal if Country code is Faroe Island (13.4.27)*/
	{"GI", "XV", 16},	/* Universal if Country code is Gibraltar (13.4.27)*/
	{"IM", "XV", 16},	/* Universal if Country code is Isle of Man (13.4.27)*/
	{"CI", "XV", 16},	/* Universal if Country code is Ivory Coast (13.4.27)*/
	{"JE", "XV", 16},	/* Universal if Country code is Jersey (13.4.27)*/
	{"KP", "XV", 16},	/* Universal if Country code is North Korea (13.4.27)*/
	{"FM", "XV", 16},	/* Universal if Country code is Micronesia (13.4.27)*/
	{"MM", "XV", 16},	/* Universal if Country code is Myanmar (13.4.27)*/
	{"NU", "XV", 16},	/* Universal if Country code is Niue (13.4.27)*/
	{"NF", "XV", 16},	/* Universal if Country code is Norfolk Island (13.4.27)*/
	{"PN", "XV", 16},	/* Universal if Country code is Pitcairn Islands (13.4.27)*/
	{"PM", "XV", 16},	/* Universal if Country code is Saint Pierre and Miquelon (13.4.27)*/
	{"SS", "XV", 16},	/* Universal if Country code is South_Sudan (13.4.27)*/
	{"AL", "AL", 2},
	{"DZ", "DZ", 1},
	{"AS", "AS", 12},	/* changed 2 -> 12*/
	{"AI", "AI", 1},
	{"AG", "AG", 2},
	{"AR", "AR", 21},
	{"AW", "AW", 2},
	{"AU", "AU", 6},
	{"AT", "AT", 4},
	{"AZ", "AZ", 2},
	{"BS", "BS", 2},
	{"BH", "BH", 4},	/* changed 24 -> 4*/
	{"BD", "BD", 2},
	{"BY", "BY", 3},
	{"BE", "BE", 4},
	{"BM", "BM", 12},
	{"BA", "BA", 2},
	{"BR", "BR", 4},
	{"VG", "VG", 2},
	{"BN", "BN", 4},
	{"BG", "BG", 4},
	{"KH", "KH", 2},
	{"CA", "CA", 31},
	{"KY", "KY", 3},
	{"CN", "CN", 24},
	{"CO", "CO", 17},
	{"CR", "CR", 17},
	{"HR", "HR", 4},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DK", "DK", 4},
	{"EE", "EE", 4},
	{"ET", "ET", 2},
	{"FI", "FI", 4},
	{"FR", "FR", 5},
	{"GF", "GF", 2},
	{"DE", "DE", 7},
	{"GR", "GR", 4},
	{"GD", "GD", 2},
	{"GP", "GP", 2},
	{"GU", "GU", 12},
	{"HK", "HK", 2},
	{"HU", "HU", 4},
	{"IS", "IS", 4},
	{"IN", "IN", 3},
	{"ID", "KR", 25},	/* ID/1 -> KR/24 */
	{"IE", "IE", 5},
	{"IL", "BO", 0},	/* IL/7 -> BO/0 */
	{"IT", "IT", 4},
	{"JP", "JP", 58},
	{"JO", "JO", 3},
	{"KW", "KW", 5},
	{"LA", "LA", 2},
	{"LV", "LV", 4},
	{"LB", "LB", 5},
	{"LS", "LS", 2},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"MO", "MO", 2},
	{"MK", "MK", 2},
	{"MW", "MW", 1},
	{"MY", "MY", 3},
	{"MV", "MV", 3},
	{"MT", "MT", 4},
	{"MQ", "MQ", 2},
	{"MR", "MR", 2},
	{"MU", "MU", 2},
	{"YT", "YT", 2},
	{"MX", "MX", 20},
	{"MD", "MD", 2},
	{"MC", "MC", 1},
	{"ME", "ME", 2},
	{"MA", "MA", 2},
	{"NP", "NP", 3},
	{"NL", "NL", 4},
	{"AN", "AN", 2},
	{"NZ", "NZ", 4},
	{"NO", "NO", 4},
	{"OM", "OM", 4},
	{"PA", "PA", 17},
	{"PG", "PG", 2},
	{"PY", "PY", 2},
	{"PE", "PE", 20},
	{"PH", "PH", 5},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PR", "PR", 20},
	{"RE", "RE", 2},
	{"RO", "RO", 4},
	{"SN", "SN", 2},
	{"RS", "RS", 2},
	{"SG", "SG", 4},
	{"SK", "SK", 4},
	{"SI", "SI", 4},
	{"ES", "ES", 4},
	{"LK", "LK", 1},
	{"SE", "SE", 4},
	{"CH", "CH", 4},
	{"TW", "TW", 1},
	{"TH", "TH", 5},
	{"TT", "TT", 3},
	{"TR", "TR", 7},
	{"AE", "AE", 6},
	{"UG", "UG", 2},
	{"GB", "GB", 6},
	{"UY", "UY", 1},
	{"VI", "VI", 13},
	{"VA", "VA", 2},
	{"VE", "VE", 3},
	{"VN", "VN", 4},
	{"MA", "MA", 1},
	{"ZM", "ZM", 2},
	{"EC", "EC", 21},
	{"SV", "SV", 19},
	{"KR", "KR", 57},
	{"RU", "RU", 13},
	{"UA", "UA", 8},
	{"GT", "GT", 1},
	{"MN", "MN", 1},
	{"NI", "NI", 2},
	{"US", "US", 118},
};

static void *bravo_wifi_get_country_code(char *ccode)
{
	int size, i;
	static struct cntry_locales_custom country_code;

	size = ARRAY_SIZE(wifi_translate_custom_table);

	if ((size == 0) || (ccode == NULL))
		return NULL;

	for (i = 0; i < size; i++) {
		if (!strcmp(ccode, wifi_translate_custom_table[i].iso_abbrev))
			return &wifi_translate_custom_table[i];
	}

	memset(&country_code, 0, sizeof(struct cntry_locales_custom));
	strlcpy(country_code.custom_locale, ccode, COUNTRY_BUF_SZ);

	return &country_code;
}

static struct wifi_platform_data bravo_wifi_control = {
	.set_power      = bravo_wifi_power,
	.set_reset      = bravo_wifi_reset,
	.set_carddetect = bravo_wifi_set_carddetect,
	.get_country_code = bravo_wifi_get_country_code,
#if defined(CONFIG_DHD_USE_STATIC_BUF) || defined(CONFIG_BCM4329_DHD_USE_STATIC_BUF)
	.mem_prealloc	= bravo_wifi_mem_prealloc,
#else
	.mem_prealloc   = NULL,
#endif
};

static struct platform_device bravo_wifi_device = {
        .name           = "bcmdhd_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(bravo_wifi_resources),
        .resource       = bravo_wifi_resources,
        .dev            = {
                .platform_data = &bravo_wifi_control,
        },
};

extern unsigned char *get_wifi_nvs_ram(void);
extern int wifi_calibration_size_set(void);

static unsigned bravo_wifi_update_nvs(char *str, int add_flag)
{
#define NVS_LEN_OFFSET		0x0C
#define NVS_DATA_OFFSET		0x40
	unsigned char *ptr;
	unsigned len;

	if (!str)
		return -EINVAL;
	ptr = get_wifi_nvs_ram();
	/* Size in format LE assumed */
	memcpy(&len, ptr + NVS_LEN_OFFSET, sizeof(len));
	/* if the last byte in NVRAM is 0, trim it */
	if (ptr[NVS_DATA_OFFSET + len - 1] == 0)
		len -= 1;
	if (add_flag) {
		strcpy(ptr + NVS_DATA_OFFSET + len, str);
		len += strlen(str);
	} else {
		if (strnstr(ptr + NVS_DATA_OFFSET, str, len))
			len -= strlen(str);
	}
	memcpy(ptr + NVS_LEN_OFFSET, &len, sizeof(len));
	wifi_calibration_size_set();
	return 0;
}

static int __init bravo_wifi_init(void)
{
	if (!machine_is_bravo() && !machine_is_bravoc())
		return 0;

	printk("%s: start\n", __func__);
	bravo_wifi_update_nvs("sd_oobonly=1\r\n", 0);
	bravo_wifi_update_nvs("btc_params70=0x32\r\n", 1);
	bravo_init_wifi_mem();
	return platform_device_register(&bravo_wifi_device);
}

late_initcall(bravo_wifi_init);
