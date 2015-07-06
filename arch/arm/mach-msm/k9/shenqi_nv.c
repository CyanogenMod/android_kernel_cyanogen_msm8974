
/*
 * Based on the work of Tony Sun
 * Zhenglq : Add for Get nv data from modem using SMEM.
*/

#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/proc_fs.h>
#include "smd_private.h"
#include "include/mach/proc_comm.h"

#define SMEM_ID_VENDOR_READ_NV SMEM_ID_VENDOR1
#define NV_WIFI_ADDR_SIZE	6
#define NV_BT_ADDR_SIZE		6
#define NV_MAX_SIZE		512
/* [BEGIN] guohh1 20131011 add for FACTORYDATACHECK */
//#define NV_OTHERS_SIZE		(NV_MAX_SIZE - NV_WIFI_ADDR_SIZE - NV_BT_ADDR_SIZE)
#define NV_OTHERS_SIZE   (NV_MAX_SIZE - NV_WIFI_ADDR_SIZE - NV_BT_ADDR_SIZE-32-32-16-16-16 -16 - 32 - 8-8-100)
/* [END   ] guohh1 20131011 add for FACTORYDATACHECK*/

struct smem_nv {
	unsigned char nv_wifi[NV_WIFI_ADDR_SIZE];
	unsigned char nv_bt[NV_BT_ADDR_SIZE];
       /* [BEGIN] guohh1 20131011 add for FACTORYDATACHECK */
       unsigned char nv_sn1[32];
       unsigned char nv_sn2[32];
       unsigned char nv_meid[16];
       unsigned char nv_imei1[16];
       unsigned char nv_imei2[16];
       unsigned char nv_hwid[16];
       unsigned char nv_station[32];
       /* [END   ] guohh1 20131011 add for FACTORYDATACHECK*/
/* [BEGIN][PLAT-59][MODEM][cuigq1][20150518] add global mode switch */
       unsigned char nv_globalmode_status[8];
/* [END][PLAT-59][MODEM][cuigq1][20150518] add global mode switch */
/* [BEGIN][PLAT-59][MODEM][cuigq1][20150520] set UE Mode by EFS */
       unsigned char nv_uemode[8];
/* [END][PLAT-59][MODEM][cuigq1][20150520] set UE Mode by EFS */
/* [BEGIN][PLAT-66][MODEM][guohh11][20150610] read NV2498 and set proc */
       unsigned char nv_2498[100];
/* [END][PLAT-66][MODEM][guohh11][20150610] read NV2498 and set proc */
	unsigned char nv_others[NV_OTHERS_SIZE];	
};
static struct smem_nv * psmem_nv = NULL;



static struct smem_nv * smem_read_nv(void)
{
	struct smem_nv * buf;
	buf = smem_alloc(SMEM_ID_VENDOR_READ_NV, NV_MAX_SIZE);
	if(!buf) 
		printk(KERN_ERR "SMEM_ID_VENDOR_READ_NV smem_alloc failed\n");
	return buf;	
}

static int dump_wifi_addr(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
	int len = 0;
	if (!psmem_nv)
		psmem_nv = smem_read_nv();
	if (!psmem_nv) 
		return 0;
	
	printk(KERN_ERR "wifi addr  = 0x %02x %02x %02x %02x %02x %02x\n",
		psmem_nv->nv_wifi[0],psmem_nv->nv_wifi[1],psmem_nv->nv_wifi[2],
		psmem_nv->nv_wifi[3],psmem_nv->nv_wifi[4],psmem_nv->nv_wifi[5]);
        memcpy( buf, psmem_nv->nv_wifi, NV_WIFI_ADDR_SIZE);
        len = NV_WIFI_ADDR_SIZE;
	*eof = 1;
	return len;
}

int wlan_get_nv_mac(char* buf)
{
	int ret = -1;
        if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}

	if (!psmem_nv){
		printk(KERN_ERR "Could not get smem for wlan mac nv\n");
	        return ret;
	}

        printk(KERN_ERR "wifi addr  = 0x %02x %02x %02x %02x %02x %02x\n",
                psmem_nv->nv_wifi[0],psmem_nv->nv_wifi[1],psmem_nv->nv_wifi[2],
                psmem_nv->nv_wifi[3],psmem_nv->nv_wifi[4],psmem_nv->nv_wifi[5]);
        memcpy( buf, psmem_nv->nv_wifi, NV_WIFI_ADDR_SIZE);
	return 0;
}
EXPORT_SYMBOL_GPL(wlan_get_nv_mac);

static int dump_bt_addr(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
	int len = 0;
	if (!psmem_nv) 
		psmem_nv = smem_read_nv();
	if (!psmem_nv)
		return 0;
	
	printk(KERN_ERR "bt addr  = 0x %02x %02x %02x %02x %02x %02x\n",
		psmem_nv->nv_bt[0],psmem_nv->nv_bt[1],psmem_nv->nv_bt[2],
		psmem_nv->nv_bt[3],psmem_nv->nv_bt[4],psmem_nv->nv_bt[5]);
	memcpy( buf, psmem_nv->nv_bt, NV_BT_ADDR_SIZE);
	len = NV_BT_ADDR_SIZE;
	*eof = 1;
	return len;
}

/* [BEGIN] guohh1 20131011 add for FACTORYDATACHECK */
static int dump_lnv_sn1(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;
    memcpy( buf, psmem_nv->nv_sn1, 32);
    len = strlen(psmem_nv->nv_sn1);
    *eof = 1;
    return len;
}
static int dump_lnv_sn2(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;
    memcpy( buf, psmem_nv->nv_sn2, 32);
    len = strlen(psmem_nv->nv_sn2);
    *eof = 1;
    return len;
}
static int dump_lnv_meid(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;

    memcpy( buf, psmem_nv->nv_meid, 16);
    len = strlen(psmem_nv->nv_meid);
    *eof = 1;
    return len;
}
static int dump_lnv_imei1(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;

    memcpy( buf, psmem_nv->nv_imei1, 16);
    len = strlen(psmem_nv->nv_imei1);
    *eof = 1;
    return len;
}
static int dump_lnv_imei2(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;

    memcpy( buf, psmem_nv->nv_imei2, 16);
    len = strlen(psmem_nv->nv_imei2);
    *eof = 1;
    return len;
}

static int dump_lnv_hwid(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;
    memcpy( buf, psmem_nv->nv_hwid, 16);
    len = strlen(psmem_nv->nv_hwid);
    *eof = 1;
    return len;
}
static int dump_lnv_station(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;
    memcpy( buf, psmem_nv->nv_station, 32);
    len = strlen(psmem_nv->nv_station);
    *eof = 1;
    return len;
}
/* [END   ] guohh1 20131011 add for FACTORYDATACHECK*/
/* [BEGIN][PLAT-59][MODEM][cuigq1][20150518] add global mode switch */
static int dump_lnv_globalmode_status(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;
    memcpy( buf, psmem_nv->nv_globalmode_status, 8);
    len = strlen(psmem_nv->nv_globalmode_status);
    *eof = 1;
    return len;
}
/* [END][PLAT-59][MODEM][cuigq1][20150518] add global mode switch */
/* [BEGIN][PLAT-59][MODEM][cuigq1][20150520] set UE Mode by EFS */
static int dump_lnv_uemode(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;
    memcpy( buf, psmem_nv->nv_uemode, 8);
    len = strlen(psmem_nv->nv_uemode);
    *eof = 1;
    return len;
}
/* [END][PLAT-59][MODEM][cuigq1][20150520] set UE Mode by EFS */
/* [BEGIN][PLAT-66][MODEM][guohh11][20150610] read NV2498 and set proc */
static int dump_lnv_nv2498(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
    int len = 0;
    if (!psmem_nv)
        psmem_nv = smem_read_nv();
    if (!psmem_nv)
        return 0;
    memcpy( buf, psmem_nv->nv_2498, 100);
    len = strlen(psmem_nv->nv_2498);
    *eof = 1;
    return len;
}
/* [END][PLAT-66][MODEM][guohh11][20150610] read NV2498 and set proc */
static void show_nv(void)
{
	struct proc_dir_entry *wifi_addr_entry;
	struct proc_dir_entry *bt_addr_entry;
       /* [BEGIN] guohh1 20131011 add for FACTORYDATACHECK */
       struct proc_dir_entry *sn1_addr_entry;
       struct proc_dir_entry *sn2_addr_entry;
       struct proc_dir_entry *meid_addr_entry;
       struct proc_dir_entry *imei1_addr_entry;
       struct proc_dir_entry *imei2_addr_entry;
       struct proc_dir_entry *hwid_addr_entry;
       struct proc_dir_entry *station_addr_entry;
       /* [END   ] guohh1 20131011 add for FACTORYDATACHECK*/
/* [BEGIN][PLAT-59][MODEM][cuigq1][20150518] add global mode switch */
       struct proc_dir_entry *globalmode_status_addr_entry;
/* [END][PLAT-59][MODEM][cuigq1][20150518] add global mode switch */
/* [BEGIN][PLAT-59][MODEM][cuigq1][20150520] set UE Mode by EFS */
       struct proc_dir_entry *uemode_addr_entry;
/* [END][PLAT-59][MODEM][cuigq1][20150520] set UE Mode by EFS */

/* [BEGIN][PLAT-66][MODEM][guohh11][20150610] read NV2498 and set proc */
       struct proc_dir_entry *nv2498_addr_entry;
/* [END][PLAT-66][MODEM][guohh11][20150610] read NV2498 and set proc */
	wifi_addr_entry = create_proc_entry("mac_wifi", 0, NULL);
	bt_addr_entry = create_proc_entry("mac_bt", 0, NULL);

	if (wifi_addr_entry)
		wifi_addr_entry ->read_proc = &dump_wifi_addr;

	if (bt_addr_entry)
		bt_addr_entry ->read_proc = &dump_bt_addr;
       /* [BEGIN] guohh1 20131011 add for FACTORYDATACHECK */
       sn1_addr_entry = create_proc_entry("lnvsn1", 0, NULL);
       if (sn1_addr_entry)
           sn1_addr_entry ->read_proc = &dump_lnv_sn1;
       sn2_addr_entry = create_proc_entry("lnvshowcode", 0, NULL);
       if (sn2_addr_entry)
           sn2_addr_entry ->read_proc = &dump_lnv_sn2;
       meid_addr_entry = create_proc_entry("lnvmeid", 0, NULL);
       if (meid_addr_entry)
           meid_addr_entry ->read_proc = &dump_lnv_meid;
       imei1_addr_entry = create_proc_entry("lnvimei1", 0, NULL);
       if (imei1_addr_entry)
           imei1_addr_entry ->read_proc = &dump_lnv_imei1;
       imei2_addr_entry = create_proc_entry("lnvimei2", 0, NULL);
       if (imei2_addr_entry)
           imei2_addr_entry ->read_proc = &dump_lnv_imei2;
       hwid_addr_entry = create_proc_entry("lnvhwid", 0, NULL);
       if (hwid_addr_entry)
       {
           hwid_addr_entry ->read_proc = &dump_lnv_hwid;
       }
       station_addr_entry = create_proc_entry("lnvstation", 0, NULL);
       if (station_addr_entry)
       {
           station_addr_entry ->read_proc = &dump_lnv_station;
       }
      /* [END   ] guohh1 20131011 add for FACTORYDATACHECK*/
/* [BEGIN][PLAT-59][MODEM][cuigq1][20150518] add global mode switch */
       globalmode_status_addr_entry = create_proc_entry("lnvglobalmodestatus", 0, NULL);
       if (globalmode_status_addr_entry)
       {
           globalmode_status_addr_entry ->read_proc = &dump_lnv_globalmode_status;
       }
/* [END][PLAT-59][MODEM][cuigq1][20150518] add global mode switch */
/* [BEGIN][PLAT-59][MODEM][cuigq1][20150520] set UE Mode by EFS */
       uemode_addr_entry = create_proc_entry("lnvuemode", 0, NULL);
       if (uemode_addr_entry)
       {
           uemode_addr_entry ->read_proc = &dump_lnv_uemode;
       }
/* [END][PLAT-59][MODEM][cuigq1][20150520] set UE Mode by EFS */
/* [BEGIN][PLAT-66][MODEM][guohh11][20150610] read NV2498 and set proc */
       nv2498_addr_entry = create_proc_entry("lnv2498", 0, NULL);
       if (nv2498_addr_entry)
       {
           nv2498_addr_entry ->read_proc = &dump_lnv_nv2498;
       }
/* [END][PLAT-66][MODEM][guohh11][20150610] read NV2498 and set proc */
}

void __init shenqi_nv_init(void)
{
	show_nv();	
}

