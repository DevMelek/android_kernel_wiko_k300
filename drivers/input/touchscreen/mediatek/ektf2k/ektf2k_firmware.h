#ifdef IAP_PORTION

 u8 yijian_fw[]=
{

#include "Tinno_yijian_K600_2BB4_C608.i"

};

struct vendor_map
{
	int vendor_id;
	char vendor_name[30];
	uint8_t* fw_array;
};
 struct vendor_map g_vendor_map[]=
{
       {0x2BB5,"DIJING", NULL},
};

#endif/*IAP_PORTION*/
