/////////////////////////////////////////////////////////////////////////////
// File Name	: OIS_user.c
// Function		: User defined function.
// 				  These functions depend on user's circumstance.

// Rule         : Use TAB 4

// Copyright(c)	Rohm Co.,Ltd. All rights reserved

/***** ROHM Confidential ***************************************************/
#ifndef OIS_USER_C
#define OIS_USER_C
#endif
/*add for msm platform*/
#include <linux/debugfs.h>
#include <linux/module.h>
#include "msm_sd.h"
#include "msm_actuator.h"
#include "msm_cci.h"
#include "../eeprom/msm_eeprom.h"
//#include <stdio.h>
#include "OIS_head.h"
//#include "usb_func.h"
//#include "winbase.h"

extern struct msm_actuator_ctrl_t * actuator_ctrl;


// Following Variables that depend on user's environment			RHM_HT 2013.03.13	add
OIS_UWORD			FOCUS_VAL	= 0x0122;				// Focus Value

// <== RHM_HT 2013/07/10	Added new user definition variables


 _FACT_ADJ	FADJ_MEM

= {
	0x1cf,	// gl_CURDAT;
	0x1be,	// gl_HALOFS_X;
	0x1eb,	// gl_HALOFS_Y;
	0x36,	// gl_HX_OFS;
	0xfff2,	// gl_HY_OFS;
	0x80,	// gl_PSTXOF;		RHM_HT 2013.03.21	Change order to adjust EEP ROM map
	0x81,	// gl_PSTYOF;		RHM_HT 2013.03.21	Change order to adjust EEP ROM map
	0xf5,	// gl_GX_OFS;
	0x31a,	// gl_GY_OFS;

	0xdb44,	// gl_KgxHG ;		RHM_HT 2013/11/25	Modified
	0xde35,	// gl_KgyHG ;		RHM_HT 2013/11/25	Modified
	0x2eca,	// gl_KGXG  ;		RHM_HT 2013/11/25	Modified
	0x29c8,	// gl_KGYG  ;		RHM_HT 2013/11/25	Modified
	0x1be,	// gl_SFTHAL_X;		RHM_HT 2013/11/25	Added
	0x1eb,	// gl_SFTHAL_Y;		RHM_HT 2013/11/25	Added
	0x0000,	// gl_TMP_X_;		RHM_HT 2013/11/25	Added
	0x0000,	// gl_TMP_Y_;		RHM_HT 2013/11/25	Added
	0xff95,	// gl_KgxH0;		RHM_HT 2013/11/25	Added
	0x32,	// gl_KgyH0;		RHM_HT 2013/11/25	Added
};

// *********************************************************
// Disable/Enable Power Save Mode (CLK_PS port = L->H/H->L)
// and VCO setting
// ---------------------------------------------------------
// <Function>
//		Disable or Enable Power Save Mode of OIS controller chip.
//		This function relate to your own circuit.
//
// <Input>
//		None
//
// <Output>
//		None
//
// <Description>
//
// *********************************************************
void	POWER_UP_AND_PS_DISABLE( void )
{
	/* Please write your source code according to following instructions: */
	// Instractions:
	// 		CLK_PS = L
	// 		1ms wait
	// 		CLK_PS = External clock

	{
		// Sample Code
#if 0
		PortCtrl( _picsig_PS____, _PIO_O_L_ );					// Disable external clock
		Wait(1);												// 1ms wait
		PortCtrl( _picsig_PS____, _PIO_O_H_ );					// Enable external clock
#endif
	}
}
void	POWER_DOWN_AND_PS_ENABLE( void )
{
	/* Please write your source code according to following instructions: */
	// Instractions:
	// 		CLK_PS = L
	// 		1ms wait

	{
		// Sample Code
#if 0
		PortCtrl( _picsig_PS____, _PIO_O_L_ );					// CLK_PS = L
		Wait(1);												// 1ms wait
#endif
	}
}	// 		Depend on your system


// /////////////////////////////////////////////////////////
// VCOSET function
// ---------------------------------------------------------
// <Function>
//		To use external clock at CLK/PS, it need to set PLL.
//		After enabling PLL, more than 30ms wait time is required to change clock source.
//		So the below sequence has to be used:
// 		Input CLK/PS --> Call VCOSET0 --> Download Program/Coed --> Call VCOSET1
//
// <Input>
//		none
//
// <Output>
//		none
//
// =========================================================
void	VCOSET0( void )
{

	OIS_UWORD 	CLK_PS = 24000;            					// X5=24MHz //Input Frequency [kHz] of CLK/PS terminal (Depend on your system)
	OIS_UWORD 	FVCO_1 = 27000;                					// Target Frequency [kHz]
	OIS_UWORD 	FREF   = 25;             						// Reference Clock Frequency [kHz]

	OIS_UWORD	DIV_N  = CLK_PS / FREF - 1;         			// calc DIV_N
	OIS_UWORD	DIV_M  = FVCO_1 / FREF - 1;         			// calc DIV_M

	I2C_OIS_per_write( 0x62, DIV_N  );				// Divider for internal reference clock
	I2C_OIS_per_write( 0x63, DIV_M  ); 							// Divider for internal PLL clock
	I2C_OIS_per_write( 0x64, 0x4060 );					// Loop Filter

	I2C_OIS_per_write( 0x60, 0x3011 ); 							// PLL
	I2C_OIS_per_write( 0x65, 0x0080 ); 							//
	I2C_OIS_per_write( 0x61, 0x8002 ); 							// VCOON
	I2C_OIS_per_write( 0x61, 0x8003 ); 							// Circuit ON
	I2C_OIS_per_write( 0x61, 0x8809 ); 							// PLL ON

}

int	FadjDat_Lenovo( uint8_t * ois_data)
{

	pr_err("FadjDat_Lenovo  data = 0x%x   ois_data=%p \n",( unsigned short )*(ois_data),ois_data);

	if (ois_data)
	{
		FADJ_MEM.gl_CURDAT = ( unsigned short )((*(ois_data+0))|(*(ois_data+0+1)<<8));
		pr_err("ois gl_CURDAT = 0x%x\n",FADJ_MEM.gl_CURDAT);
		FADJ_MEM.gl_HALOFS_X =  ( unsigned short )((*(ois_data+2))|(*(ois_data+2+1)<<8));
		pr_err("ois gl_HALOFS_X = 0x%x\n",FADJ_MEM.gl_HALOFS_X);
		FADJ_MEM.gl_HALOFS_Y =  ( unsigned short )((*(ois_data+4))|(*(ois_data+4+1)<<8));
		pr_err("ois gl_HALOFS_Y = 0x%x\n",FADJ_MEM.gl_HALOFS_Y);
		FADJ_MEM.gl_HX_OFS =  ( unsigned short )((*(ois_data+6))|(*(ois_data+6+1)<<8));
		pr_err("ois gl_HX_OFS = 0x%x\n",FADJ_MEM.gl_HX_OFS);
		FADJ_MEM.gl_HY_OFS =  ( unsigned short )((*(ois_data+8))|(*(ois_data+8+1)<<8));
		pr_err("ois gl_HY_OFS = 0x%x\n",FADJ_MEM.gl_HY_OFS);
		FADJ_MEM.gl_PSTXOF =  ( unsigned short )((*(ois_data+10))|(*(ois_data+10+1)<<8));
		pr_err("ois gl_PSTXOF = 0x%x\n",FADJ_MEM.gl_PSTXOF);
		FADJ_MEM.gl_PSTYOF =  ( unsigned short )((*(ois_data+12))|(*(ois_data+12+1)<<8));
		pr_err("ois gl_PSTYOF = 0x%x\n",FADJ_MEM.gl_PSTYOF);
		FADJ_MEM.gl_GX_OFS = ( unsigned short )((*(ois_data+14))|(*(ois_data+14+1)<<8));
		pr_err("ois gl_GX_OFS = 0x%x\n",FADJ_MEM.gl_GX_OFS);
		FADJ_MEM.gl_GY_OFS = ( unsigned short )((*(ois_data+16))|(*(ois_data+16+1)<<8));
		pr_err("ois gl_GY_OFS = 0x%x\n",FADJ_MEM.gl_GY_OFS);
		FADJ_MEM.gl_KgxHG = ( unsigned short )((*(ois_data+18))|(*(ois_data+18+1)<<8));
		pr_err("ois gl_KgxHG = 0x%x\n",FADJ_MEM.gl_KgxHG);
		FADJ_MEM.gl_KgyHG = ( unsigned short )((*(ois_data+20))|(*(ois_data+20+1)<<8));
		pr_err("ois gl_KgyHG = 0x%x\n",FADJ_MEM.gl_KgyHG);
		FADJ_MEM.gl_KGXG = ( unsigned short )((*(ois_data+22))|(*(ois_data+22+1)<<8));
		pr_err("ois gl_KGXG = 0x%x\n",FADJ_MEM.gl_KGXG);
		FADJ_MEM.gl_KGYG = ( unsigned short )((*(ois_data+24))|(*(ois_data+24+1)<<8));
		pr_err("ois gl_KGYG = 0x%x\n",FADJ_MEM.gl_KGYG);
		FADJ_MEM.gl_SFTHAL_X = ( unsigned short )((*(ois_data+26))|(*(ois_data+26+1)<<8));
		pr_err("ois gl_SFTHAL_X = 0x%x\n",FADJ_MEM.gl_SFTHAL_X);
		FADJ_MEM.gl_SFTHAL_Y = ( unsigned short )((*(ois_data+28))|(*(ois_data+28+1)<<8));
		pr_err("ois gl_SFTHAL_Y = 0x%x\n",FADJ_MEM.gl_SFTHAL_Y);
		FADJ_MEM.gl_TMP_X_ = ( unsigned short )((*(ois_data+30))|(*(ois_data+30+1)<<8));
		pr_err("ois gl_TMP_X_ = 0x%x\n",FADJ_MEM.gl_TMP_X_);
		FADJ_MEM.gl_TMP_Y_ = ( unsigned short )((*(ois_data+32))|(*(ois_data+32+1)<<8));
		pr_err("ois gl_TMP_Y_ = 0x%x\n",FADJ_MEM.gl_TMP_Y_);
		FADJ_MEM.gl_KgxH0 = ( unsigned short )((*(ois_data+34))|(*(ois_data+34+1)<<8));
		pr_err("ois gl_KgxH0 = 0x%x\n",FADJ_MEM.gl_KgxH0);
		FADJ_MEM.gl_KgyH0 = ( unsigned short )((*(ois_data+36))|(*(ois_data+36+1)<<8));
		pr_err("ois gl_KgyH0 = 0x%x\n",FADJ_MEM.gl_KgyH0);
	}
	return 0;
}
ADJ_STS	rh63163_init( void )
{
	ADJ_STS		OIS_MAIN_STS  = ADJ_ERR;		// Status register of this main routine.	RHM_HT 2013/04/15	Change "typedef" and initial value.
	int rc = 0;

	VCOSET0();
	rc = func_PROGRAM_DOWNLOAD( );
	if (rc < 0)
		pr_err("func_PROGRAM_DOWNLOAD failed\n");
	func_COEF_DOWNLOAD( 0 );
	VCOSET1();
	I2C_OIS_spcl_cmnd( 1, _cmd_8C_EI );
	rc = SET_FADJ_PARAM(&FADJ_MEM);

	rh63163_ois_enable( 1 );	// Set default SCENE ( Just example )
	DEBUG_printf(("gl_CURDAT = 0x%04X\n", FADJ_MEM.gl_CURDAT));
	DEBUG_printf(("gl_HALOFS_X = 0x%04X\n", FADJ_MEM.gl_HALOFS_X));
	DEBUG_printf(("gl_HALOFS_Y = 0x%04X\n", FADJ_MEM.gl_HALOFS_Y));
	DEBUG_printf(("gl_PSTXOF = 0x%04X\n", FADJ_MEM.gl_PSTXOF));
	DEBUG_printf(("gl_PSTYOF = 0x%04X\n", FADJ_MEM.gl_PSTYOF));
	DEBUG_printf(("gl_HX_OFS = 0x%04X\n", FADJ_MEM.gl_HX_OFS));
	DEBUG_printf(("gl_HY_OFS = 0x%04X\n", FADJ_MEM.gl_HY_OFS));
	DEBUG_printf(("gl_GX_OFS = 0x%04X\n", FADJ_MEM.gl_GX_OFS));
	DEBUG_printf(("gl_GY_OFS = 0x%04X\n", FADJ_MEM.gl_GY_OFS));
	DEBUG_printf(("gl_KgxHG  = 0x%04X\n", FADJ_MEM.gl_KgxHG));
	DEBUG_printf(("gl_KgyHG  = 0x%04X\n", FADJ_MEM.gl_KgyHG));
	DEBUG_printf(("gl_KGXG   = 0x%04X\n", FADJ_MEM.gl_KGXG));
	DEBUG_printf(("gl_KGYG   = 0x%04X\n", FADJ_MEM.gl_KGYG));
	DEBUG_printf(("gl_SFTHAL_X = 0x%04X\n", FADJ_MEM.gl_SFTHAL_X));
	DEBUG_printf(("gl_SFTHAL_Y = 0x%04X\n", FADJ_MEM.gl_SFTHAL_Y));
	DEBUG_printf(("gl_TMP_X_ = 0x%04X\n", FADJ_MEM.gl_TMP_X_));		// RHM_HT 2013/05/23	Added
	DEBUG_printf(("gl_TMP_Y_ = 0x%04X\n", FADJ_MEM.gl_TMP_Y_));		// RHM_HT 2013/05/23	Added
	DEBUG_printf(("gl_KgxH0 = 0x%04X\n", FADJ_MEM.gl_KgxH0));		// RHM_HT 2013/05/23	Added
	DEBUG_printf(("gl_KgyH0 = 0x%04X\n", FADJ_MEM.gl_KgyH0));		// RHM_HT 2013/05/23	Added
	OIS_MAIN_STS  = ADJ_OK;

	return OIS_MAIN_STS;

}

void	rh63163_ois_enable( uint8_t ois_enable )
{
	if(ois_enable) {
		func_SET_SCENE_PARAM_for_NewGYRO_Fil( _SCENE_SPORT_3, 1, 0, 1, &FADJ_MEM );	// Set default SCENE ( Just example )
		pr_err("rh63163_ois_enable\n");
	} else {
		func_SET_SCENE_PARAM_for_NewGYRO_Fil( _SCENE_SPORT_3, 0, 0, 1, &FADJ_MEM ); // Change SCENE parameter OIS-ON
		pr_err("rh63163_ois_disable\n");
	}
}

void	VCOSET1( void )
{

//     OIS_UWORD 	CLK_PS = 6750;            						// Input Frequency [kHz] of CLK/PS terminal (Depend on your system)	RHM_HT 2013.05.09	Change 12M -> 6.75M
//     OIS_UWORD 	FVCO_1 = 27000;                					// Target Frequency [kHz]
//     OIS_UWORD 	FREF   = 25;             						// Reference Clock Frequency [kHz]

//     OIS_UWORD	DIV_N  = CLK_PS / FREF - 1;         			// calc DIV_N
//     OIS_UWORD	DIV_M  = FVCO_1 / FREF - 1;         			// calc DIV_M

//     I2C_OIS_per_write( 0x62, DIV_N  ); 							// Divider for internal reference clock
//     I2C_OIS_per_write( 0x63, DIV_M  ); 							// Divider for internal PLL clock
//     I2C_OIS_per_write( 0x64, 0x4060 ); 							// Loop Filter

//     I2C_OIS_per_write( 0x60, 0x3011 ); 							// PLL
//     I2C_OIS_per_write( 0x65, 0x0080 ); 							//
//     I2C_OIS_per_write( 0x61, 0x8002 ); 							// VCOON
//     I2C_OIS_per_write( 0x61, 0x8003 ); 							// Circuit ON
//     I2C_OIS_per_write( 0x61, 0x8809 ); 							// PLL ON
// 
//     Wait( 30 );                  								// Wait for PLL lock

    I2C_OIS_per_write( 0x05, 0x000C ); 							// Prepare for PLL clock as master clock
    I2C_OIS_per_write( 0x05, 0x000D ); 							// Change to PLL clock
}


// /////////////////////////////////////////////////////////
// Write Data to Slave device via I2C master device
// ---------------------------------------------------------
// <Function>
//		I2C master send these data to the I2C slave device.
//		This function relate to your own circuit.
//
// <Input>
//		OIS_UBYTE	slvadr	I2C slave adr
//		OIS_UBYTE	size	Transfer Size
//		OIS_UBYTE	*dat	data matrix
//
// <Output>
//		none
//
// <Description>
//		[S][SlaveAdr][W]+[dat[0]]+...+[dat[size-1]][P]

// =========================================================
int	WR_I2C( OIS_UBYTE slvadr, OIS_UWORD size, OIS_UBYTE *dat )
{
	int rc = -1;
	actuator_ctrl->i2c_client.addr_type = 1;//MSM_CAMERA_I2C_WORD_BYTE

	actuator_ctrl->i2c_data_type = 1;
	//DEBUG_printf(("WR_I2C reg:%02x,size:%d ",dat[0],size-1));
	//DEBUG_printf(("zrb:sid:%x",actuator_ctrl->i2c_client.cci_client->sid));
#if  0
	if(size == 5)
	DEBUG_printf(("call WR_I2C (0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X,0x%02X,0x%02X)", slvadr, size, dat[0], dat[1], dat[2], dat[3], dat[4]));
	else
	DEBUG_printf(("call WR_I2C (0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X,0x%02X)", slvadr, size, dat[0], dat[1], dat[2], dat[3]));
#endif
	rc = actuator_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(&actuator_ctrl->i2c_client,dat[0],&dat[1],size-1);
	return rc;

	//actuator_ctrl->i2c_client.addr_type = 2;//MSM_CAMERA_I2C_WORD_BYTE

	//actuator_ctrl->i2c_data_type = 2;
	//DEBUG_printf(("zrb:sid:%x\n",actuator_ctrl->i2c_client.cci_client->sid));
	//actuator_ctrl->i2c_client.i2c_func_tbl->i2c_write(&actuator_ctrl->i2c_client,dat[0]<<8 | dat[1],dat[2]<<8 | dat[3],2);

}


// *********************************************************
// Read Data from Slave device via I2C master device
// ---------------------------------------------------------
// <Function>
//		I2C master read data from the I2C slave device.
//		This function relate to your own circuit.
//
// <Input>
//		OIS_UBYTE	slvadr	I2C slave adr
//		OIS_UBYTE	size	Transfer Size
//		OIS_UBYTE	*dat	data matrix
//
// <Output>
//		OIS_UWORD	16bit data read from I2C Slave device
//
// <Description>
//	if size == 1
//		[S][SlaveAdr][W]+[dat[0]]+         [RS][SlaveAdr][R]+[RD_DAT0]+[RD_DAT1][P]
//	if size == 2
//		[S][SlaveAdr][W]+[dat[0]]+[dat[1]]+[RS][SlaveAdr][R]+[RD_DAT0]+[RD_DAT1][P]
//
// *********************************************************
OIS_UWORD	RD_I2C( OIS_UBYTE slvadr, OIS_UBYTE size, OIS_UBYTE *dat )
{
	OIS_UWORD	read_data = 0;
	OIS_ULONG addr = 0;
	OIS_UBYTE ret[2] = {0, 0};
	int rc = 0;

	/* Please write your source code here. */

	{
		// Sample Code
		//DEBUG_printf(("call RD_I2C (0x%02X, 0x%02X, 0x%02X, 0x%02X)", slvadr, size, dat[0], dat[1]));
		actuator_ctrl->i2c_client.addr_type = size;//MSM_CAMERA_I2C_WORD_ADDR
		if(size == 1){
			addr = dat[0];
		}else if(size == 2){
			addr = dat[0] << 8 |dat[1];
		}
		 rc = actuator_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
											&actuator_ctrl->i2c_client,
											(unsigned long)addr,
											(unsigned char *)ret,
											2);

		read_data = ret[0] << 8 | ret[1];
		//DEBUG_printf(("return 0x%04X\n", read_data));
	}

	return read_data;
}


// *********************************************************
// Write Factory Adjusted data to the non-volatile memory
// ---------------------------------------------------------
// <Function>
//		Factory adjusted data are sotred somewhere
//		non-volatile memory.
//
// <Input>
//		_FACT_ADJ	Factory Adjusted data
//
// <Output>
//		none
//
// <Description>
//		You have to port your own system.
//
// *********************************************************
void	store_FADJ_MEM_to_non_volatile_memory( _FACT_ADJ param )
{
	/*	Write to the non-vollatile memory such as EEPROM or internal of the CMOS sensor*/
}


;

// *********************************************************
// Read Factory Adjusted data from the non-volatile memory
// ---------------------------------------------------------
// <Function>
//		Factory adjusted data are sotred somewhere
//		non-volatile memory.  I2C master has to read these
//		data and store the data to the OIS controller.
//
// <Input>
//		none
//
// <Output>
//		_FACT_ADJ	Factory Adjusted data
//
// <Description>
//		You have to port your own system.
//
// *********************************************************
_FACT_ADJ	get_FADJ_MEM_from_non_volatile_memory( void )
{
	/* 	Read from the non-vollatile memory such as EEPROM or internal of the CMOS sensor... */
	
	return FADJ_MEM;		// Note: This return data is for DEBUG.
}


// ==> RHM_HT 2013/04/15	Add for DEBUG
// *********************************************************
// Printf for DEBUG
// ---------------------------------------------------------
// <Function>
//
// <Input>
//		const char *format,
// 				Same as printf
//
// <Output>
//		none
//
// <Description>
//
// *********************************************************
int debug_print(const char *format, ...)
{

	char str[512];
	int r;
	va_list va;

	int length = (int)strlen(format);

	if( length >= 512 ){
		pr_err("length of %s: %d\n", format, length);
		return -1;
	}

	va_start(va, format);
	r = vsprintf(str, format, va);
	va_end(va);

	pr_err("%s\n",str);
	return r;
}
// <== RHM_HT 2013/04/15	Add for DEBUG
