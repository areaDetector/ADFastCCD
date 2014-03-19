#ifndef CIN_REGISTER_MAP_H
#define CIN_REGISTER_MAP_H

/* not strictly necessary if file contains only macro defs */
#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
*		CIN Registers
* ============================================================================
* -----------------------------------< Configuration FPGA Registers >
*/

/* Command Registers */
#define REG_COMMAND 				0x0001
#define REG_READ_ADDRESS 			0x0002
#define REG_STREAM_TYPE 			0x0003

/* FCLK Values */
#define CMD_FCLK_125          0xB000
#define CMD_FCLK_200          0x7000
#define CMD_FCLK_250          0x3000

/* Ethernet Interface */
#define REG_IF_MAC0         			0x0010
#define REG_IF_MAC1         			0x0011
#define REG_IF_MAC2         			0x0012
#define REG_IF_IP0          			0x0013
#define REG_IF_IP1	         		0x0014
#define REG_IF_CMD_PORT_NUM    			0x001A
#define REG_IF_STREAM_IN_PORT_NUM		0x001C
#define REG_IF_STREAM_OUT_PORT_NUM		0x001D

#define REG_ETH_RESET 				0x0020 /* Reset Eth Hardware 1=Rx, 2=Tx, 3=Both */
#define REG_ETH_ENABLE 				0x0021 /* Enable Eth Hardware 1=Rx, 2=Tx, 3=Both */
#define REG_PHY1_MDIO_CMD			0x0022 /* Start(1), RnW(1), WDRd(1), PHY Addr(5), REG Addr(5) */
#define REG_PHY1_MDIO_CMD_DATA			0x0023
#define REG_PHY1_MDIO_STATUS			0x0024
#define REG_PHY1_MDIO_RD_ADDR			0x0025
#define REG_PHY1_MDIO_RD_DATA			0x0026
#define REG_MAC_CFG_VECTOR1 			0x0027 /* Ethernet Hardware Conf */
#define REG_PHY2_MDIO_CMD			0x0028
#define REG_PHY2_MDIO_CMD_DATA			0x0029
#define REG_PHY2_MDIO_STATUS			0x002A
#define REG_PHY2_MDIO_RD_ADDR			0x002B
#define REG_PHY2_MDIO_RD_DATA			0x002C
#define REG_MAC_CFG_VECTOR2 			0x002D /* Ethernet Hardware Conf */

/* Power Supply Control */
#define CMD_PS_ENABLE 				0x0021 /* Enable Selected Power Modules */
#define CMD_PS_POWERDOWN 			0x0022 /* Start power down sequence */

#define REG_PS_ENABLE    			0x0030 /* Power Supply Enable: */

/*
 b(0): 12V power bus enable
 b(1): 2.5v general enable
 b(2): 3.3v general enable
 b(3): 0.9v Frame FPGA enable
 b(4): 2.5v Frame FPGA enable
 b(5): 5.0v front panel enable
*/

#define REG_PS_SYNC_DIV0 			0x0031 /* 2.5V Gen */
#define REG_PS_SYNC_DIV1 			0x0032 /* 3.3V Gen */
#define REG_PS_SYNC_DIV2 			0x0033 /* 2.5V Frame */
#define REG_PS_SYNC_DIV3 			0x0034 /* 0.9V Frame */
#define REG_PS_SYNC_DIV4 			0x0035 /* 5.0V FP */

/* Frame FPGA Control */
#define CMD_PROGRAM_FRAME			0x0041
#define REG_FRM_RESET 				0x0036 /* Frame Reset */
#define REG_FRM_10GbE_SEL			0x0037; /* 10GbE Link Select */

/* Clock Enables */
#define CMD_ENABLE_CLKS				0x0031 /* Enable selected Frame FPGA clock crystals */
#define CMD_DISABLE_CLKS			0x0032 /* Disable Frame FPGA clock crystals */
#define REG_CLOCK_EN_REG 			0x0038 /* Clock Enable Register */

/* Programmable Si570 Clock Registers */
#define REG_SI570_REG0 				0x0039
#define REG_SI570_REG1 				0x003A
#define REG_SI570_REG2 				0x003B
#define REG_SI570_REG3 				0x003C

/* Power Monitor Registers */
#define CMD_MON_STOP 				0x0011 /* Stop voltage and current monitor */
#define CMD_MON_START 				0x0012 /* Start voltage and current monitor */

#define REG_VMON_ADC1_CH1   		0x0040 /*	V12P_BUS Voltage Monitor */
#define REG_IMON_ADC1_CH0 			0x0041 /*	V12P_BUS Current Monitor */
#define REG_VMON_ADC0_CH5  			0x0042 /*	V3P3_MGMT Voltage Monitor */
#define REG_IMON_ADC0_CH5  			0x0043 /*	V3P3_MGMT Current Monitor */
#define REG_VMON_ADC0_CH4  			0x0044 /*	V3P3_S3E Voltage Monitor */
#define REG_IMON_ADC0_CH4  			0x0045 /*	V3P3_S3E Current Monitor */
#define REG_VMON_ADC0_CH7  			0x0046 /*	V2P5_MGMT Voltage Monitor */
#define REG_IMON_ADC0_CH7  			0x0047 /*	V2P5_MGMT Current Monitor */
#define REG_VMON_ADC0_CH6  			0x0048 /*	V1P8_MGMT Voltage Monitor */
#define REG_IMON_ADC0_CH6  			0x0049 /*	V1P8_MGMT Current Monitor */
#define REG_VMON_ADC0_CH2  			0x004A /*	V1P2_MGMT Voltage Monitor */
#define REG_IMON_ADC0_CH2  			0x004B /*	V1P2_MGMT Current Monitor */
#define REG_VMON_ADC0_CH3  			0x004C /*	V1P0_ENET Voltage Monitor */
#define REG_IMON_ADC0_CH3  			0x004D /*	V1P0_ENET Current Monitor */
#define REG_VMON_ADC0_CH8  			0x004E /*	V3P3_GEN Voltage Monitor */
#define REG_IMON_ADC0_CH8  			0x004F /*	V3P3_GEN Current Monitor */
#define REG_VMON_ADC0_CH9  			0x0050 /*	V2P5_GEN Voltage Monitor */
#define REG_IMON_ADC0_CH9  			0x0051 /*	V2P5_GEN Current Monitor */
#define REG_VMON_ADC0_CHE  			0x0052 /*	V0P9_V6 Voltage Monitor */
#define REG_IMON_ADC0_CHE  			0x0053 /*	V0P9_V6 Current Monitor */
#define REG_VMON_ADC0_CHD  			0x0054 /*	V2P5_V6 Voltage Monitor */
#define REG_IMON_ADC0_CHD  			0x0055 /*	V2P5_V6 Current Monitor */
#define REG_VMON_ADC0_CHB  			0x0056 /*	V1P0_V6 Voltage Monitor */
#define REG_IMON_ADC0_CHB  			0x0057 /*	V1P0_V6 Current Monitor */
#define REG_VMON_ADC0_CHC  			0x0058 /*	V1P2_V6 Voltage Monitor */
#define REG_IMON_ADC0_CHC  			0x0059 /*	V1P2_V6 Current Monitor */
#define REG_VMON_ADC0_CHF  			0x005A /*	V5P0_FP Voltage Monitor (1/2) */
#define REG_IMON_ADC0_CHF  			0x005B /*	V5P0_FP Current Monitor (1/2) */

/* Status Registers */
#define REG_DCM_STATUS				0x0080
#define REG_FPGA_STATUS    			0x0081
#define REG_BOARD_ID       			0x008D
#define REG_HW_SERIAL_NUM  			0x008E
#define REG_FPGA_VERSION   			0x008F

/* Sandbox Registers */
#define REG_SANDBOX_REG00 			0x00F0
#define REG_SANDBOX_REG01 			0x00F1
#define REG_SANDBOX_REG02 			0x00F2
#define REG_SANDBOX_REG03 			0x00F3
#define REG_SANDBOX_REG04 			0x00F4
#define REG_SANDBOX_REG05 			0x00F5
#define REG_SANDBOX_REG06 			0x00F6
#define REG_SANDBOX_REG07 			0x00F7
#define REG_SANDBOX_REG08 			0x00F8
#define REG_SANDBOX_REG09 			0x00F9
#define REG_SANDBOX_REG0A 			0x00FA
#define REG_SANDBOX_REG0B 			0x00FB
#define REG_SANDBOX_REG0C 			0x00FC
#define REG_SANDBOX_REG0D 			0x00FD
#define REG_SANDBOX_REG0E 			0x00FE
#define REG_SANDBOX_REG0F 			0x00FF

/* ---------------------------------< Frame FPGA Registers > */

/* Command Registers */
#define REG_FRM_COMMAND 			0x8001
#define REG_FRM_READ_ADDRESS 			0x8002
#define REG_FRM_STREAM_TYPE 			0x8003

/* List of Commands */
#define CMD_SEND_SYNC_PULSE                    	0x0100 /* ISSUES A SYNC PULSE */
#define CMD_SYNC_DETECTOR2READOUT              	0x0101 /* COMMAND TO SYNC DETECTOR AND READOUT (SEE IMAGE PROCESSING) */
#define CMD_IP_RESET                           	0x0102 /* RESET IMAGE PROCESSING REGISTERS (N/A) */
#define CMD_WR_CCD_BIAS_REG                    	0x0103 /* WRITE CCD BIAS REGISTERS */
#define CMD_WR_CCD_CLOCK_REG                   	0x0104 /* WRITE CCD CLOCK REGISTER */
#define CMD_SEND_FCRIC_CONFIG                  	0x0105 /* SEND CONFIG DATA TO FRIC */
#define CMD_RESET_FRAME_COUNT                  	0x0106 /* RESET STATISTICS/DEBUG COUNTERS */

/* Ethernet Interface */
#define REG_IF_MAC_FAB1B0   			0x8010
#define REG_IF_MAC_FAB1B1   			0x8011
#define REG_IF_MAC_FAB1B2   			0x8012 
#define REG_IF_IP_FAB1B0    			0x8013
#define REG_IF_IP_FAB1B1    			0x8014 
#define REG_IF_CMD_PORT_NUM_FAB1B   	      	0x8015 
#define REG_IF_STREAM_IN_PORT_NUM_FAB1B        	0x8016 
#define REG_IF_STREAM_OUT_PORT_NUM_FAB1B       	0x8017 

#define REG_XAUI_FAB1B 				0x8018

#define REG_MAC_CONFIG_VEC_FAB1B0 		0x8019
#define REG_MAC_CONFIG_VEC_FAB1B1 		0x801A
#define REG_MAC_STATS1_FAB1B0          		0x801B
#define REG_MAC_STATS1_FAB1B1          		0x801C
#define REG_MAC_STATS2_FAB1B0			0x801D
#define REG_MAC_STATS2_FAB1B1          		0x801E

#define REG_IF_MAC_FAB2B0   			0x8020
#define REG_IF_MAC_FAB2B1   		    	0x8021
#define REG_IF_MAC_FAB2B2   		    	0x8022 
#define REG_IF_IP_FAB2B0    		    	0x8023 
#define REG_IF_IP_FAB2B1    		    	0x8024 
#define REG_IF_CMD_PORT_NUM_FAB2B   	    	0x8025
#define REG_IF_STREAM_IN_PORT_NUM_FAB2B    	0x8026
#define REG_IF_STREAM_OUT_PORT_NUM_FAB2B   	0x8027

#define REG_XAUI_FAB2B 		    	    	0x8028 

#define REG_MAC_CONFIG_VEC_FAB2B0 	    	0x8029 
#define REG_MAC_CONFIG_VEC_FAB2B1 	    	0x802A 
#define REG_MAC_STATS1_FAB2B0              	0x802B
#define REG_MAC_STATS1_FAB2B1              	0x802C
#define REG_MAC_STATS2_FAB2B0              	0x802D
#define REG_MAC_STATS2_FAB2B1              	0x802E

/* SRAM Test Interface */
#define REG_SRAM_COMMAND 			0x8030
/*  1 bit  [0]    >> Read NOT Write
 *  2 bits [3:2] >> Modes:
 *      --  Single RW 0x00
 *      --  Burst RW  0x01
 *      --  Test/Diagnostic 10
 *      --  Sleep  11
 *   1 bit [4]     >> start/stop
 */	 
#define REG_SRAM_START_ADDR1     		0x8031
#define REG_SRAM_START_ADDR0     		0x8032
#define REG_SRAM_STOP_ADDR1      		0x8033
#define REG_SRAM_STOP_ADDR0      		0x8034
#define REG_SRAM_FRAME_DATA_OUT1 		0x8035
#define REG_SRAM_FRAME_DATA_OUT0 		0x8036                           
#define REG_SRAM_FRAME_DATA_IN1			0x8037
#define REG_SRAM_FRAME_DATA_IN0  		0x8038
#define REG_SRAM_FRAME_DV        		0x8039
#define REG_SRAM_STATUS1         		0x803A
#define REG_SRAM_STATUS0         		0x803B

/* Programmable Clock */
#define CMD_FCLK_COMMIT 			0x0012 /* Start I2C Write/Read */
#define REG_FCLK_I2C_ADDRESS 			0x8040 /* [ Slave Address(7), RD/WRn(1), Reg Adress(8) ] Slave adddress Hx58  -> HxB when shifted up by 1 */
#define REG_FCLK_I2C_DATA_WR 			0x8041 /* [ Clock Select(2), Clock Enable (1), 0(5), Write Data (8) ] */
/*   Clock Select: (00): 250 MHz (01): 200 MHz (10): FPGA FCRIC Clk (11): Si570 Programmable */
#define REG_FCLK_I2C_DATA_RD 			0x8042 /* [ Read Failed (1), Write Failed(1), Toggle bit(1), 0(5), Read Data (8) ] */
	
#define REG_TRIGGERSELECT_REG		      	0x8050
#define REG_TRIGGERMASK_REG		      	0x8051  /* [00]==SW Trigger, [01]==FP TrigIn2, [10]==FP TrigIn1, [11]==FP TrigIn1OR2 */
#define REG_CCDFCLKSELECT_REG		      	0x8052
#define REG_CDICLKDISABLE_REG		      	0x8053

/* FRM Status */
#define REG_FRM_DCM_STATUS     			0x8080
#define REG_FRM_FPGA_STATUS    			0x8081
#define REG_FRM_BOARD_ID       			0x808D
#define REG_FRM_HW_SERIAL_NUM  			0x808E
#define REG_FRM_FPGA_VERSION   			0x808F
	
/* Sandbox Registers */
#define REG_FRM_SANDBOX_REG00  			0x80F0
#define REG_FRM_SANDBOX_REG01  			0x80F1
#define REG_FRM_SANDBOX_REG02  			0x80F2
#define REG_FRM_SANDBOX_REG03  			0x80F3
#define REG_FRM_SANDBOX_REG04  			0x80F4
#define REG_FRM_SANDBOX_REG05  			0x80F5
#define REG_FRM_SANDBOX_REG06  			0x80F6
#define REG_FRM_SANDBOX_REG07  			0x80F7
#define REG_FRM_SANDBOX_REG08  			0x80F8
#define REG_FRM_SANDBOX_REG09  			0x80F9
#define REG_FRM_SANDBOX_REG0A  			0x80FA
#define REG_FRM_SANDBOX_REG0B  			0x80FB
#define REG_FRM_SANDBOX_REG0C  			0x80FC
#define REG_FRM_SANDBOX_REG0D  			0x80FD
#define REG_FRM_SANDBOX_REG0E  			0x80FE
#define REG_FRM_SANDBOX_REG0F  			0x80FF
	
/* Image Processing Registers */
#define REG_DETECTOR_REVISION_REG   		0x8100
#define REG_DETECTOR_CONFIG_REG1    		0x8101
#define REG_DETECTOR_CONFIG_REG2    		0x8102
#define REG_DETECTOR_CONFIG_REG3    		0x8103
#define REG_DETECTOR_CONFIG_REG4    		0x8104
#define REG_DETECTOR_CONFIG_REG5    		0x8105
#define REG_DETECTOR_CONFIG_REG6    		0x8106
#define REG_DETECTOR_CONFIG_REG7		0x8107
#define REG_DETECTOR_CONFIG_REG8    		0x8108
#define REG_IMG_PROC_REVISION_REG   		0x8120
#define REG_IMG_PROC_CONFIG_REG1    		0x8121
#define REG_IMG_PROC_CONFIG_REG2    		0x8122
#define REG_IMG_PROC_CONFIG_REG3    		0x8123
#define REG_IMG_PROC_CONFIG_REG4    		0x8124
#define REG_IMG_PROC_CONFIG_REG5    		0x8125
#define REG_IMG_PROC_CONFIG_REG6    		0x8126
#define REG_IMG_PROC_CONFIG_REG7    		0x8127
#define REG_IMG_PROC_CONFIG_REG8    		0x8128
	
#define REG_BIASANDCLOCKREGISTERADDRESS_REG	0x8200
#define REG_BIASANDCLOCKREGISTERDATA_REG	0x8201
#define REG_CLOCKREGISTERDATAOUT_REG		0x8202
#define REG_BIASREGISTERDATAOUT_REG		0x8203

/* Bias Static Registers */
#define REG_BIASCONFIGREGISTER0_REG     	0x8204

/* Clock Static Registers */
#define REG_CLOCKCONFIGREGISTER0_REG     	0x8205

#define REG_EXPOSURETIMEMSB_REG		      	0x8206
#define REG_EXPOSURETIMELSB_REG		      	0x8207
#define REG_ALTEXPOSURETIMEMSB_REG	      0x8306
#define REG_ALTEXPOSURETIMELSB_REG	      0x8307

#define REG_TRIGGERREPETITIONTIMEMSB_REG	0x8208
#define REG_TRIGGERREPETITIONTIMELSB_REG	0x8209
#define REG_DELAYTOEXPOSUREMSB_REG	0x820A
#define REG_DELAYTOEXPOSURELSB_REG	0x820B
#define REG_NUMBEROFEXPOSURE_REG		0x820C
#define REG_SHUTTERTIMEMSB_REG			0x820D
#define REG_SHUTTERTIMELSB_REG			0x820E
#define REG_DELAYTOSHUTTERMSB_REG		0x820F
#define REG_DELAYTOSHUTTERLSB_REG		0x8210

/* Digitizer Registers */
#define REG_FCRIC_MASK_REG1			0x8211
#define REG_FCRIC_MASK_REG2			0x8212
#define REG_FCRIC_MASK_REG3			0x8213
#define REG_LVDS_OVERFLOW_ERROR_REG1		0x8214
#define REG_LVDS_OVERFLOW_ERROR_REG2		0x8215
#define REG_LVDS_OVERFLOW_ERROR_REG3		0x8216
#define REG_LVDS_PARITY_ERROR_REG1		0x8217
#define REG_LVDS_PARITY_ERROR_REG2		0x8218
#define REG_LVDS_PARITY_ERROR_REG3		0x8219
#define REG_LVDS_STOP_BIT_ERROR_REG1		0x821A
#define REG_LVDS_STOP_BIT_ERROR_REG2		0x821B
#define REG_LVDS_STOP_BIT_ERROR_REG3		0x821C
#define REG_FCRIC_WRITE0_REG			0x821D
#define REG_FCRIC_WRITE1_REG			0x821E
#define REG_FCRIC_WRITE2_REG			0x821F
#define REG_FCRIC_READ0_REG			0x8220
#define REG_FCRIC_READ1_REG			0x8221
#define REG_FCRIC_READ2_REG			0x8222
#define REG_DEBUGVIDEO0_REG			0x8223
#define REG_DEBUGVIDEO1_REG			0x8224
#define REG_DEBUGVIDEO2_REG			0x8225
#define REG_DEBUGVIDEO3_REG			0x8226
#define REG_DEBUGVIDEO4_REG			0x8227
#define REG_DEBUGVIDEO5_REG			0x8228
#define REG_DEBUGVIDEO6_REG			0x8229
#define REG_DEBUGVIDEO7_REG			0x822A
#define REG_DEBUGVIDEO8_REG			0x822B
#define REG_DEBUGVIDEO9_REG			0x822C
#define REG_DEBUGVIDEO10_REG			0x822D
#define REG_DEBUGVIDEO11_REG			0x822E
#define REG_DEBUGCOUNTER00_REG			0x822F
#define REG_DEBUGCOUNTER01_REG			0x8230
#define REG_DEBUGCOUNTER02_REG			0x8231
#define REG_DEBUGCOUNTER03_REG			0x8232
#define REG_DEBUGCOUNTER04_REG			0x8233

/* ============================================================================
 *                                                       CIN Commands
 * ============================================================================
 */

/* Common Commands */
#define CMD_READ_REG            0x0001 /* Read Register */

#ifdef __cplusplus
}
#endif

#endif /* CIN_REGISTER_MAP_H */
