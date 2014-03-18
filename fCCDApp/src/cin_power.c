#include <stdio.h>
#include <unistd.h> /* for sleep() */
#include <string.h>
#include <stdlib.h>

#include "cin.h"
#include <iocsh.h>
#include <epicsExport.h>
#include "cin_register_map.h"


// Set HARDWARE to 1 on real system
#define HARDWARE 1
//#undef HARDWARE
#define LOCAL static

#define kSTR_UNINITIALIZED "---"


LOCAL char cin_fpga_config[1024]     = kSTR_UNINITIALIZED;
LOCAL char cin_waveform_config[1024] = kSTR_UNINITIALIZED;
LOCAL char cin_fcric_config[1024]    = kSTR_UNINITIALIZED;
LOCAL char cin_bias_config[1024]     = kSTR_UNINITIALIZED;
LOCAL struct cin_port cp[2];

static const iocshArg FCCD_Config_Dirs_Arg  = {"Path", iocshArgString};
static const iocshArg * const FCCD_ConfigDirsArgs[] =  
   {&FCCD_Config_Dirs_Arg, &FCCD_Config_Dirs_Arg, &FCCD_Config_Dirs_Arg, 
    &FCCD_Config_Dirs_Arg, &FCCD_Config_Dirs_Arg };
static const iocshFuncDef configFCCD_Dirs = {"FCCD_ConfigDirs", 5, FCCD_ConfigDirsArgs};




void cin_power_up (){

   int ret_fpga,ret_fclk;  
   printf("***cin_power_up\n");// debug
   
   cin_init_ctl_port(&cp[0], 0, 0);
   cin_init_ctl_port(&cp[1], 0,49202);
   
   printf("***cp[0] srvaddr = %s, cliaddr = %s, srvport = %d cliport = %d\r\n", 
      cp[0].srvaddr, cp[0].cliaddr, cp[0].srvport, cp[0].cliport);
   printf("***cp[1] srvaddr = %s, cliaddr = %s, srvport = %d cliport = %d\r\n", 
      cp[1].srvaddr, cp[1].cliaddr, cp[1].srvport, cp[1].cliport);
   printf("***Control ports initialized\n");// debug

   
   // Check that file paths have been initialized
   if  (strcmp(cin_fpga_config,kSTR_UNINITIALIZED) == 0)  
   {
      fprintf(stdout,"\n!Severe Error. File Path cin_fpga_config has not been set!\n\n");
      return;     // ****** error out *********
   }
   if  (strcmp(cin_waveform_config,kSTR_UNINITIALIZED) == 0)  
   {
      fprintf(stdout,"\n!Severe Error. File Path cin_waveform_config has not been set!\n\n");
      return;     // ****** error out *********
   }
   if  (strcmp(cin_fcric_config,kSTR_UNINITIALIZED) == 0) 
   {
      fprintf(stdout,"\n!Severe Error. File Path cin_fcric_config has not been set!\n\n");
      return;     // ****** error out *********
   }
   if  (strcmp(cin_bias_config,kSTR_UNINITIALIZED) == 0) 
   {
      fprintf(stdout,"/n!Severe Error. File Path cin_bias_config has not been set!/n/n");
      return;     // ****** error out *********
   }
   
   
#ifdef HARDWARE

	cin_off(&cp[0]);
	sleep(5);

	cin_on(&cp[0]);
	sleep(5);

	cin_fp_on(&cp[0]);
	sleep(5);

	cin_load_firmware(&cp[0],&cp[1],cin_fpga_config);	
	sleep(5);
   
	// Set CIN DATA IP address to 10.23.5.127 
#define LOWER_IP_ADDRESS 0x057F  // 5.127
#define UPPER_IP_ADDRESS 0x0A17  // 10.23
 
	cin_ctl_write(&cp[0],REG_IF_IP_FAB1B0,  LOWER_IP_ADDRESS);
        usleep(1000);

	cin_ctl_write(&cp[0],REG_IF_IP_FAB1B1,  UPPER_IP_ADDRESS);
	usleep(1000);

	ret_fpga=cin_get_cfg_fpga_status(&cp[0]);
	sleep(1);

	ret_fclk=cin_get_fclk_status(&cp[0]);			
	sleep(1);

	cin_get_power_status(&cp[0]);
	sleep(1);

/************************* FCCD Configuration **************************/	

	cin_load_config(&cp[0],cin_waveform_config);		//Load FCCD clock configuration
	sleep(3);
/*	
	cin_load_config(&cp[0],cin_fcric_config);		//Load CIN fcric Configuration
	sleep(3);
	
	cin_load_config(&cp[0],cin_bias_config);		//Load FCCD bias Configuration
	sleep(3);
*/
/**********************************************************************/		
	fprintf(stdout,"\nCIN startup complete!!\n");

	if (ret_fpga==0){fprintf(stdout,"  *FPGA Status: OK\n");}
	else{fprintf(stdout,"  *FPGA Status: ERROR\n");}
	
	if (ret_fclk==0){fprintf(stdout,"  *FCLK Status: OK\n");}
	else{fprintf(stdout,"  *FCLK Status: ERROR\n");}	
#endif
 
}

void cin_power_down(){

   struct cin_port cp[2];

   // debug
   printf("***cin_power_down\n");
   
#ifdef HARDWARE
   
   // Should add a check that cp is valid...

   fprintf(stdout,"Turning off clock and bias.......\n");

	cin_set_bias(&cp[0],0);   	//Turn OFF camera CCD bias
	sleep(1);						

	cin_set_clocks(&cp[0],0);		//Turn OFF camera CCD bias
	sleep(1);

	cin_fp_off(&cp[0]);      		//Power OFF CIN front Panel
	sleep(2);	

	cin_off(&cp[0]);          	//Power OFF CIN
	sleep(4);

	fprintf(stdout,"Closing ports.......\n");

	cin_close_ctl_port(&cp[0]);       //Close Control port
	cin_close_ctl_port(&cp[1]); 			//Close Stream-in port
	sleep(1);

	fprintf(stdout,"CIN shutdown complete!!\n");

#endif   
  
}


// Public wrappers to cin_api functions
int CIN_set_bias(int val)
{
   printf("CIN_set_bias:%d\n", val);
   return cin_set_bias(cp,val);
}

int CIN_set_clocks(int val)
{
   printf("CIN_set_clocks:%d\n", val);
   return cin_set_clocks(cp, val);
}

//val:{0-Internal, 1-External1, 2-External2, 3-External 1 or 2}
int CIN_set_trigger(int val)
{
   printf("CIN_set_trigger:%d\n", val);
   return cin_set_trigger(cp,val); 
}

//Return:{0-Internal, 1-External1, 2-External2, 3-External 1 or 2}
int CIN_get_trigger_status()
{
   return cin_get_trigger_status(cp);
}

//val: {1-single, 0-continuous, N>1 is Multiple N Images)
int CIN_set_trigger_mode(int val)
{
   printf("CIN_set_trigger_mode :%d\n", val);
   return cin_set_trigger_mode(cp, val);
}

//Starts a single trigger or continuous triggers
// depending on previous call to set_trigger_mode
int CIN_trigger_start()
{
   printf("CIN_trigger_start\n");
   return cin_trigger_start(cp);
}
  
//Stops triggers
int CIN_trigger_stop()
{
   printf("CIN_trigger_stop\n");
   return cin_trigger_stop(cp);
}

     
// Set the Camera exposure time
// Input:e_time (ms)
int CIN_set_exposure_time(float e_time)
{
   return cin_set_exposure_time(cp,e_time); 
}

//Set the trigger delay time
//Input:t_time (ms)  
int CIN_set_trigger_delay(float t_time)
{  
   return cin_set_trigger_delay(cp,t_time);
}  

//Set the Camera cyle time time
//Input:c_time (ms)             
int CIN_set_cycle_time(float c_time)
{
   return cin_set_cycle_time(cp,c_time);  
}



//extern "C" {

   static void configFCCD_DirsCallFunc(const iocshArgBuf *args)
   {
       FCCD_ConfigDirs(args[0].sval, args[1].sval, args[2].sval, args[3].sval, args[4].sval);
   }
   static void FCCD_Dirs_Register(void)
   {

       iocshRegister(&configFCCD_Dirs, configFCCD_DirsCallFunc);
   }

   int FCCD_ConfigDirs(
      const char *fccd_config_dir, 
      const char *fpga_configfile,
      const char *cin_configfile_waveform , 
      const char *cin_configfile_fcric, 
      const char *cin_configfile_bias)
   {
      printf("fccd_config_dir: %s\n", fccd_config_dir);
      printf("fpga_configfile: %s\n", fpga_configfile);
      printf("cin_configfile_waveform: %s\n", cin_configfile_waveform);
      printf("cin_configfile_fcric: %s\n", cin_configfile_fcric);
      printf("cin_configfile_bias: %s\n", cin_configfile_bias);
     
      sprintf(cin_fpga_config,"%s%s", fccd_config_dir,fpga_configfile);
      sprintf(cin_waveform_config,"%s%s", fccd_config_dir,cin_configfile_waveform);
      sprintf(cin_fcric_config,"%s%s", fccd_config_dir,cin_configfile_fcric);
      sprintf(cin_bias_config,"%s%s", fccd_config_dir,cin_configfile_bias);

      return(0);
   }
   
   epicsExportRegistrar(FCCD_Dirs_Register);
//}
