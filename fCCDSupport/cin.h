#ifndef __CIN_H__
#define __CIN_H__

#include <stdint.h>     // for uint16_t
#include <stdio.h>      // for fprintf
#include <sys/socket.h> // For struct sockaddr_in
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/time.h>   // For timespec

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------------
 *
 * Global definitions
 *
 * -------------------------------------------------------------------------------
 */

#define CIN_CTL_IP                         "192.168.1.207"
#define CIN_CTL_PORT                       49200
#define CIN_CTL_FRMW_PORT                  49202
#define CIN_DATA_IP                        "10.0.5.207"
#define CIN_DATA_PORT                      49201
#define CIN_DATA_CTL_PORT                  49203
#define CIN_DATA_MAX_MTU                   9000
#define CIN_DATA_UDP_HEADER                8
#define CIN_DATA_MAGIC_PACKET              UINT64_C(0x0000F4F3F2F1F000)
#define CIN_DATA_MAGIC_PACKET_MASK         UINT64_C(0x0000FFFFFFFFFF00)
#define CIN_DATA_TAIL_MAGIC_PACKET         UINT64_C(0x010DF0ADDEF2F1F0)
#define CIN_DATA_TAIL_MAGIC_PACKET_MASK    UINT64_C(0xFFFFFFFFFFFFFFFF)
#define CIN_DATA_DROPPED_PACKET_VAL        0x2000
#define CIN_DATA_DATA_MASK                 0x1FFF
#define CIN_DATA_PACKET_LEN                8184
#define CIN_DATA_MAX_PACKETS               542
//#define CIN_DATA_FRAME_HEIGHT              1924
#define CIN_DATA_FRAME_HEIGHT              1950
#define CIN_DATA_FRAME_WIDTH               1152
#define CIN_DATA_RCVBUF                    100  // Mb 

/* -------------------------------------------------------------------------------
 *
 * Definitions for CIN DATA config
 *
 * -------------------------------------------------------------------------------
 */

#define CIN_DATA_MODE_PUSH_PULL         0x01
#define CIN_DATA_MODE_DBL_BUFFER        0x02
#define CIN_DATA_MODE_BUFFER            0x04
#define CIN_DATA_MODE_WRITER            0x08
#define CIN_DATA_MODE_DBL_BUFFER_COPY   0x10

/* ---------------------------------------------------------------------
 *
 * MACROS for debugging
 *
 * ---------------------------------------------------------------------
 */

#ifdef __DEBUG__
  #define DEBUG_PRINT(fmt, ...) \
    if(1) { fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, __VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) do {}while(0)
#endif

#ifdef __DEBUG__
  #define DEBUG_COMMENT(fmt)\
    if(1) { fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__); }
#else
  #define DEBUG_COMMENT(...) do {}while(0)
#endif

/* ---------------------------------------------------------------------
 *
 * Global datastructures
 *
 * ---------------------------------------------------------------------
 */

struct cin_port {
    char *srvaddr;
    char *cliaddr;
    uint16_t srvport;
    uint16_t cliport;
    int sockfd;
    struct timeval tv;
    struct sockaddr_in sin_srv; /* server info */
    struct sockaddr_in sin_cli; /* client info (us!) */
    socklen_t slen; /* for recvfrom() */
    int rcvbuf; /* For setting data recieve buffer */
    int rcvbuf_rb; /* For readback */
};

typedef struct cin_data_frame {
  uint16_t *data;
  uint16_t number;
  struct timeval timestamp;
} cin_data_frame_t;

typedef struct cin_data_stats {
  // Frame data

  int last_frame;
  double framerate;
  double datarate;
  double av_datarate;
  double av_framerate;

  // FIFO data
  
  double packet_percent_full;
  double frame_percent_full;
  double image_percent_full;
  long int packet_overruns;
  long int frame_overruns;
  long int image_overruns;

  // Packet stats
  
  long int dropped_packets;
  long int mallformed_packets;
} cin_data_stats_t;

/* ---------------------------------------------------------------------
 *
 * CIN Control Routines
 *
 * ---------------------------------------------------------------------
 */

/*------------------------
 * UDP Socket
 *------------------------*/
int cin_init_ctl_port(struct cin_port* cp, char* ipaddr, uint16_t port);

int cin_close_ctl_port(struct cin_port* cp);

/*------------------------
 * CIN Read-Write
 *------------------------*/
uint16_t cin_ctl_read(struct cin_port* cp, uint16_t reg);

int cin_ctl_write(struct cin_port* cp, uint16_t reg, uint16_t val);
	//TODO - implement write verification procedure */

int cin_stream_write(struct cin_port* cp, char* val,int size);
	//TODO - implement write verification procedure

/*------------------------
 * CIN PowerUP-PowerDown
 *------------------------*/
int cin_on(struct cin_port* cp);

int cin_off(struct cin_port* cp);

int cin_fp_on(struct cin_port* cp);

int cin_fp_off(struct cin_port* cp);
/*------------------------
 * CIN Configuration-Status
 *------------------------*/
int cin_load_config(struct cin_port* cp,char *filename);

int cin_load_firmware(struct cin_port* cp,struct cin_port* dcp, char *filename);

int cin_set_fclk(struct cin_port* cp,uint16_t clkfreq);
/*
 * Input:clkfreq={125, 200, 250}(MHz)
 */

int cin_get_fclk_status(struct cin_port* cp);
/*
 * Return:{0-FCLK Configured, (-1)-FCLK not configured}
 */

int cin_get_cfg_fpga_status(struct cin_port* cp);
/*
 *Return:{0-FPGA Configured, (-1)-FPGA not configured}
 */

int cin_get_power_status(struct cin_port* cp);

/*------------------------
 * CIN Control
 *------------------------*/
int cin_set_bias(struct cin_port* cp,int val);
/*
 * Input:val={0-OFF,1-ON}
 */
int cin_set_clocks(struct cin_port* cp,int val);
/*
 * Input:val={0-OFF,1-ON}
 */

int cin_set_trigger(struct cin_port* cp,int val);
/*
 * Input:val={0-Internal, 1-External1, 2-External2, 3-External 1 or 2}
 */

uint16_t cin_get_trigger_status (struct cin_port* cp);
/*
 * Return:{0-Internal, 1-External1, 2-External2, 3-External 1 or 2}
 */

int cin_set_trigger_mode(struct cin_port* cp,int val);
/*
 * Input:val={0-Single, 1-Continuous, 2-Multiple}
 */

int cin_set_exposure_time(struct cin_port* cp,float e_time);
/*
 * Input:e_time (s)
 */	

int cin_set_trigger_delay(struct cin_port* cp,float t_time);
/*
 * Input:t_time (us)
 */

int cin_set_cycle_time(struct cin_port* cp,float c_time);
/*
 * Input:c_time (s)
 */    

int cin_trigger_start(struct cin_port* cp);
/*
 * Start triggers.   Looks at current mode (single, continuous
 * or multiple) to determine which registers to set
 */

int cin_trigger_stop(struct cin_port* cp);
/*
 * Stops triggers
 */

int cin_set_frame_count_reset(struct cin_port* cp);

/*------------------------
 * Testing
 *------------------------*/
int cin_test_cfg_leds(struct cin_port* cp); //Flash configuration LEDs 	

/* ---------------------------------------------------------------------
 *
 * CIN Data Routines
 *
 * ---------------------------------------------------------------------
 */

int cin_init_data_port(struct cin_port* dp,
                       char* ipaddr, uint16_t port,
                       char* cin_ipaddr, uint16_t cin_port,
                       int rcvbuf);
/*
 * Initialize the data port used for recieveing the UDP packets. A
 * structure of cin_port is modified with the settings. If the strings
 * are NULL and the ports zero then defaults are used.
 */

int cin_data_init(int mode, int packet_buffer_len, int frame_buffer_len);
/*
 * Initialize the data handeling routines and start the threads for listening.
 * mode should be set for the desired output. The packet_buffer_len in the
 * length of the packet FIFO in number of packets. The frame_buffer_len is
 * the number of data frames to buffer. 
 */
 
void cin_data_wait_for_threads(void);
/* 
 * Block until all th threads have closed. 
 */
int cin_data_stop_threads(void);
/* 
 * Send a cancel request to all threads.
 */

struct cin_data_frame* cin_data_get_next_frame(void);
void cin_data_release_frame(int free_mem);

struct cin_data_frame* cin_data_get_buffered_frame(void);
void cin_data_release_buffered_frame(void);

struct cin_data_stats cin_data_get_stats(void);
void cin_data_show_stats(void);

int cin_data_load_frame(uint16_t *buffer, uint16_t *frame_num);

void cin_data_start_monitor_output(void);
void cin_data_stop_monitor_output(void);

#ifdef __cplusplus
}
#endif

#endif //__CIN_H__
