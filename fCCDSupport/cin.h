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
#define CIN_DATA_RCVBUF                    100  // Mb 

// The maximum size of the CCD chip is 960 columns by
// 2 x 960 (1920) rows. In frame store you only read out 960 x 960
//
// If we overscan the columns, then you get 960 x 1.2 columns
// which is 1152 columns. 
// 
#define CIN_DATA_FRAME_HEIGHT              1920
#define CIN_DATA_FRAME_WIDTH               1152
#define CIN_DATA_MAX_STREAM                2300000
#define CIN_DATA_CCD_COLS               96
#define CIN_DATA_CCD_COLS_PER_CHAN      10
#define CIN_DATA_PIPELINE_FLUSH         1344 // 7 converts * 2 * 96 cols

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

#define ERROR_COMMENT(fmt)\
  if(1) { fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__); }
#define ERROR_PRINT(fmt, ...) \
  if(1) { fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, __VA_ARGS__); }

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
  int size_x;
  int size_y;
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

#define CIN_CTL_DCM_LOCKED          0x0001
#define CIN_CTL_DCM_PSDONE          0x0002
#define CIN_CTL_DCM_STATUS0         0x0004
#define CIN_CTL_DCM_STATUS1         0x0008
#define CIN_CTL_DCM_STATUS2         0x0010
#define CIN_CTL_DCM_TX1_READY       0x0020
#define CIN_CTL_DCM_TX2_READY       0x0040
#define CIN_CTL_DCM_ATCA_ALARM      0x0080

#define CIN_CTL_TRIG_INTERNAL       0
#define CIN_CTL_TRIG_EXTERNAL_1     1
#define CIN_CTL_TRIG_EXTERNAL_2     2
#define CIN_CTL_TRIG_EXTERNAL_BOTH  3

#define CIN_CTL_FOCUS_BIT           0x0002

#define CIN_CTL_FCLK_125            0
#define CIN_CTL_FCLK_200            1
#define CIN_CTL_FCLK_250            2

/* 
 * Datastructures for status readouts 
 */


typedef struct cin_ctl_fpga_status {
  uint16_t board_id;
  uint16_t serial_no;
  uint16_t fpga_ver;
  uint16_t fpga_status;
} cin_ctl_fpga_status_t;

typedef struct cin_ctl_pwr_val {
  double i;
  double v;
} cin_ctl_pwr_val_t;

typedef struct {
  cin_ctl_pwr_val_t bus_12v0;
  cin_ctl_pwr_val_t mgmt_3v3;
  cin_ctl_pwr_val_t mgmt_2v5;
  cin_ctl_pwr_val_t mgmt_1v2;
  cin_ctl_pwr_val_t enet_1v0;
  cin_ctl_pwr_val_t s3e_3v3; 
  cin_ctl_pwr_val_t gen_3v3;
  cin_ctl_pwr_val_t gen_2v5;
  cin_ctl_pwr_val_t v6_0v9;
  cin_ctl_pwr_val_t v6_1v0;
  cin_ctl_pwr_val_t v6_2v5;
  cin_ctl_pwr_val_t fp;
} cin_ctl_pwr_mon_t;

/*------------------------
 * UDP Socket
 *------------------------*/

int cin_ctl_init_port(struct cin_port* cp, char* ipaddr, uint16_t port);
int cin_ctl_close_port(struct cin_port* cp);

/*------------------------
 * CIN Read-Write
 *------------------------*/

int cin_ctl_read(struct cin_port* cp, uint16_t reg, uint16_t *val);
int cin_ctl_write(struct cin_port* cp, uint16_t reg, uint16_t val);
int cin_ctl_stream_write(struct cin_port* cp, char* val,int size);

/*------------------------
 * CIN PowerUP-PowerDown
 *------------------------*/

int cin_pwr(struct cin_port *cp, int pwr);
int cin_on(struct cin_port* cp);
int cin_off(struct cin_port* cp);

int cin_fp_pwr(struct cin_port* cp, int pwr);
int cin_fp_on(struct cin_port* cp);
int cin_fp_off(struct cin_port* cp);

/*------------------------
 * CIN Configuration-Status
 *------------------------*/

int cin_ctl_load_config(struct cin_port* cp,char *filename);
int cin_ctl_load_firmware(struct cin_port* cp,struct cin_port* dcp, char *filename);
int cin_ctl_set_fclk(struct cin_port* cp, int clkfreq);
int cin_ctl_get_fclk(struct cin_port* cp, int *clkfreq);
int cin_ctl_get_cfg_fpga_status(struct cin_port* cp,
                                cin_ctl_fpga_status_t *_val);
int cin_ctl_get_dcm_status(struct cin_port* cp, uint16_t *_val);

/* Power status */


double cin_ctl_current_calc(uint16_t val);
int cin_ctl_get_power_status(struct cin_port* cp, int *pwr, cin_ctl_pwr_mon_t *values);
void cin_ctl_display_pwr(FILE *out, cin_ctl_pwr_mon_t *values);
void cin_ctl_display_pwr_line(FILE *out,const char* msg, cin_ctl_pwr_val_t val);
int cin_ctl_calc_vi_status(struct cin_port* cp, 
                           uint16_t vreg, uint16_t ireg, double vfact,
                           cin_ctl_pwr_val_t *vi);

/*------------------------
 * CIN Control
 *------------------------*/

int cin_ctl_set_bias(struct cin_port* cp,int val);
int cin_ctl_set_clocks(struct cin_port* cp,int val);
int cin_ctl_set_trigger(struct cin_port* cp,int val);
int cin_ctl_get_trigger(struct cin_port* cp, int *val);
int cin_ctl_set_focus(struct cin_port* cp, int val);
int cin_ctl_int_trigger_start(struct cin_port* cp, int nimages);
int cin_ctl_int_trigger_stop(struct cin_port* cp);
int cin_ctl_ext_trigger_start(struct cin_port* cp, int trigger_mode);
int cin_ctl_ext_trigger_stop(struct cin_port* cp);
int cin_ctl_set_exposure_time(struct cin_port* cp,float e_time);
int cin_ctl_set_trigger_delay(struct cin_port* cp,float t_time);
int cin_ctl_set_cycle_time(struct cin_port* cp,float ftime);
int cin_ctl_set_frame_count_reset(struct cin_port* cp);

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
