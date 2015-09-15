#ifndef __CIN_H__
#define __CIN_H__

#include <stdint.h>     // for uint16_t
#include <stdio.h>      // for fprintf
#include <sys/socket.h> // For struct sockaddr_in
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/time.h>   // For timespec
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const char *cin_build_git_time;
extern const char *cin_build_git_sha;
extern const char *cin_build_version;

/* -------------------------------------------------------------------------------
 *
 * Global definitions
 *
 * -------------------------------------------------------------------------------
 */

#define CIN_CTL_IP                         "192.168.1.207"
#define CIN_CTL_SVR_PORT                   49200
#define CIN_CTL_CLI_PORT                   50200
#define CIN_CTL_SVR_FRMW_PORT              49202
#define CIN_CTL_CLI_FRMW_PORT              50202

#define CIN_CTL_MAX_READ_TRIES             10
#define CIN_CTL_MAX_WRITE_TRIES            5
#define CIN_CTL_WRITE_SLEEP                2000 // microsecs

#define CIN_CTL_POWER_ENABLE               0x001F
#define CIN_CTL_POWER_DISABLE              0x0000
#define CIN_CTL_FP_POWER_ENABLE            0x0020

#define CIN_CTL_DCM_LOCKED                 0x0001
#define CIN_CTL_DCM_PSDONE                 0x0002
#define CIN_CTL_DCM_STATUS0                0x0004
#define CIN_CTL_DCM_STATUS1                0x0008
#define CIN_CTL_DCM_STATUS2                0x0010
#define CIN_CTL_DCM_TX1_READY              0x0020
#define CIN_CTL_DCM_TX2_READY              0x0040
#define CIN_CTL_DCM_ATCA_ALARM             0x0080

#define CIN_CTL_TRIG_INTERNAL              0x0000
#define CIN_CTL_TRIG_EXTERNAL_1            0x0001
#define CIN_CTL_TRIG_EXTERNAL_2            0x0002
#define CIN_CTL_TRIG_EXTERNAL_BOTH         0x0003

#define CIN_CTL_FOCUS_BIT                  0x0002

#define CIN_CTL_FCLK_125                   0x0000
#define CIN_CTL_FCLK_200                   0x0001
#define CIN_CTL_FCLK_250                   0x0002
#define CIN_CTL_FCLK_125_C                 0x0003
#define CIN_CTL_FCLK_200_C                 0x0004
#define CIN_CTL_FCLK_250_C                 0x0005
#define CIN_CTL_FCLK_180_C                 0x0006

#define CIN_CTL_FPGA_STS_CFG               0x8000
#define CIN_CTL_FPGA_STS_FP_PWR            0x0008

#define CIN_CTL_DCM_STS_ATCA               0x0080
#define CIN_CTL_DCM_STS_LOCKED             0x0001
#define CIN_CTL_DCM_STS_OVERIDE            0x0800

#define CIN_CTL_MUX1_VCLK1                 0x0001
#define CIN_CTL_MUX1_VCLK2                 0x0002
#define CIN_CTL_MUX1_VCLK3                 0x0003
#define CIN_CTL_MUX1_ATG                   0x0004
#define CIN_CTL_MUX1_VFSCLK1               0x0005
#define CIN_CTL_MUX1_VFSCLK2               0x0006
#define CIN_CTL_MUX1_VFSCLK3               0x0007
#define CIN_CTL_MUX1_HCLK1                 0x0008
#define CIN_CTL_MUX1_HCLK2                 0x0009
#define CIN_CTL_MUX1_OSW                   0x000A
#define CIN_CTL_MUX1_RST                   0x000B
#define CIN_CTL_MUX1_CONVERT               0x000C
#define CIN_CTL_MUX1_SHUTTER               0x000D
#define CIN_CTL_MUX1_SWTRIGGER             0x000E
#define CIN_CTL_MUX1_TRIGMON               0x000F
#define CIN_CTL_MUX1_EXPOSE                0x0000

#define CIN_CTL_MUX2_VCLK1                 0x0010
#define CIN_CTL_MUX2_VCLK2                 0x0020
#define CIN_CTL_MUX2_VCLK3                 0x0030
#define CIN_CTL_MUX2_ATG                   0x0040
#define CIN_CTL_MUX2_VFSCLK1               0x0050
#define CIN_CTL_MUX2_VFSCLK2               0x0060
#define CIN_CTL_MUX2_VFSCLK3               0x0070
#define CIN_CTL_MUX2_HCLK1                 0x0080
#define CIN_CTL_MUX2_HCLK2                 0x0090
#define CIN_CTL_MUX2_HCLK3                 0x00A0
#define CIN_CTL_MUX2_OSW                   0x00B0
#define CIN_CTL_MUX2_RST                   0x00C0
#define CIN_CTL_MUX2_CONVERT               0x00D0
#define CIN_CTL_MUX2_SAVE                  0x00E0
#define CIN_CTL_MUX2_HWTRIG                0x00F0
#define CIN_CTL_MUX2_EXPOSE                0x0000

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
#define CIN_DATA_CTRL_MASK                 0xE000
#define CIN_DATA_SIGN_MASK                 0x1000
#define CIN_DATA_GAIN_8                    0xC000
#define CIN_DATA_GAIN_4                    0x4000
#define CIN_DATA_PACKET_LEN                8184
#define CIN_DATA_MAX_PACKETS               542
#define CIN_DATA_RCVBUF                    100  // Mb 

// The maximum size of the CCD chip is 960 columns by
// 2 x 960 (1920) rows. In frame store you only read out 960 x 960
//
// If we overscan the columns, then you get 960 x 1.2 columns
// which is 1152 columns. 
// 
// We include 20 more rows
//
#define CIN_DATA_MAX_FRAME_X               1152 // Columns
#define CIN_DATA_MAX_FRAME_Y               2050 // Rows
#define CIN_DATA_MAX_STREAM                2400000
#define CIN_DATA_CCD_COLS                  96
#define CIN_DATA_CCD_COLS_PER_CHAN         10
#define CIN_DATA_PIPELINE_FLUSH            1344 // 7 converts * 2 * 96 cols

/* -------------------------------------------------------------------------------
 *
 * Definitions for CIN DATA config
 *
 * -------------------------------------------------------------------------------
 */

#define CIN_DATA_MODE_CALLBACK          0x01
#define CIN_DATA_MODE_DBL_BUFFER        0x02

/* -------------------------------------------------------------------------------
 *
 * Definitions for CIN BIAS SETTINGS
 *
 * -------------------------------------------------------------------------------
 */

#define NUM_BIAS_VOLTAGE            20

#define pt_posH                     0
#define pt_negH                     1
#define pt_posRG                    2
#define pt_negRG                    3
#define pt_posSW                    4
#define pt_negSW                    5
#define pt_posV                     6
#define pt_negV                     7
#define pt_posTG                    8
#define pt_negTG                    9
#define pt_posVF                    10
#define pt_negVF                    11
#define pt_NEDGE                    12
#define pt_OTG                      13
#define pt_VDDR                     14
#define pt_VDD_OUT                  15
#define pt_BUF_Base                 16
#define pt_BUF_Delta                17
#define pt_Spare1                   18
#define pt_Spare2                   19

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

#define FIFO_MAX_READERS 10 

typedef struct {
  void *data;
  void *head;
  void *tail[FIFO_MAX_READERS];
  void *end;
  int readers;
  long int size;
  int elem_size;
  int full;
  long int overruns;
  pthread_mutex_t mutex;
  pthread_cond_t signal;
} fifo;

typedef struct cin_ctl_listener {
  struct cin_port *cp;
  fifo ctl_fifo;
  pthread_t thread_id;
} cin_ctl_listener_t;

typedef struct cin_port {
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
    cin_ctl_listener_t *listener;
    pthread_mutex_t access; /* For sequential access to CIN */
    pthread_mutexattr_t access_attr;
} cin_port_t;

typedef struct cin_data_frame {
  uint16_t *data;
  uint16_t number;
  struct timespec timestamp;
  int size_x;
  int size_y;
  void *usr_ptr; // User container
} cin_data_frame_t;

typedef struct cin_data_stats {
  // Frame data

  int last_frame;
  double framerate;
  double datarate;

  // FIFO data
  
  double packet_percent_full;
  double frame_percent_full;
  double image_percent_full;
  long int packet_overruns;
  long int frame_overruns;
  long int image_overruns;
  long int packet_used;
  long int frame_used;
  long int image_used;

  // Packet stats
  
  long int dropped_packets;
  long int mallformed_packets;
} cin_data_stats_t;

// Callback functions

typedef void (*cin_data_callback) (cin_data_frame_t *);

/* ---------------------------------------------------------------------
 *
 * CIN Control Routines
 *
 * ---------------------------------------------------------------------
 */


/* 
 * Datastructures for status readouts 
 */


typedef struct cin_ctl_id {
  uint16_t board_id;
  uint16_t serial_no;
  uint16_t fpga_ver;
} cin_ctl_id_t;

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
 * Reporting functions
 *------------------------*/

void cin_report(FILE *fp, int details);

/*------------------------
 * UDP Socket
 *------------------------*/

int cin_ctl_init_port(struct cin_port* cp, char* ipaddr, uint16_t oport, uint16_t iport);
int cin_ctl_close_port(struct cin_port* cp);

/*------------------------
 * CIN Read-Write
 *------------------------*/

int cin_ctl_read(struct cin_port* cp, uint16_t reg, uint16_t *val);
int cin_ctl_write(struct cin_port* cp, uint16_t reg, uint16_t val, int wait);
int cin_ctl_stream_write(struct cin_port* cp, char* val,int size);
int cin_ctl_write_with_readback(struct cin_port* cp, uint16_t reg, uint16_t val);

/*------------------------
 * CIN PowerUP-PowerDown
 *------------------------*/

int cin_ctl_pwr(struct cin_port *cp, int pwr);
int cin_ctl_fp_pwr(struct cin_port* cp, int pwr);

/*------------------------
 * CIN Configuration-Status
 *------------------------*/

int cin_ctl_load_config(struct cin_port* cp,char *filename);
int cin_ctl_load_firmware(struct cin_port* cp,struct cin_port* dcp, char *filename);
int cin_ctl_set_fclk(struct cin_port* cp, int clkfreq);
int cin_ctl_get_fclk(struct cin_port* cp, int *clkfreq);
int cin_ctl_set_dco(struct cin_port* cp, int freeze);
int cin_ctl_get_cfg_fpga_status(struct cin_port* cp, uint16_t *_val);
int cin_ctl_get_id(struct cin_port *cp, cin_ctl_id_t *_val);
void cin_ctl_display_id(FILE *out, cin_ctl_id_t val);
void cin_ctl_display_fpga_status(FILE *out, uint16_t val);
int cin_ctl_get_dcm_status(struct cin_port* cp, uint16_t *_val);
void cin_ctl_display_dcm_status(FILE *out, uint16_t *_val);

/* Power status */


double cin_ctl_current_calc(uint16_t val);
int cin_ctl_get_power_status(struct cin_port* cp, int full, int *pwr, cin_ctl_pwr_mon_t *values);
void cin_ctl_display_pwr(FILE *out, cin_ctl_pwr_mon_t *values);
void cin_ctl_display_pwr_line(FILE *out,const char* msg, cin_ctl_pwr_val_t val);
int cin_ctl_calc_vi_status(struct cin_port* cp, 
                           uint16_t vreg, uint16_t ireg, double vfact,
                           cin_ctl_pwr_val_t *vi);

/*------------------------
 * CIN Control
 *------------------------*/

int cin_ctl_get_camera_pwr(struct cin_port* cp, int *val);
int cin_ctl_set_camera_pwr(struct cin_port* cp, int val);
int cin_ctl_set_bias(struct cin_port* cp,int val);
int cin_ctl_get_bias(struct cin_port* cp, int *val);
int cin_ctl_set_clocks(struct cin_port* cp,int val);
int cin_ctl_get_clocks(struct cin_port* cp, int *val);
int cin_ctl_set_trigger(struct cin_port* cp,int val);
int cin_ctl_get_trigger(struct cin_port* cp, int *val);
int cin_ctl_set_focus(struct cin_port* cp, int val);
int cin_ctl_get_focus(struct cin_port* cp, int *val);
int cin_ctl_get_triggering(struct cin_port *cp, int *trigger);
int cin_ctl_int_trigger_start(struct cin_port* cp, int nimages);
int cin_ctl_int_trigger_stop(struct cin_port* cp);
int cin_ctl_ext_trigger_start(struct cin_port* cp, int trigger_mode);
int cin_ctl_ext_trigger_stop(struct cin_port* cp);
int cin_ctl_set_exposure_time(struct cin_port* cp,float e_time);
int cin_ctl_set_trigger_delay(struct cin_port* cp,float t_time);
int cin_ctl_set_cycle_time(struct cin_port* cp,float ftime);
int cin_ctl_frame_count_reset(struct cin_port* cp);
int cin_ctl_set_mux(struct cin_port *cp, int setting);
int cin_ctl_get_mux(struct cin_port *cp, int *setting);
int cin_ctl_set_fcric_gain(struct cin_port *cp, int gain);

/*------------------------
 * CIN TCP/IP Settings
 *------------------------*/

int cin_ctl_set_fabric_address(struct cin_port* cp, char *ip);
int cin_ctl_set_address(struct cin_port* cp, char *ip, uint16_t reg0, uint16_t reg1);

/*------------------------
 * CIN Register Dump
 *------------------------*/

int cin_ctl_reg_dump(struct cin_port *cp, FILE *fp);

/*------------------------
 * CIN Bias Voltages
 *------------------------*/

int cin_ctl_get_bias_voltages(struct cin_port *cp, float *voltage);
int cin_ctl_set_bias_voltages(struct cin_port *cp, float *voltage);

/* ---------------------------------------------------------------------
 *
 * CIN Data Routines
 *
 * ---------------------------------------------------------------------
 */

int cin_data_init_port(struct cin_port* dp,
                       char* ipaddr, uint16_t port,
                       char* cin_ipaddr, uint16_t cin_port,
                       int rcvbuf);
/*
 * Initialize the data port used for recieveing the UDP packets. A
 * structure of cin_port is modified with the settings. If the strings
 * are NULL and the ports zero then defaults are used.
 */

int cin_data_init(int mode, int packet_buffer_len, int frame_buffer_len,
                  cin_data_callback push_callback, cin_data_callback pop_callback,
                  void *usr_ptr);
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

void cin_data_compute_stats(cin_data_stats_t *stats);
void cin_data_show_stats(FILE *fp,cin_data_stats_t stats);
void cin_data_reset_stats(void);

void cin_data_start_monitor_output(void);
void cin_data_stop_monitor_output(void);

int cin_data_send_magic(void);

int cin_data_set_descramble_params(int rows, int overscan);
int cin_data_get_descramble_params(int *rows, int *overscan, int *xsize, int *ysize);

#ifdef __cplusplus
}
#endif

#endif //__CIN_H__
