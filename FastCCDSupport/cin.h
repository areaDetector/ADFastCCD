/* vim: set ts=2 sw=2 tw=0 noet : */


/**
 * @file 
 * @author Stuart B. Wilkins <swilkins@bnl.gov> 
 *
 * @section LICENSE
 *  
 *  Copyright (c) 2014, Brookhaven Science Associates, Brookhaven National Laboratory
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met: 
 *  
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer. 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution. 
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  
 *  The views and conclusions contained in the software and documentation are those
 *  of the authors and should not be interpreted as representing official policies, 
 *  either expressed or implied, of the FreeBSD Project.
 *  
 *  @section DESCRIPTION
 *
 *  Header file for CIN communications
 *
 */

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

#define CIN_OK                             0
#define CIN_ERROR                          -1
#define CIN_CTL_MSG_OK                     0
#define CIN_CTL_MSG_MINOR                  1
#define CIN_CTL_MSG_MAJOR                  2

#define CIN_CTL_IP                         "192.168.1.207"
#define CIN_CTL_CIN_PORT                   49200
#define CIN_CTL_BIND_PORT                  50200
#define CIN_CTL_FRMW_CIN_PORT              49202
#define CIN_CTL_FRMW_BIND_PORT             50202
#define CIN_CTL_RCVBUF                     10  // Mb 

#define CIN_CTL_MAX_READ_TRIES             5
#define CIN_CTL_MAX_WRITE_TRIES            5
#define CIN_CTL_WRITE_SLEEP                100 // microsecs
#define CIN_CTL_READ_SLEEP                 100 // microsecs
#define CIN_CTL_BIAS_SLEEP                 100000 // microseconds
#define CIN_CTL_FO_SLEEP                   500000 // microseconds
#define CIN_CTL_CONFIG_SLEEP               100 // microseconds
#define CIN_CTL_DCO_SLEEP                  1000000 // microseconds
#define CIN_CTL_FCLK_SLEEP                 200000 // microseconds
#define CIN_CTL_STREAM_CHUNK               512
#define CIN_CTL_STREAM_SLEEP               5

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
#define CIN_CTL_FCLK_156_C                 0x0006

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

#define CIN_CTL_FO_REG1                    0x821D
#define CIN_CTL_FO_REG2                    0x821E
#define CIN_CTL_FO_REG3                    0x821F

#define CIN_DATA_IP                        "10.0.5.207"
#define CIN_DATA_BIND_PORT                 49201
#define CIN_DATA_CIN_PORT                  49203
#define CIN_DATA_FRAME_BUFFER_LEN          1000
#define CIN_DATA_PACKET_BUFFER_LEN         10000
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
#define CIN_DATA_RCVBUF                    (100*1024*1024)  // Bytes

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
 * Definitions for CIN BIAS SETTINGS
 *
 * -------------------------------------------------------------------------------
 */

#define CIN_CTL_NUM_BIAS                   20
#define CIN_CTL_BIAS_OFFSET                0x0030  /**< Offset in address to read bias */

#define CIN_CTL_BIAS_POSH                  0
#define CIN_CTL_BIAS_NEGH                  1
#define CIN_CTL_BIAS_POSRG                 2
#define CIN_CTL_BIAS_NEGRG                 3
#define CIN_CTL_BIAS_POSSW                 4
#define CIN_CTL_BIAS_NEGSW                 5
#define CIN_CTL_BIAS_POSV                  6
#define CIN_CTL_BIAS_NEGV                  7
#define CIN_CTL_BIAS_POSTG                 8
#define CIN_CTL_BIAS_NEGTG                 9
#define CIN_CTL_BIAS_POSVF                 10
#define CIN_CTL_BIAS_NEGVF                 11
#define CIN_CTL_BIAS_NEDGE                 12
#define CIN_CTL_BIAS_OTG                   13
#define CIN_CTL_BIAS_VDDR                  14
#define CIN_CTL_BIAS_VDD_OUT               15
#define CIN_CTL_BIAS_BUF_BASE              16
#define CIN_CTL_BIAS_BUF_DELTA             17
#define CIN_CTL_BIAS_SPARE1                18
#define CIN_CTL_BIAS_SPARE2                19

/* ---------------------------------------------------------------------
 *
 * MACROS and functions for debugging
 *
 * ---------------------------------------------------------------------
 */

// Global variables to set debug levels 

extern int _debug_print_flag;
extern int _error_print_flag;
void cin_set_debug_print(int debug);
void cin_set_error_print(int error);

#define DEBUG_PRINT(fmt, ...) \
  if(_debug_print_flag) { fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, __VA_ARGS__); }

#define DEBUG_COMMENT(fmt)\
  if(_debug_print_flag) { fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__); }

#define ERROR_COMMENT(fmt)\
  if(_error_print_flag) { fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__); }

#define ERROR_PRINT(fmt, ...) \
  if(_error_print_flag) { fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, __VA_ARGS__); }

/* ---------------------------------------------------------------------
 *
 * Global datastructures
 *
 * ---------------------------------------------------------------------
 */

#define CIN_CONFIG_MAX_STRING 40

typedef struct {
  void *data;
  void *head;
  void *tail;
  void *end;
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
  pthread_barrier_t barrier;
} cin_ctl_listener_t;

typedef struct cin_port {
  int sockfd;
  struct timeval tv;
  struct sockaddr_in sin_srv; /* server info */
  struct sockaddr_in sin_cli; /* client info (us!) */
  socklen_t slen; /* for recvfrom() */
} cin_port_t;

#define CIN_CONFIG_MAX_TIMING_DATA       880  /**< Max = 55 per state, 16 states */
#define CIN_CONFIG_MAX_TIMING_MODES      10  /**< states max */
#define CIN_CONFIG_MAX_TIMING_NAME       40  /**< Max characters for timing name */

typedef struct cin_config_timing {
  uint16_t *data;                           /**< Pointer to timing data */
  int data_len;                             /**< timing data length */
  char name[CIN_CONFIG_MAX_TIMING_NAME];    /**< String for config name */
  int rows;                                 /**< Rows for this timing setup */
  int cols;                                 /**< Cols for this timing setup */
  int overscan;                             /**< Number of overscan cols for this setup */
  int fclk_freq;                            /**< FCLK Frequency to use */
  int framestore;                           /**< Flag (not zero means framestore */
} cin_config_timing_t;

typedef struct cin_ctl {
  // Store IP Address information
  char *addr;
  char *bind_addr;
  int port;
  int bind_port;
  int sport;
  int bind_sport;

  // TCP/IP Port Information
  cin_port_t ctl_port;
  cin_port_t stream_port;

  // Config information 
  cin_config_timing_t timing[CIN_CONFIG_MAX_TIMING_MODES];
  int timing_num;
  cin_config_timing_t *current_timing;

  // FCLK info for absolute time
  float fclk_time_factor;               /**< In micro seconds */

  // Mutex for threaded access
  cin_ctl_listener_t *listener;
  pthread_mutex_t access; 
  pthread_mutexattr_t access_attr;

  // Callback fot status info
  void (*msg_callback)(const char*, int, void*);
  void *msg_callback_ptr;
} cin_ctl_t;


typedef struct cin_data_frame {
  uint16_t *data;
  uint16_t number;
  struct timespec timestamp;
  int size_x;
  int size_y;
} cin_data_frame_t;

typedef struct cin_data_stats {
  // Frame data

  int last_frame;
  double framerate;

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

typedef struct cin_data_threads {
  pthread_t thread_id;
  pthread_barrier_t barrier;
  int started;
} cin_data_threads_t;

typedef struct cin_data_callbacks {
  void* (*push) (cin_data_frame_t *, void* usr_ptr);
  void* (*pop)  (cin_data_frame_t *, void* usr_ptr);
  cin_data_frame_t *frame;
  void *usr_ptr; // User container
} cin_data_callbacks_t;

typedef struct {
  uint32_t *map;
  int      size_x; // COLS
  int      size_y; // ROWS
  int      overscan;
  int      rows;
} cin_data_descramble_map_t;


typedef struct cin_data {

  /* FIFO Elements */
  fifo *packet_fifo;  
  fifo *frame_fifo;

  /* Thread Information */

  cin_data_threads_t listen_thread;
  cin_data_threads_t assembler_thread;
  cin_data_threads_t descramble_thread;
  pthread_mutex_t descramble_mutex;
  pthread_mutex_t stats_mutex;
  pthread_mutex_t framestore_mutex;

  /* Callbacks Buffer */

  cin_data_callbacks_t callbacks;

  /* Interface */
  char *addr;
  char *bind_addr;
  int port;
  int bind_port;
  int recv_buf;
  cin_port_t dp;

  /* Statistics */
  struct timespec framerate;
  unsigned long int dropped_packets;
  unsigned long int mallformed_packets;
  uint16_t last_frame;

  /* Current Descramble Map */
  cin_data_descramble_map_t map;

  /* Framestore mode info */
  int framestore_mode;
  struct timespec framestore_trigger;
  int framestore_counter;

} cin_data_t;
// Callback functions

// FUnction pointer definitions

typedef void (*cin_data_callback) (cin_data_frame_t *, void *usr_ptr);
typedef void (*cin_ctl_msg_callback)(const char*, int, void*);

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
  uint16_t base_board_id;
  uint16_t base_serial_no;
  uint16_t base_fpga_ver;
  uint16_t fabric_board_id;
  uint16_t fabric_serial_no;
  uint16_t fabric_fpga_ver;
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

/** @defgroup cin_ctl_init Cin Control Initialization Routines
 *
 * @{
 */

/*--------------------------------------------------------------------------------------------------------
 * 
 * Initialization Routines
 *
 *--------------------------------------------------------------------------------------------------------*/


/**
 * Initialize the cin control library
 *
 * Initialize the control structures and communications with the CIN via the control
 * interface. This function opens the UDP ports and starts a listening thread to 
 * recieve packets from the CIN.
 *
 * @param cin handle to cin library
 * @param addr ip address of CIN base address
 * @param port UDP port of cin 
 * @param sport stream output UDP port of cin
 * @param bind_addr ip address to bind to
 * @param bind_port input udp port of cin 
 * @param bind_sport stream input udp port of cin
 *
 * @return Returns 0 on sucsess non-zero if error
 */
int cin_ctl_init(cin_ctl_t *cin, 
                 char *addr, uint16_t port, uint16_t sport, 
                 char *bind_addr, uint16_t bind_port, uint16_t bind_sport);

/*!
 * Destroy (close) the cin control library
 *
 * Close connections, free memory and exit library
 *
 * @param cin handle to cin library
 *
 * @return Returns 0 on sucsess non-zero if error
 */
int cin_ctl_destroy(cin_ctl_t *cin);

/*!
 * Register a function to recieve status messages
 *
 * Close connections, free memory and exit library
 *
 * @param cin handle to cin library
 * @param callback function pointer to callback function
 * @param ptr user pointer which is passed to callback routine
 *
 */
void cin_ctl_set_msg_callback(cin_ctl_t *cin, cin_ctl_msg_callback callback, void *ptr);

/*!
 * Send a magic packet to the CIN to initialize data
 *
 * @param cin handle to cin library
 *
 * @return Returns 0 on sucsess non-zero if error
 */
int cin_data_send_magic(cin_data_t *cin);
/** @} */

/*--------------------------------------------------------------------------------------------------------
 * 
 * CIN Read Write Routines
 *
 *--------------------------------------------------------------------------------------------------------*/

/** @defgroup cin_ctl_rw Cin Control Read/Rwite Routines
 *
 * @{
 */

/*!
 * Read register from CIN
 *
 * @param cin handle to cin library
 * @param reg register to read
 * @param val variable to read value of register to
 * @param wait if non-zero, wait a predefined time before read command (for i2c)
 *
 * @return Returns 0 on sucsess non-zero if error
 */
int cin_ctl_read(cin_ctl_t *cin, uint16_t reg, uint16_t *val, int wait);
/*!
 * Write register to CIN
 *
 * @param cin handle to cin library
 * @param reg register to write to
 * @param val value to write to register
 * @param wait if non-zero
 *
 * Write register value to CIN. If wait is non-zero then wait a sleep time of i
 * CIN_CTL_WRITE_SLEEP before releasing the mutex to add flow control to the cin.
 *
 * @return Returns 0 on sucsess non-zero if error
 */
int cin_ctl_write(cin_ctl_t *cin, uint16_t reg, uint16_t val, int wait);
/*!
 * Write stream data to CIN
 *
 * @param cin handle to cin library
 * @param val array of values to write
 * @param size size of array pointed to by val
 *
 * Write stream data to cin in form of 16 bit array.
 *
 * @return Returns 0 on sucsess non-zero if error
 */
int cin_ctl_stream_write(cin_ctl_t *cin, unsigned char* val,int size);
/*!
 * Write register to CIN with readback verification
 *
 * @param cin handle to cin library
 * @param reg register to write to
 * @param val value to write to register
 *
 * Write register value to CIN. Follow write with read of register and compare value.
 * CIN_CTL_WRITE_SLEEP before releasing the mutex to add flow control to the cin.
 *
 * @return Returns 0 on sucsess non-zero if error
 */
int cin_ctl_write_with_readback(cin_ctl_t *cin, uint16_t reg, uint16_t val);

/** @} */

/** @defgroup cin_ctl_power Cin Power Routines
 *
 * These routine  control power to the CIN for the Frame FPGA and the Front Panel
 * 
 * @{
 */

/* ----------------------------------------------------------------------------*/
/**
 * @Breif Control CIN Frame FPGA Power
 *
 * Turn on and off the frame FPGA power. If pwr is 0 then turn off power. If
 * pwr is 1 turn on power.
 *
 * @Param cin handle to cin library
 * @Param pwr power status 
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_ctl_pwr(cin_ctl_t *cin, int pwr);

/* ----------------------------------------------------------------------------*/
/**
 * @Breif Control CIN Front Panel Power
 *
 * Turn on and off the CIN Front Panel power. The front panel power powers either 
 * the fiber optic modules or the LVDS lines to the camera. If pwr is 0 then turn
 * off FP power and if pwr is 1 turn on the FP power. 
 *
 * @Param cin handle to cin library
 * @Param pwr power status
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_ctl_fp_pwr(cin_ctl_t *cin, int pwr);

/* ----------------------------------------------------------------------------*/
/**
 * @Breif Control Fiber Optic Interface Test Pattern
 *
 * Turn on and off the fiber optic test pattern. The FO modules transmit a test
 * pattern to indicate that communication with the data modules is correct. This
 * routine turns on and off the modules. If on_off is 0 then the FO test pattern is
 * turned off, and if on_off is 1 then the FO test pattern is turned on. Note: this 
 * routine manipulates the FCRIC mask, so you may need to reconfigure the fCRICs 
 * after using it. 
 *
 * @Param cin handle to cin library
 * @Param on_off test pattern status
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_ctl_fo_test_pattern(cin_ctl_t *cin, int on_off);

/** @} */

/*--------------------------------------------------------------------------------------------------------
 * 
 * CIN Configuration-Status
 *
 *--------------------------------------------------------------------------------------------------------*/

/** @defgroup cin_ctl_firmware CIN Firmware Upload Routines
 * These routines control the upload of firmware to the frame FPGA in the CIN. The firmware can be
 * uploaded using either a external file or an array of unsigned char (bytes). The function
 * cin_ctl_load_firmware() loads the pre-compiled firmware.
 * @{
 */
/* ----------------------------------------------------------------------------*/
/**
 * @Breif Load the pre-compiled frame FPGA firmware
 *
 * @Param cin handle to cin library
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_ctl_load_firmware(cin_ctl_t *cin);

/* ----------------------------------------------------------------------------*/
/**
 * @Breif Load the frame FPGA firmware from file
 *
 * @Param cin handle to cin library
 * @Param filename file containing the binary FPGA firmware
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_ctl_load_firmware_file(cin_ctl_t *cin, char *filename);

/* ----------------------------------------------------------------------------*/
/**
 * @Breif Load the frame FPGA firmware from char array
 *
 * @Param cin handle to cin library
 * @Param data array of binary FPGA firmware
 * @Param data_len length of binary data
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_ctl_load_firmware_data(cin_ctl_t *cin, unsigned char *data, int data_len);

/* ----------------------------------------------------------------------------*/
/**
 * @Breif Load FPGA config file
 *
 * Upload a FPGA config file to the CIN. This file is a simple file with each line
 * containing a 4 digit hex value for the register location and a 4 digit hex 
 * value for the value to be written to the register. 
 *
 * @Param cin handle to the cin library
 * @Param filename filename to load
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_ctl_load_config(cin_ctl_t *cin,const char *filename);

/** @} */

/** @defgroup cin_ctl_fclk CIN FCLK Configuration Routines
 * These routines configure the Internal Frame FPGA Clock (FCLK) frequency.
 * @{
 */
/* ----------------------------------------------------------------------------*/
/**
 * @Breif Get the current frame FPGA clock frequency
 *
 * @Param cin handle to cin library
 * @Param clkfreq clock frequency
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_ctl_get_fclk(cin_ctl_t *cin, int *clkfreq);

/* ----------------------------------------------------------------------------*/
/**
 * @Breif Set the frame FPGA clock frequency
 *
 * @Param cin handle to cin library
 * @Param clkfreq clock frequency to set
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_ctl_set_fclk(cin_ctl_t *cin, int clkfreq);
/** @} */


/** @defgroup cin_ctl_status CIN Status Routines
 * Status Routines
 * @{
 */
int cin_ctl_get_cfg_fpga_status(cin_ctl_t *cin, uint16_t *_val);
int cin_ctl_get_id(cin_ctl_t *cin, cin_ctl_id_t *_val);
int cin_ctl_get_dcm_status(cin_ctl_t *cin, uint16_t *_val);
int cin_ctl_get_power_status(cin_ctl_t *cin, int full, int *pwr, cin_ctl_pwr_mon_t *values);
/** @} */


/** @defgroup cin_ctl_bias CIN Control Bias Routines
 * Initialization group
 * @{
 */
int cin_ctl_set_bias(cin_ctl_t *cin,int val);
int cin_ctl_get_bias(cin_ctl_t *cin, int *val);
int cin_ctl_set_bias_regs(cin_ctl_t * cin, uint16_t *vals, int verify);
int cin_ctl_get_bias_regs(cin_ctl_t * cin, uint16_t *vals);
int cin_ctl_set_bias_voltages(cin_ctl_t *cin, float *voltage, int verify);
int cin_ctl_get_bias_voltages(cin_ctl_t *cin, float *voltage, uint16_t *regs);
/** @} */


/** @defgroup cin_ctl_timing CIN Control Timing Routines
 * Timing setup group
 * @{
 */
int cin_ctl_set_timing_regs(cin_ctl_t *cin, uint16_t *vals, int vals_len);
int cin_ctl_get_timing_regs(cin_ctl_t *cin, uint16_t *vals, int vals_len);
/** @} */

int cin_ctl_get_camera_pwr(cin_ctl_t *cin, int *val);
int cin_ctl_set_camera_pwr(cin_ctl_t *cin, int val);
int cin_ctl_set_clocks(cin_ctl_t *cin,int val);
int cin_ctl_get_clocks(cin_ctl_t *cin, int *val);
int cin_ctl_set_trigger(cin_ctl_t *cin,int val);
int cin_ctl_get_trigger(cin_ctl_t *cin, int *val);
int cin_ctl_set_focus(cin_ctl_t *cin, int val);
int cin_ctl_get_focus(cin_ctl_t *cin, int *val);
int cin_ctl_get_triggering(cin_ctl_t *cin, int *trigger);
int cin_ctl_int_trigger_start(cin_ctl_t *cin, int nimages);
int cin_ctl_int_trigger_stop(cin_ctl_t *cin);
int cin_ctl_ext_trigger_start(cin_ctl_t *cin, int trigger_mode);
int cin_ctl_ext_trigger_stop(cin_ctl_t *cin);
int cin_ctl_set_exposure_time(cin_ctl_t *cin,float e_time);
int cin_ctl_set_trigger_delay(cin_ctl_t *cin,float t_time);
int cin_ctl_set_cycle_time(cin_ctl_t *cin,float ftime);
int cin_ctl_frame_count_reset(cin_ctl_t *cin);
int cin_ctl_set_mux(cin_ctl_t *cin, int setting);
int cin_ctl_get_mux(cin_ctl_t *cin, int *setting);
int cin_ctl_set_fcric_clamp(cin_ctl_t *cin, int clamp);
int cin_ctl_set_fcric_gain(cin_ctl_t *cin, int gain);
int cin_ctl_set_fcric_regs(cin_ctl_t *cin, uint16_t *reg, int num_reg);
int cin_ctl_set_fcric(cin_ctl_t *cin);
int cin_ctl_set_fabric_address(cin_ctl_t *cin, char *ip);

int cin_ctl_bias_dump(cin_ctl_t *cin, FILE *fp);
int cin_ctl_reg_dump(cin_ctl_t *cin, FILE *fp);

/*------------------------
 * CIN Config File
 *------------------------*/

/** @defgroup cin_ctl_timing CIN Control Timing Routines
 * Timing setup group
 * @{
 */

int cin_config_read_file(cin_ctl_t *cin, const char *file);

/* ----------------------------------------------------------------------------*/
/**
 * @Breif Get the name of the timing config options 
 *
 * @Param cin handle to cin library
 * @Param num number of timing option
 * @Param name char array of name
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_config_get_timing_name(cin_ctl_t *cin, int num, char **name);

/* ----------------------------------------------------------------------------*/
/**
 * @Breif Get the name of the current timing mode
 *
 * @Param cin handle to cin library
 * @Param name char array of name 
 *
 * @Returns CIN_OK on sucsess, CIN_ERROR on an error 
 */
/* ----------------------------------------------------------------------------*/
int cin_config_get_current_timing_name(cin_ctl_t *cin, char **name);
/** @} */

void cin_ctl_message(cin_ctl_t *cin, const char *message, int severity);
/* ---------------------------------------------------------------------
 *
 * CIN Data Routines
 *
 * ---------------------------------------------------------------------
 */

/** @defgroup cin_data_init CIN Data Initialization Routines
 * Initialization group
 * @{
 */

/** Initialize the cin data library
 *
 * Initialize the data handeling routines and start the threads for listening.
 *
 * @param cin Handle to cin data library
 * @param addr IP-Address of cin (if NULL defaults to standard)
 * @param port UDP Port of CIN 
 * @param bind_addr IP-Address to bind to (if NULL binds to 0.0.0.0)
 * @param bind_port UDP Port of host
 * @param rcvbuf TCP/IP Kernel recieve buffer size
 * @param packet_buffer_len Length of packet buffer fifo (in units number of packets)
 * @param frame_buffer_len Length of frame (assembler) buffer fifo (in units of number of frames)
 * @param push_callback This function is called when a data structure is needed
 * @param pop_callback This function is called when an image has been processed
 * @param usr_ptr Pointer passed to callback functions
 *
 */
int cin_data_init(cin_data_t *cin, 
                  char* addr, uint16_t port, char* bind_addr, uint16_t bind_port, int rcvbuf,
                  int packet_buffer_len, int frame_buffer_len,
                  cin_data_callback push_callback, cin_data_callback pop_callback, void *usr_ptr);
/** Close the cin data library and cleanup
 *
 * Stop all the processing threads and join them to the main thread. This function blocks until all
 * threads have joined the main thread (program). This should be called to clean up the library before
 * the program is exited 
 *
 * @param cin Handle to cin data library
 */
void cin_data_destroy(cin_data_t *cin);
/**
 * @}
 */

/*--------------------------------------------------------------------------------------------------------
 * 
 * CIN DATA Software trigger and framestore modes
 *
 *--------------------------------------------------------------------------------------------------------*/

/** @defgroup cin_data_framestore CIN Data Framestore Functions
 * Framestore Group
 * @{
 */

/** Send a framestore (software) trigger
 * 
 * Send a software trigger to the CIN by timestamping the request time and allow images to be
 * processed when recieved after this time. The count option sets the number of frames
 * to trigger. A value of -1 indicated that the trigger should not count images but run indefinately
 * after the trigger has occured.
 *
 * @param cin handle to the cin_data library
 * @param count number of frames to trigger
 *
 */
void cin_data_framestore_trigger(cin_data_t *cin, int count);

/** Enable framestore skip mode
 *
 * Enable the framestore skip mode. This function should be called before hardware triggering the camera.
 * This causes the data processing to skip @param count frames from the first images to be read. This is
 * usually done to stop the first few frames from being over exposed. 
 *
 * @param cin handle to the cin_data library
 * 
 */
void cin_data_framestore_skip(cin_data_t *cin, int count);

/** Get the value of the framestore counter
 * 
 * Return the number of frames in the framestore counter. In trigger mode, this returns the number of frames
 * to go. In skip mode, this returns the number of frames that have to be skipped.
 *
 * @param cin handle to the cin_data library
 * @returns Number of frames to go in trigger 
 * 
 */
int cin_data_get_framestore_counter(cin_data_t *cin);

/** Disable the framestore modes
 *
 * This function disables the framestore modes (software trigger and skip). If the camera is hardware triggering
 * then the images will start to be processed. 
 *
 * @param cin Handle to the cin library
 * 
 */
void cin_data_framestore_disable(cin_data_t *cin);

/** Enable the framestore trigger mode
 *
 * This function enables the framestore trigger mode. It cases the images to not be processed pending a call
 * to the function to (software) trigger the camera.
 *
 * @param cin Handle to the cin library
 * 
 */
void cin_data_framestore_trigger_enable(cin_data_t *cin);

/** @} */ // End of cin_data_framestore group

struct cin_data_frame* cin_data_get_next_frame(cin_data_t *cin);
void cin_data_release_frame(cin_data_t *cin, int free_mem);

struct cin_data_frame* cin_data_get_buffered_frame(void);
void cin_data_release_buffered_frame(void);

void cin_data_compute_stats(cin_data_t *cin, cin_data_stats_t *stats);
void cin_data_show_stats(FILE *fp,cin_data_stats_t stats);
void cin_data_reset_stats(cin_data_t *cin);


int cin_data_set_descramble_params(cin_data_t *cin, int rows, int overscan);
void cin_data_get_descramble_params(cin_data_t *cin, int *rows, int *overscan, int *xsize, int *ysize);

int cin_com_boot(cin_ctl_t *cin_ctl, cin_data_t *cin_data, const char *mode);
int cin_com_set_timing(cin_ctl_t *cin_ctl, cin_data_t *cin_data,  const char *name);
int cin_ctl_upload_bias(cin_ctl_t *cin);
#ifdef __cplusplus
}
#endif

#endif //__CIN_H__
