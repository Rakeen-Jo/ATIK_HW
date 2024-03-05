#ifndef _term_print_h_
#define _term_print_h_

#define TPID_DEBUG    TPID_UART_1
#define TPID_RS232_0  TPID_UART_2
#define TPID_RS232_1  TPID_UART_3

#define SER_DEBUG_TX_BUF_SIZE     512
#define SER_DEBUG_RX_BUF_SIZE     512

#define SER_RS232_0_TX_BUF_SIZE   512
#define SER_RS232_0_RX_BUF_SIZE   512

#define SER_RS232_1_TX_BUF_SIZE   512
#define SER_RS232_1_RX_BUF_SIZE   512

#define RX_BUFFER_EMPTY     		  0

#define COMMAND_MAX_LENGTH  	  	256
#define LINE_MAX_LENGTH  			    2048

#define MAX_CMD_ARGV        	  	20
#define MAX_ARGV_LENGTH     	  	20

typedef enum
{
  TPID_UART_1     = 0x00,   // PC debugging port
  TPID_UART_2     = 0x01,   // RS232 #0
  TPID_UART_3     = 0x02,   // RS232 #1
  TPID_MAX
}TERM_PORT;

typedef struct
{
  char        *pTx_buf;
  uint16_t    tx_buf_size;
  uint16_t    tx_head;
  uint16_t    tx_tail;
                    
  char        *pRx_buf;
  uint16_t    rx_buf_size;
  uint16_t    rx_head;
  uint16_t    rx_tail;

  uint32_t		rx_err_flag;				
	uint8_t			rx_data;					
	uint8_t			rx_tmp_buf[2];
	
	uint16_t		cmd_index;
	char				command[COMMAND_MAX_LENGTH];
	uint8_t			ESC_step;						// ESC charater parsing step
}UART_STR;

void init_terminal_buffers();
void uart_tx_service();
void rxQ_write(TERM_PORT tpid, char ch);
char rxQ_read(TERM_PORT tpid);
void txQ_write(TERM_PORT tpid, char ch);
void txQ_read(TERM_PORT tpid);
int term_printf(TERM_PORT tpid, const char *format, ...);

#endif
