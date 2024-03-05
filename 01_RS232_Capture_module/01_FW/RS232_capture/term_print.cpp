#include "Arduino.h"
#include "term_print.h"

extern HardwareSerial ser_debug;
extern HardwareSerial ser_rs232_0;
extern HardwareSerial ser_rs232_1;

//---------------------------------------------
// UART serial comm. Tx/Rx buffer assign
//
//---------------------------------------------------------
// serial comm. : DEBUG UART port define
char  ser_debug_tx_buf[SER_DEBUG_TX_BUF_SIZE];
char  ser_debug_rx_buf[SER_DEBUG_RX_BUF_SIZE];
//---------------------------------------------------------
// serial comm. : RS232#0 UART port define
char  ser_rs232_0_tx_buf[SER_RS232_0_TX_BUF_SIZE];
char  ser_rs232_0_rx_buf[SER_RS232_0_RX_BUF_SIZE];
//---------------------------------------------------------
// serial comm. : RS232#1 PC UART port define
char  ser_rs232_1_tx_buf[SER_RS232_1_TX_BUF_SIZE];
char  ser_rs232_1_rx_buf[SER_RS232_1_RX_BUF_SIZE];

UART_STR    uart[TPID_MAX];

char echo_mode = 1;

void init_terminal_buffers()
{
  memset(&uart[TPID_UART_1],  0, sizeof(UART_STR));
  memset(&uart[TPID_UART_2],  0, sizeof(UART_STR));
  memset(&uart[TPID_UART_3],  0, sizeof(UART_STR));
  
  uart[TPID_UART_1].pTx_buf       = &ser_debug_tx_buf[0];
  uart[TPID_UART_1].pRx_buf       = &ser_debug_rx_buf[0];
  uart[TPID_UART_1].tx_buf_size   = SER_DEBUG_TX_BUF_SIZE;
  uart[TPID_UART_1].rx_buf_size   = SER_DEBUG_RX_BUF_SIZE;
  
  uart[TPID_UART_2].pTx_buf     = &ser_rs232_0_tx_buf[0];
  uart[TPID_UART_2].pRx_buf     = &ser_rs232_0_rx_buf[0];
  uart[TPID_UART_2].tx_buf_size   = SER_RS232_0_TX_BUF_SIZE;
  uart[TPID_UART_2].rx_buf_size   = SER_RS232_0_RX_BUF_SIZE;
  
  uart[TPID_UART_3].pTx_buf     = &ser_rs232_1_tx_buf[0];
  uart[TPID_UART_3].pRx_buf     = &ser_rs232_1_rx_buf[0];
  uart[TPID_UART_3].tx_buf_size   = SER_RS232_1_TX_BUF_SIZE;
  uart[TPID_UART_3].rx_buf_size   = SER_RS232_1_RX_BUF_SIZE;
}

void uart_tx_service()
{  
  txQ_read(TPID_DEBUG);
  txQ_read(TPID_RS232_0);
  txQ_read(TPID_RS232_1);
}

int term_printf(TERM_PORT tpid, const char *format, ...)
{
	int length = 0;
	char logBuf[COMMAND_MAX_LENGTH] = {0};

	va_list args;
	va_start(args, format);

	length = vsnprintf(logBuf, sizeof(logBuf), format, args);

	if (length >= COMMAND_MAX_LENGTH)
		logBuf[COMMAND_MAX_LENGTH-1] = '\0';

  int count;
  for (count = 0; count < length; count++)
  {
      txQ_write (tpid, logBuf[count]);
  }

	va_end(args);
	return 0;
}

void rxQ_write(TERM_PORT tpid, char ch)
{
  uint16_t  head;
  uint16_t  head_next;
  uint16_t  tail;
  UART_STR* pUART;
  
  if (tpid >= TPID_MAX)
    return;

  pUART = &uart[tpid];

  head = pUART->rx_head;
  tail = pUART->rx_tail;
  head_next = (head + 1) % pUART->rx_buf_size;

  // 1. check RxQ full
  if (head_next == tail)    // RxQ full, current tx data will be lost
  {
    term_printf(tpid, "UART[%d] Rx Buffer Full\r\n", tpid);
  }
  else
  {
    pUART->pRx_buf[head] = ch;
    pUART->rx_head = head_next;
  }
}

char rxQ_read(TERM_PORT tpid)
{
  UART_STR* pUART;
  char  rxChar = RX_BUFFER_EMPTY;

  if (tpid >= TPID_MAX)
    return rxChar;

  pUART = &uart[tpid];
  
  // RX Buffer NOT Empty!!!!!!
  if (pUART->rx_head != pUART->rx_tail)
  {
    rxChar = pUART->pRx_buf[pUART->rx_tail];
    pUART->rx_tail = (pUART->rx_tail + 1) % pUART->rx_buf_size;
  }

  return rxChar;
}

void txQ_write(TERM_PORT tpid, char ch)
{
	UART_STR*	pUART;
	uint16_t	head;
	uint16_t	head_next;
	uint16_t	tail;
	
	if (tpid >= TPID_MAX)
		return;

	pUART = &uart[tpid];
  head = pUART->tx_head;
	tail = pUART->tx_tail;
	head_next = (head + 1) % pUART->tx_buf_size;

	// 1. check TxQ full
  if (head_next == tail) 		// TxQ full, current tx data will be lost
  return;

	// 2. write data to TxQ
	pUART->pTx_buf[head] = ch;
	pUART->tx_head = head_next;
}

void txQ_read(TERM_PORT tpid)
{
  UART_STR* pUART;
  uint16_t  head;
  uint16_t  tail;
  
  if (tpid >= TPID_MAX)
    return;

  pUART = &uart[tpid];
  head = pUART->tx_head;
  tail = pUART->tx_tail;

  // 1. check TxQ empty,
  if (head == tail)     // TxQ empty, just return
      return;
  
  if (tpid == TPID_DEBUG)
    ser_debug.print(pUART->pTx_buf[pUART->tx_tail]);
  else if (tpid == TPID_RS232_0)
    ser_rs232_0.print(pUART->pTx_buf[pUART->tx_tail]);
  else //if (tpid == TPID_RS232_1)
    ser_rs232_1.print(pUART->pTx_buf[pUART->tx_tail]);

  pUART->tx_tail = (pUART->tx_tail + 1) % pUART->tx_buf_size;
}
