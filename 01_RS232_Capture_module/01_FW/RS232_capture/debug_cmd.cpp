#include "Arduino.h"
#include "debug_cmd.h"
#include "term_print.h"
#include "version.h"
#include "time.h"

#define MAX_LONG_COMMANDS       256
#define END_OF_LONG_COMMANDS    "ZZZ"
#define END_OF_COMMAND_HELP    	"YYY"

#define ARROW_UP        		    1
#define ARROW_DOWN      		    2

#define COMMAND_HISTORY_MAX     6

#define ESC_ESC_STEP            10

#define ESC_PROCESS_IN_PROGRESS 1
#define ESC_PROCESS_CANCELED    2
#define ESC_PROCESS_FINISHED    3

uint8_t command_history_write_index = 0;
uint8_t command_history_read_index = 0;

int cmd_line_argc;
int cmd_line_index = 0;
int no_of_long_commands = 0;

uint8_t trace_rs232 = 0;

char cmd_line_argv[MAX_CMD_ARGV][MAX_ARGV_LENGTH];
char command_history[COMMAND_HISTORY_MAX][COMMAND_MAX_LENGTH];

extern HardwareSerial ser_debug;
extern HardwareSerial ser_rs232_0;
extern HardwareSerial ser_rs232_1;

extern UART_STR    uart[];

extern char echo_mode;

extern tm timeinfo;

const CommandList LongCommandsSet[]  =
{
  // Long Commands
  // {"ST",  	            LongCmdProcess_ST,      	  "Status print"			      	},
  {"TIME",  	          LongCmdProcess_TIME,      	"Time read cmd"		      		},
  {"RESET",	            LongCmdProcess_RESET,		    "Soft Reset board"				  },
  {"TR",  	            LongCmdProcess_TRACE,      	"Trace control cmd"					},
	{"VER",		            LongCmdProcess_VER,			    "FW version print"					},
  {"?",                 LongCmdProcess_HELP,        "Show Help"         				},
  {END_OF_COMMAND_HELP, LongCmdProcess_NULL,        ""         						      },

	// End Of Commands. Do Not Remove...
  {END_OF_LONG_COMMANDS,  LongCmdProcess_EndOfLongCommands, ""},
};

void LongCmdProcess_TIME(TERM_PORT tpid)
{
  if (cmd_line_argc == 1)
		// term_printf(TPID_DEBUG, "%Y-%B-%d, %H:%M:%S", &timeinfo);
    ser_debug.println(&timeinfo, "%Y-%m-%d %A, %H:%M:%S %Z");
  else
    return;
}

void LongCmdProcess_RESET(TERM_PORT tpid)
{
	UART_STR*	pUART;

	if (tpid >= TPID_MAX)
		return;

	pUART = &uart[tpid];
	
  term_printf(tpid, "board soft reset ...\r\n");
  while (pUART->tx_head != pUART->tx_tail)
  {
    txQ_read(tpid);
  }
  
  delay(300);
  ESP.restart();
}

// trace menu help print
void trace_help_print(TERM_PORT tpid)
{
	term_printf(tpid, "--- TRACE command usage ---\r\n");
	term_printf(tpid, " tr xx yy : RS232 #0..1 Trace [On/Off]\r\n");
}
void LongCmdProcess_TRACE(TERM_PORT tpid)
{
	uint8_t		port;
  uint8_t   val;

	// single parameter
	if (cmd_line_argc == 1)
	{
		trace_help_print(tpid);
	}
	else if (cmd_line_argc == 3)
	{
    port = (uint8_t)strtol(cmd_line_argv[1], NULL, 0);
    val = strcmp (cmd_line_argv[2], "ON") ? 0 : 1;

    trace_rs232 = (port << 4) | val;

    term_printf(tpid, "trace_rs232 : 0x%02x\r\n", trace_rs232);
  }
  else
    return;
}

void LongCmdProcess_VER(TERM_PORT tpid)
{
    term_printf(tpid, "%s\r\n", _VERSION_);
    term_printf(tpid, "%s\r\n\r\n", _RELEASE_INFO_);
}

void LongCmdProcess_HELP(TERM_PORT tpid)
{
  int i = 0;

  term_printf(tpid, "\r\n----- RS232 Caputre module Help Menu -----\r\n");
  term_printf(tpid, "COMMAND\t\tUSAGE\r\n\n");

  //Search the input command from the command set...
  for (i = 0; i < MAX_LONG_COMMANDS; i++)
  {
    // Matched ?
    if (strcmp (END_OF_COMMAND_HELP, LongCommandsSet[i].cmd) == 0)
    {
        break;
    }
    else
    {
        term_printf(tpid, "%s\t\t%s\r\n", LongCommandsSet[i].cmd, LongCommandsSet[i].help);
    }
  }
  term_printf(tpid, "\r\n");
}

void LongCmdProcess_NULL(TERM_PORT tpid)
{
}

void LongCmdProcess_EndOfLongCommands(TERM_PORT tpid)
{
    term_printf (tpid, "End Of Long Commands.. \r\nShould NOT be printed!r\n");
}

void init_debug_history()
{
    no_of_long_commands = GetNumberOfLongCommands();
    memset(command_history, 0x0, sizeof(command_history));
}

int GetNumberOfLongCommands()
{
    int i;

    // Search the input command from the command set...
    for (i = 0; i < MAX_LONG_COMMANDS; i++)
    {
        // Matched ?
        if (strcmp (END_OF_LONG_COMMANDS, LongCommandsSet[i].cmd) == 0)
        {
            return i;
        }
    }

    return 0;
}

void uart_debug_service(TERM_PORT tpid)
{
  char ch = rxQ_read(tpid);

  if (ch != RX_BUFFER_EMPTY)
    debug_command_process(tpid, ch);
}

void debug_command_process(TERM_PORT tpid, char cmd)
{
  UART_STR* pUART;

  pUART = &uart[tpid];

  // 1. ignore bare LF
	//    usually data comes as : data0 + data1 + ... + dataN + CR(0x0d) + LF(0x0a)
	//                                                          |          |
	//                                                      process CMD   ignored
	if ((cmd == 0x0a) && (pUART->cmd_index == 0))
		return;
  // 2. Command Buffer Full, ignore too long command character
	if (pUART->cmd_index + 1 == COMMAND_MAX_LENGTH)
	{
		if (cmd != 0x0d)
            return;
	}
  // 3-1) ESC character(arrow keys) service
	//    arrow key uses 3 byte special format
	//    upper arrow : 0x1b(ESC) + 0x5b([) + 0x41(A)
	//    down  arrow : 0x1b(ESC) + 0x5b([) + 0x42(B)
	//    right arrow : 0x1b(ESC) + 0x5b([) + 0x43(C)
	//    left  arrow : 0x1b(ESC) + 0x5b([) + 0x44(D)
	if ((cmd == 0x1B) && (pUART->ESC_step == 0))
  {
    pUART->ESC_step = 1;
    return;
  }
  // 3-2) ESC + Next character....
	if (pUART->ESC_step)
	{
		// Unless ESC + NOT-Supported Characters....
		if (ArrowKeyProcess (tpid, cmd) != ESC_PROCESS_CANCELED)
			return;
	}

  switch (cmd)
	{
		case 0x0d:
			if (echo_mode)
				term_printf (tpid, "\r\n");
			
			if (pUART->cmd_index == 0)
				return;
			
			// Add one more space for easier parameter input...
			if (pUART->command[pUART->cmd_index - 1] != ' ')
				pUART->command[pUART->cmd_index++] = ' ';
			
			pUART->command[pUART->cmd_index] = '\0';

			if (pUART->cmd_index)
			{
				if (IsDifferentFromPrevCommand(&pUART->command[0]))
				    SaveCommandInHistoryBuffer(&pUART->command[0]);
				else
				    ResetReadIndexOfHistoryBuffer();
			}
			else
			{
				// Restore Buffer Read Index to search again from scratch...
				command_history_read_index = command_history_write_index;
			}
			break;
        
    case 0x08:		// Backspace Key
			if (echo_mode)
				term_printf (tpid, "\b \b");

			pUART->command[pUART->cmd_index] = '\0';
			if (pUART->cmd_index != 0)
				pUART->cmd_index--;
			break;

    default:		// Other Keys    
			if (echo_mode)
				term_printf (tpid, "%c", cmd);

			pUART->command[pUART->cmd_index] = toupper(cmd);
			pUART->cmd_index++;
			break;
	}
	
	// Get Command until ENTER key is pressed...
	if (cmd != 0x0d)
	    return;
	
	// Enter key pressed
	if (pUART->cmd_index == 0)               // Enter pressed
	{
		memset (pUART->command, 0, COMMAND_MAX_LENGTH);
		return;
	}
	
	LongCommandProcess(tpid);
	
	memset (pUART->command, 0x00, sizeof(COMMAND_MAX_LENGTH));
	
	pUART->cmd_index = 0;
}

int LongCommandProcess(TERM_PORT tpid)
{
    return (FindAndRunCommandArgv (tpid));
}

int FindAndRunCommandArgv (TERM_PORT tpid)
{
  int     	i;
  char    	*str_cmd;
	UART_STR	*pUART;

	if (tpid >= TPID_MAX)
		return 0;

	pUART = &uart[tpid];

  // Set Command ARGC and ARGV...
  cmd_line_argc = 0;
  memset (cmd_line_argv, 0, sizeof(cmd_line_argv));

  // First word = Command
	str_cmd = strtok (pUART->command, " ");

  // Command Empty?
  if (str_cmd == NULL)
    return 0;

  // Copy Command...
  strncpy (cmd_line_argv[0], str_cmd, MAX_ARGV_LENGTH-1);

  // Copy Parameters...
  for (i = 1; i < MAX_CMD_ARGV; i++)
  {
    str_cmd = strtok (NULL, " ");

    if (str_cmd == NULL)
        break;

    if (strlen(str_cmd) < MAX_ARGV_LENGTH)
    {
        strcpy (cmd_line_argv[i], str_cmd);
    }
    else
    {
        strncpy (cmd_line_argv[i], str_cmd, MAX_ARGV_LENGTH-1);
    }
  }

  // Set ARGC...
  cmd_line_argc = i;

  // Search the input command from the command set...
  for (i = 0; i < no_of_long_commands; i++)
  {
    // Matched ?
    if (strcmp (cmd_line_argv[0], LongCommandsSet[i].cmd) == 0)
    {
      cmd_line_index = i;

      // Run the command function....
      (*(LongCommandsSet[i].funcPtr))(tpid);

      // Command Processed...
      return 1;
    }
  }

  // Command NOT Processed...
  return 0;
}

// arrow key uses 3 byte special format
// upper arrow : 0x1b(ESC) + 0x5b([) + 0x41(A)
// down  arrow : 0x1b(ESC) + 0x5b([) + 0x42(B)
// right arrow : 0x1b(ESC) + 0x5b([) + 0x43(C)
// left  arrow : 0x1b(ESC) + 0x5b([) + 0x44(D)
int ArrowKeyProcess (TERM_PORT tpid, char cmd)
{
	UART_STR*	pUART;

	if (tpid >= TPID_MAX)
		return ESC_PROCESS_IN_PROGRESS;

	pUART = &uart[tpid];

  if (pUART->ESC_step == 1)
  {
    if (cmd == '[')				// 2nd character == 0x5b([)
    {
        pUART->ESC_step = 2;
        return ESC_PROCESS_IN_PROGRESS;
    }
    else if (cmd == 0x1B)		// 2nd character == 0x1b(ESC) again
    {
    // Only when the command buffer is not empty....
      if (pUART->cmd_index != 0)
      {
        // Clear All Displayed Characters....
        memset (pUART->command, ' ', pUART->cmd_index);
        pUART->command[pUART->cmd_index+1] = '\0';

        if (echo_mode)
          term_printf (tpid, "\r%s\r", pUART->command);

        // Clear the current command...
        memset (pUART->command, 0, COMMAND_MAX_LENGTH-1);
        pUART->cmd_index = 0;

        // Restore Buffer Read Index to search again from scratch...
        command_history_read_index = command_history_write_index;
      }

      pUART->ESC_step = 0;
      return ESC_PROCESS_FINISHED;
    }
    else						// 2nd character == others
    {
      pUART->ESC_step = 0;
      return ESC_PROCESS_CANCELED;
    }
  }
  else if (pUART->ESC_step == 2)	// 3rd character
  {
		char *cmd_buf;

		switch (cmd) 
		{ // the real value
      case 'A':		// Arrow Up,   ESC + '[' + 'A'
      case 'B':		// Arrow Down, ESC + '[' + 'B'
        cmd_buf = GetCommandPtrInHistoryBuffer((cmd == 'A')? ARROW_UP: ARROW_DOWN);

        // Command History buffer Not Empty..
        if (*cmd_buf != '\0')
        {
          // Clear All Displayed Characters....
          // memset (pUART->command, ' ', sizeof (COMMAND_MAX_LENGTH)); //23.03.28 By RHJo, Modify memset size from 4 to 256
          memset (pUART->command, ' ', COMMAND_MAX_LENGTH);
          pUART->command[pUART->cmd_index] = '\0';

          if (echo_mode)
            term_printf (tpid, "\r%s\r", pUART->command);

          // Restore the stored command...
          // memset (pUART->command, 0, sizeof (COMMAND_MAX_LENGTH));
          memset (pUART->command, 0, COMMAND_MAX_LENGTH);
          strcpy(pUART->command, cmd_buf);

          pUART->cmd_index = strlen(pUART->command);

          // Display the restored command...
          if (echo_mode)
            term_printf (tpid, "\r%s", pUART->command);
        }
        break;
      case 'C':		// Arrow Right, ESC + '[' + 'C'
      case 'D':		// Arrow Left,  ESC + '[' + 'D'
        break;
    }

    pUART->ESC_step = 0;
    return ESC_PROCESS_FINISHED;
  }

  // Cancel ESC sequence process...
  pUART->ESC_step = 0;
  return ESC_PROCESS_CANCELED;
}

char *GetCommandPtrInHistoryBuffer (char arrow_key)
{
    // Index to Save the command...
    uint8_t read_index = command_history_read_index;
    int count;

    // Find the next available command in history buffer
    for (count = 0; count < COMMAND_HISTORY_MAX; count ++)
    {
        // Check the stored command...
        switch (arrow_key)
        {
            case ARROW_UP:
            	read_index = (read_index == 0)	? (COMMAND_HISTORY_MAX - 1) : (read_index - 1);
            	break;
            case ARROW_DOWN:
            	read_index = (read_index + 1) % COMMAND_HISTORY_MAX;
            	break;
        }

        // In case the stored command is not empty......
        if (command_history[read_index][0] != '\0')
        {
            command_history_read_index = read_index;
            return command_history[read_index];
        }
    }

    // Command Not Found..
    return command_history[read_index];
}

char IsDifferentFromPrevCommand (char* cmd)
{
    // Index to Save the command...
    uint8_t write_index = command_history_write_index;

    if (write_index == 0)
        write_index = COMMAND_HISTORY_MAX - 1;
    else
        write_index = write_index - 1;

    // Matched ?
    if (strcmp (command_history[write_index], cmd) == 0)
        return 0;
    else
        return 1;
}

void ResetReadIndexOfHistoryBuffer ()
{
    // Index to Save the command...
    uint8_t write_index = command_history_write_index;

    // Save Buffer Read index...
    command_history_read_index = write_index;
}

void SaveCommandInHistoryBuffer (char* cmd)
{
    // Index to Save the command...
    uint8_t write_index = command_history_write_index;

    // Copy the command to the history buffer...
    strcpy(command_history[write_index], cmd);

    // Increment Index for buffer write...
    write_index = (write_index + 1) % COMMAND_HISTORY_MAX;

    // Save Buffer Read index...
    command_history_read_index = write_index;

    // Increment Buffer Write Index...
    command_history_write_index = write_index;

    // Clear the current buffer...
    memset (command_history[write_index], 0, COMMAND_MAX_LENGTH);
}