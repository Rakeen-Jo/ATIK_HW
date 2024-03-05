#ifndef _debug_cmd_h_
#define _debug_cmd_h_

#include "term_print.h"

#ifndef COMMAND_LIST
#define COMMAND_LIST
typedef struct
{
    char        *cmd;
    void        (*funcPtr)(TERM_PORT tpid);
    char        *help;
} CommandList;
#endif	// COMMAND_LIST

void uart_debug_service (TERM_PORT tpid);
void debug_command_process (TERM_PORT tpid, char cmd);
int ArrowKeyProcess (TERM_PORT tpid, char cmd);
char *GetCommandPtrInHistoryBuffer (char arrow_key);
char IsDifferentFromPrevCommand (char* cmd);
void ResetReadIndexOfHistoryBuffer (void);
void SaveCommandInHistoryBuffer (char* cmd);
int LongCommandProcess(TERM_PORT tpid);
int FindAndRunCommandArgv (TERM_PORT tpid);
void init_debug_history(void);
int GetNumberOfLongCommands(void);

void LongCmdProcess_TIME(TERM_PORT tpid);
void LongCmdProcess_RESET(TERM_PORT tpid);
void LongCmdProcess_TRACE(TERM_PORT tpid);
void LongCmdProcess_VER(TERM_PORT tpid);
void LongCmdProcess_HELP(TERM_PORT tpid);
void LongCmdProcess_NULL(TERM_PORT tpid);
void LongCmdProcess_EndOfLongCommands(TERM_PORT tpid);

#endif
