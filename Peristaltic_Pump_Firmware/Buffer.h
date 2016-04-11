#ifndef BUFFER_H
#define BUFFER_H

/*===== Variables =====*/
// Number of commands that can be stored in the command buffer
#define BUFSIZE 4
// Maximum command length
#define MAX_CMD_SIZE 96

extern int buflen;
extern int bufindr;
extern int bufindw;


/*===== Routines =====*/
void get_command();
void process_commands();



#endif // BUFFER_H
