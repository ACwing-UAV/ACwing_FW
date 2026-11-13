/**
 * @file  rc_task.h
 * @brief RcTask: the RC receiver, and nothing else. Reads the receiver UART, feeds rc_input, follows the
 *        parameters rcProtocol / rcMap (reopens the UART when the protocol changes). Built only with APP_RC_UART.
 */
#ifndef RC_TASK_H
#define RC_TASK_H

void rc_task_start(void);

#endif /* RC_TASK_H */
