
#ifndef TCBLINK_H
#define TCBLINK_H


#ifdef __cplusplus
extern "C" {
#endif


#include "tcb.h"


typedef volatile struct TCBLINK TCBLINK;

struct TCBLINK {
  TCB root;
};

typedef int (*TCBLINK_LIST_CALLBACK)(TCB *tcb);


void tcblink_init(TCBLINK *tcblink);
void tcblink_push(TCBLINK *tcblink, TCB *tcb);

TCB *tcblink_pull_with_priority(TCBLINK *tcblink);
TCB *tcblink_pull_with_tcb(TCBLINK *tcblink, TCB *tcb);
TCB *tcblink_pull_state(TCBLINK *tcblink, STATE_T state);

void tcblink_list(TCBLINK *tcblink, TCBLINK_LIST_CALLBACK callback);


#ifdef __cplusplus
}
#endif

#endif /* TCBLINK_H */

