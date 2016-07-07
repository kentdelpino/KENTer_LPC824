
#include "tcblink.h"


void tcblink_init(TCBLINK *tcblink) {

} /* tcblink_init */


void tcblink_push(TCBLINK *tcblink, TCB *tcb) {
  
  TCB *root = &(tcblink->root);
  TCB *work = root;
 // Just check   
  if (tcb == TCB_NULL)
    return;
 // Find free slot, to insert it
  while (work->next != TCB_NULL)
    work = work->next;
 // Insert it
  work->next = tcb;
  tcb->prev = work;
 // There is a new free slot 
  tcb->next = TCB_NULL;
} /* tcblink_push */


TCB *tcblink_pull_with_priority(TCBLINK *tcblink) {

  unsigned int highest_priority = 0;
  TCB *highest_tcb = TCB_NULL;
  TCB *w = tcblink->root.next;

  while (w != TCB_NULL) {
    if (highest_priority < w->thread.info.priority) {
      highest_priority = w->thread.info.priority;
      highest_tcb = w;
     }
    w = w->next;
   }

  if (highest_tcb != TCB_NULL) {
    TCB *p = highest_tcb->prev;
    TCB *n = highest_tcb->next;

    if (p != TCB_NULL) {
       p->next = highest_tcb->next;
     }
    
    if (n != TCB_NULL) {
      n->prev = highest_tcb->prev;
     }
    
    highest_tcb->prev = TCB_NULL;
    highest_tcb->next = TCB_NULL;

    return highest_tcb;
   }

  return TCB_NULL;
} /* Pull_with_priority */


TCB *tcblink_pull_with_tcb(TCBLINK *tcblink, TCB *tcb) {

  TCB *w = tcblink->root.next;
  
  while (w != TCB_NULL) {

    if (w == tcb) {
      TCB *p = w->prev;
      TCB *n = w->next;

      if (p != TCB_NULL) {
        p->next = w->next;
       }

      if (n != TCB_NULL) {
        n->prev = w->prev;
       }
            
      w->prev = TCB_NULL;
      w->next = TCB_NULL;
            
      return w;
     }
    
   w = w->next;
  }
  
  return TCB_NULL;
} /* tcblink_pull_with_tcb */


TCB *tcblink_pull_state(TCBLINK *tcblink, STATE_T state) {

  TCB *w = tcblink->root.next;
  
  while (w != TCB_NULL) {
    
    if (w->thread.info.state == state) {
      TCB *p = w->prev;
      TCB *n = w->next;

      if (p != TCB_NULL) {
        p->next = w->next;
       }

      if (n != TCB_NULL) {
        n->prev = w->prev;
       }

      w->prev = TCB_NULL;
      w->next = TCB_NULL;

      return w;
     }

    w = w->next;
   }
  
  return TCB_NULL;
} /* tcblink_pull_state */


void tcblink_list(TCBLINK *tcblink, TCBLINK_LIST_CALLBACK callback) {

  TCB *w = &(tcblink->root);
  
  do {
    TCB *next = w->next;
    callback(w);
    w = next;
   } while (w != TCB_NULL);
} /* tcblink_list */


/* NOT used, is for Round-Robin mode */
/*
TCB *tcblink_pull(TCBLINK *tcblink) {
    TCB *root = &(tcblink->root);
    TCB *q1st = root->next;
    TCB *q2nd = (q1st == TCB_NULL) ? TCB_NULL : q1st->next;
    if (q1st == TCB_NULL) {
        return TCB_NULL;
    }
    root->next = q2nd;
    if (q2nd != TCB_NULL) {
        q2nd->prev = root;
    }
    q1st->prev = TCB_NULL;
    q1st->next = TCB_NULL;
    return q1st;
}
*/