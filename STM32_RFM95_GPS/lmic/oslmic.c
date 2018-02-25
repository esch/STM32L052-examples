/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

#include "lmic.h"
// #include "stm32_rfm95_gps.h"

extern unsigned int dbg_runloop;


// RUNTIME STATE
static struct {
    osjob_t* scheduledjobs;
    osjob_t* runnablejobs;
} OS;

void os_init () {
    memset(&OS, 0x00, sizeof(OS));
    print("os_init 1 \r\n");
    hal_init();
    print("os_init 2 \r\n");
    radio_init();
    print("os_init 3 \r\n");
    LMIC_init();
}

ostime_t os_getTime () {
    return hal_ticks();
}

static u1_t unlinkjob (osjob_t** pnext, osjob_t* job) {
    for( ; *pnext; pnext = &((*pnext)->next)) {
        if(*pnext == job) { // unlink
            *pnext = job->next;
            return 1;
        }
    }
    return 0;
}

// clear scheduled job
void os_clearCallback (osjob_t* job) {
    hal_disableIRQs();
    unlinkjob(&OS.scheduledjobs, job) || unlinkjob(&OS.runnablejobs, job);
    hal_enableIRQs();
}

// schedule immediately runnable job
void os_setCallback (osjob_t* job, osjobcb_t cb,char *fname) {
    osjob_t** pnext;
    hal_disableIRQs();
    // remove if job was already queued
    os_clearCallback(job);
    // fill-in job
    memset(job->name,0,16);
    memcpy(job->name,fname,12);
    job->func = cb;
    job->next = NULL;
    // add to end of run queue
    for(pnext=&OS.runnablejobs; *pnext; pnext=&((*pnext)->next));
    *pnext = job;
    hal_enableIRQs();
}

// schedule timed job
void os_setTimedCallback (osjob_t* job, ostime_t time, osjobcb_t cb,char *fname) {
    osjob_t** pnext;
    hal_disableIRQs();
    // remove if job was already queued
    os_clearCallback(job);
    // fill-in job
    print("os  ");
    print_hex32(cb);
    memset(job->name,0,16);
    memcpy(job->name,fname,12);
    job->deadline = time;
    job->func = cb;
    job->next = NULL;
    // insert into schedule
    for(pnext=&OS.scheduledjobs; *pnext; pnext=&((*pnext)->next)) {
        if((*pnext)->deadline - time > 0) { // (cmp diff, not abs!)
            // enqueue before next element and stop
            job->next = *pnext;
            break;
        }
    }
    *pnext = job;
    hal_enableIRQs();
}

int yy = 0;
// execute jobs from timer and from run queue
void os_runloop () {
    while(1) {
        osjob_t* j = NULL;
        // check for runnable jobs

        if( !(yy++%5000) ) {
            println(" -+- osrl -+- ");
            process_gps();
        }

        hal_disableIRQs();

#ifdef LATER
            do {
                if( ! strncmp( strt_+c,"$GNGGA",6) || ! strncmp( strt_+c,"$GNRMC",6) )

                lastStartP = c;

                if( lastStartP != 2000 && strt_[c] == '*')      
                endP = c;
 
                } while( c++ < 450 && endP == 2000 );

#endif

        // print_decimal(yy++);
        dbg_runloop++;

        if(OS.runnablejobs) {
            j = OS.runnablejobs;
            OS.runnablejobs = j->next;
             // print("osrl 1 \r\n");
        } else if(OS.scheduledjobs && hal_checkTimer(OS.scheduledjobs->deadline)) { // check for expired timed jobs
            j = OS.scheduledjobs;
            OS.scheduledjobs = j->next;
            // print("osrl 2 \r\n");
        } else { // nothing pending
            hal_sleep(); // wake by irq (timer already restarted)
        }
        // print("osrl 3 \r\n");
        hal_enableIRQs();
        if(j) { // run job callback
            print(j->name);
            print("\r\n");

            print("osrl 3a \r\n");
            print_hex32(j->func);
            j->func(j);
            print("osrl 3b \r\n");

        }
    }
}
