/******************************************************************************
 *                                                                            *
 *             Copyright (C) 2016 Mogi, LLC - All Rights Reserved             *
 *                            Author: Matt Bunting                            *
 *                                                                            *
 *   Proprietary and confidential.                                            *
 *                                                                            *
 *   Unauthorized copying of this file via any medium is strictly prohibited  *
 *   without the explicit permission of Mogi, LLC.                            *
 *                                                                            *
 *   See license in root directory for terms.                                 *
 *   http://www.binpress.com/license/view/l/0088eb4b29b2fcff36e42134b0949f93  *
 *                                                                            *
 *****************************************************************************/

#include "mogi/thread.h"
#include <iostream>

#ifdef IDENT_C
static const char* const Thread_C_Id = "$Id$";
#endif

#ifdef _cplusplus
extern "C" {
#endif

using namespace Mogi;

void Thread::entryAction() {
}

void Thread::doAction() {
	shouldTerminate = true;
}

void Thread::exitAction() {
}

void* Thread::InternalThreadEntryFunc(void* This) {
	Thread* thread = (Thread*) This;
	thread->entryAction();
	while (thread->shouldTerminate == false) {
		thread->doAction();
		thread->checkSuspend();
	}
	thread->exitAction();
	thread->isRunning = false;
	return NULL;
}

Thread::Thread() :
		pauseFlag(false), isRunning(false), shouldTerminate(false) {
	pthread_mutex_init(&_mutex, NULL);
	pthread_mutex_init(&_condMutex, NULL);
	pthread_cond_init(&_cond, NULL);
}

Thread::~Thread() {
	stop();
	while (isRunning)
		;
	pthread_mutex_destroy(&_mutex);
	pthread_mutex_destroy(&_condMutex);
	pthread_cond_destroy(&_cond);
}

/** Returns true if the thread was successfully started, false if there was an
 * error starting the thread */
bool Thread::start() {
	shouldTerminate = false;
	isRunning = true;
	isRunning = pthread_create(&_thread, NULL, InternalThreadEntryFunc, this)
			== 0;
	return isRunning;
}

void Thread::stop() {
	shouldTerminate = true;
	resume();
}

/** Will not return until the internal thread has exited. */
void Thread::WaitForInternalThreadToExit() {
	if (isRunning) {
		stop();
		pthread_join(_thread, NULL);
	}
	// isRunning = false;
}

int Thread::lock() {
	return pthread_mutex_lock(&_mutex);
}

int Thread::unlock() {
	return pthread_mutex_unlock(&_mutex);
}

void Thread::pause() {
	pthread_mutex_lock(&_condMutex);
	pauseFlag = true;
	pthread_mutex_unlock(&_condMutex);
}

void Thread::resume() {
	pthread_mutex_lock(&_condMutex);
	pauseFlag = false;
	pthread_cond_broadcast(&_cond);
	pthread_mutex_unlock(&_condMutex);
}

void Thread::checkSuspend() {
	pthread_mutex_lock(&_condMutex);
	while (pauseFlag) {
		pthread_cond_wait(&_cond, &_condMutex);
	}
	pthread_mutex_unlock(&_condMutex);
}

#ifdef _cplusplus
}
#endif
