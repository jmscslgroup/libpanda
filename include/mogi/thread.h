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

#ifndef MOGI_THREAD_H
#define MOGI_THREAD_H

#include <pthread.h>

namespace Mogi {

/*! \class Thread
 \brief Abstract class, handles a single thread.  Features mutual exclusion and pause/resume.
 */

class Thread {
public:
	Thread();
	virtual ~Thread() = 0;

	/*! \brief Starts the internal thread method.

	 @return true if the thread was successfully started, false if there was an
	 error starting the thread.
	 */
	bool start();

	/**! \brief Stops the internal thread method.
	 */
	void stop();

	/*! \brief Will wait until the thread has finished.
	 */
	void WaitForInternalThreadToExit();

	/*! \brief Performs a mutual exclusion lock.  
	 Will halt if locked by another thread, and
	 will resume when the other thread performs an unlock.
	 @see Thread#unlock
	 @return 0 on success, otherwise an error corresponding to pthreads.
	 */
	int lock();

	/*! \brief Performs a mutual exclusion unlock.  Will allow other threads that have been
	 locked to resume.
	 @see Thread#lock
	 @return 0 on success, otherwise an error corresponding to pthreads.
	 */
	int unlock();

	/*! \brief Performs a pause.
	 @see Thread#resume.
	 */
	void pause();

	/*! \brief Resumes the thread from a pause.
	 @see Thread#pause
	 */
	void resume();

	/*! \brief Returns the state of the thread.
	 @return true if running
	 */
	bool running() {
		return isRunning;
	}

protected:
	/*! \brief Called once at the beginning of the thread running.

	 Use this to set up data for the thread.
	 */
	virtual void entryAction();

	/*! \brief Continuously called until the thread is commanded to stop.

	 If not defined in the base class, the loop will simply exit. Good practice
	 would be to make this non-blocking.
	 */
	virtual void doAction();

	/*! \brief Called once when thread is commanded to stop.

	 Use this to clean up data in the thread, or place into a stopped state.
	 */
	virtual void exitAction();

	/*! \brief Blocks the thread from a pause() call until resume() is called
	 */
	void checkSuspend();

private:
	static void* InternalThreadEntryFunc(void* This);

	pthread_t _thread;
	pthread_mutex_t _mutex;
	pthread_mutex_t _condMutex;
	pthread_cond_t _cond;

	bool pauseFlag;
	//	bool killable;
	bool isRunning;
	bool shouldTerminate;
};
}

#endif
