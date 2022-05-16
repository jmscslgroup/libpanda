/*
 Author: Matt Bunting
 Copyright (c) 2020 Arizona Board of Regents
 All rights reserved.

 Permission is hereby granted, without written agreement and without
 license or royalty fees, to use, copy, modify, and distribute this
 software and its documentation for any purpose, provided that the
 above copyright notice and the following two paragraphs appear in
 all copies of this software.

 IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
 FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
 ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
 IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
 SUCH DAMAGE.

 THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
 IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
 TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

 */

#ifndef __PANDA_CANFRAMESTATS_H
#define __PANDA_CANFRAMESTATS_H

#include <map>
#include <vector>
#include <array>
#include <list>

#include "panda.h"
#include "mogi/thread.h"

typedef struct _DataInfo {
	int count=0;	// total instances of this exact data sent under the parent message ID (in IdInfo below)
	int bus=0;	// the bus for the last message sent.  perhaps this could change, not sure
	int length=0;	// the data length reported by the sender for this data
} DataInfo;

typedef struct _IdInfo {
	int count = 0;	// total messages sent with this message ID
	int ID = -1;
	double priorTime = 0;
	double currentRate = 1;
	bool highlight = false;
	
	Panda::CanFrame latest;

	// mapping of individual 64-bit data messages to data informaiton (count, bus)
	// The size of this corresponds to the number of unique data sent under the parent message ID (see canStats below)
	std::map<std::array<unsigned char, 64>,DataInfo> data;
} IdInfo;


class CanFrameStats : public Panda::CanListener, public Mogi::Thread {
public:
	CanFrameStats();
	~CanFrameStats();



	std::vector<IdInfo*> canStatsSorted;

	void sortById();
	void sortByIdCount();
	void sortByIdRate();
	void sortByUniqueMessageCount();


	void resetUniqueCount();
	void highlightUniqueCount(int countToHighlight);
	void highlightMessageId(int idToHighlight);
	void highlightCount(int countToHighlight);
	void highlightRate(int rateToHighlight);

	void newDataNotification(Panda::CanFrame* canFrame);

private:

	std::list<Panda::CanFrame*> canFrameFifo;

	// mapping of message IDs to frame stats.
	// The size of this corresponds to number of unique messages IDs recorded
	std::map<unsigned int,IdInfo> canStats;

	int rateRefreshSecondTracker = 0;

	// overload from Mogi::Thread
	void doAction();

};
#endif
