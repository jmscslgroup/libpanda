/*
 * NMEASentenceZDA.cpp
 *
 *  Created on: Sep 21, 2017
 *      Author: monte
 *  Modifed on: Feb 22, 2020
 *       Author: Matt Bunting
 *         Note: This is a modified version of monte's original NMEASentenceRMC.cpp
 */
 /*
 * MIT License
 *
 *  Copyright (c) 2018 VisualGPS, LLC
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 */


#include "NMEASentenceZDA.h"
#include <stdlib.h>

CNMEASentenceZDA::CNMEASentenceZDA() {
	// TODO Auto-generated constructor stub

}

CNMEASentenceZDA::~CNMEASentenceZDA() {
	// TODO Auto-generated destructor stub
}

CNMEAParserData::ERROR_E CNMEASentenceZDA::ProcessSentence(char *pCmd, char *pData) {

    UNUSED_PARAM(pCmd);
    char szField[c_nMaxField];

	// Example: $GNZDA,043428.20,23,02,2020,00,00*70

	// Time
	if (GetField(pData, szField, 0, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_nHour = (szField[0] - '0') * 10 + (szField[1] - '0');
		m_SentenceData.m_nMinute = (szField[2] - '0') * 10 + (szField[3] - '0');
		//m_SentenceData.m_nSecond = (szField[4] - '0') * 10 + (szField[5] - '0');

		// Bunting: added the following:
		m_SentenceData.m_dSecond = (double)((szField[4] - '0') * 10 + (szField[5] - '0')) + 0.1*(double)(szField[7] - '0') + 0.01*(double)(szField[8] - '0');
		m_SentenceData.m_nMilliSecond = 100*(double)(szField[7] - '0') + 10*(double)(szField[8] - '0');
	}

	

	// Day
	if (GetField(pData, szField, 1, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		// 23
		m_SentenceData.m_nDay = (szField[0] - '0') * 10 + (szField[1] - '0');
	}
	else {
		m_SentenceData.m_nDay = 0;
	}

	// Month
	if (GetField(pData, szField, 2, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_nMonth = (szField[0] - '0') * 10 + (szField[1] - '0');
	}
	else {
		m_SentenceData.m_nMonth = 0;
	}

	// Year
	if (GetField(pData, szField, 3, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_nYear = (szField[0] - '0') * 1000 + (szField[1] - '0') * 100 + (szField[2] - '0') * 10 + (szField[3] - '0');
	}
	else {
		m_SentenceData.m_nYear = 0;
	}

	// Local time zone offset from GMT in hours
	if (GetField(pData, szField, 4, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_nLocalHour = (szField[0] - '0') * 10 + (szField[1] - '0');
	}
	else {
		m_SentenceData.m_nLocalHour = 0;
	}

	// Local time zone offset from GMT in Minutes
	if (GetField(pData, szField, 5, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_nLocalMinute = (szField[0] - '0') * 10 + (szField[1] - '0');
	}
	else {
		m_SentenceData.m_nLocalMinute = 0;
	}



	m_uRxCount++;

	return CNMEAParserData::ERROR_OK;
}

void CNMEASentenceZDA::ResetData(void) {
	m_uRxCount = 0;
	m_SentenceData.m_dSecond = 0;
	m_SentenceData.m_nDay = 0;
	m_SentenceData.m_nHour = 0;
	m_SentenceData.m_nMinute = 0;
	m_SentenceData.m_nLocalHour = 0;
	m_SentenceData.m_nLocalMinute = 0;
	m_SentenceData.m_nMonth = 0;
	//m_SentenceData.m_nSecond = 0;
	m_SentenceData.m_nYear = 0;
}
