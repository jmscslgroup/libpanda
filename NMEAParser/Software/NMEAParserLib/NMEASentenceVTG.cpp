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
#include "NMEASentenceVTG.h"
#include <stdlib.h>
#include <string.h>


CNMEASentenceVTG::CNMEASentenceVTG()
{
}


CNMEASentenceVTG::~CNMEASentenceVTG()
{
}

CNMEAParserData::ERROR_E CNMEASentenceVTG::ProcessSentence(char * pCmd, char * pData)
{
    UNUSED_PARAM(pCmd);
    char szField[c_nMaxField];

//	// Number of sentences
//	if (GetField(pData, szField, 0, c_nMaxField) == CNMEAParserData::ERROR_OK) {
//		m_SentenceData.nTotalNumberOfSentences = atoi(szField);
//	}

	// Tracking direction, True
	if (GetField(pData, szField, 0, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_trackTrue = atof(szField);
	}
	
	// Field 1 should alwasy be 'T'
//	if (GetField(pData, szField, 1, c_nMaxField) == CNMEAParserData::ERROR_OK) {
//	}

	// Tracking direction, Magnetic
	if (GetField(pData, szField, 2, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_trackMag = atof(szField);
	}
	
	// Field 3 should alwasy be 'M'
//	if (GetField(pData, szField, 3, c_nMaxField) == CNMEAParserData::ERROR_OK) {
//	}

	// Speed in Knots
	if (GetField(pData, szField, 4, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_speedKn = atof(szField);
	}
	
	// Field 5 should alwasy be 'N'
//	if (GetField(pData, szField, 5, c_nMaxField) == CNMEAParserData::ERROR_OK) {
//	}
	
	// Speed in km/h
	if (GetField(pData, szField, 6, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_speedKm = atof(szField);
	}
	
	// Field 7 should alwasy be 'K'
//	if (GetField(pData, szField, 7, c_nMaxField) == CNMEAParserData::ERROR_OK) {
//	}
	
	// Positioning system mode indicator
	if (GetField(pData, szField, 8, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_posModeInd = (CNMEAParserData::POS_MODE_IND_E) szField[0];
	}

	m_uRxCount++;

	return CNMEAParserData::ERROR_OK;
}

void CNMEASentenceVTG::ResetData(void)
{
	m_uRxCount = 0;
	
	m_SentenceData.m_trackTrue = 0;
	m_SentenceData.m_trackMag = 0;
	m_SentenceData.m_speedKm = 0;
	m_SentenceData.m_speedKn = 0;
	m_SentenceData.m_posModeInd = CNMEAParserData::POS_MODE_IND_INVALID;

}
