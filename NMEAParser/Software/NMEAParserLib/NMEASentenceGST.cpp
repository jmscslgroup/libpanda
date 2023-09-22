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
#include <stdlib.h>
#include <math.h> // For NAN
#include "NMEASentenceGST.h"

CNMEASentenceGST::CNMEASentenceGST() :
	m_nOldVSpeedSeconds(0),
	m_dOldVSpeedAlt(0.0)
{
	ResetData();
}

CNMEASentenceGST::~CNMEASentenceGST() {
}

CNMEAParserData::ERROR_E CNMEASentenceGST::ProcessSentence(char * pCmd, char * pData)
{
    UNUSED_PARAM(pCmd);
    char szField[c_nMaxField];
	//char szTemp[c_nMaxField];

//	if (GetField(pData, szField, 0, c_nMaxField) == CNMEAParserData::ERROR_OK) {    // TODO get UTC time and do somethign with it
//		m_SentenceData.m_nHour = (szField[0] - '0') * 10 + (szField[1] - '0');
//		m_SentenceData.m_nMinute = (szField[2] - '0') * 10 + (szField[3] - '0');
//		m_SentenceData.m_nSecond = (szField[4] - '0') * 10 + (szField[5] - '0');
//	}

	if (GetField(pData, szField, 1, c_nMaxField) == CNMEAParserData::ERROR_OK)
	{
        m_SentenceData.m_nRMS = atof((char *)szField);
	}
	
    if (GetField(pData, szField, 2, c_nMaxField) == CNMEAParserData::ERROR_OK)
    {
        m_SentenceData.m_nErrorEllipseMajor = atof((char *)szField);
    }
    if (GetField(pData, szField, 3, c_nMaxField) == CNMEAParserData::ERROR_OK)
    {
        m_SentenceData.m_nErrorEllipseMinor = atof((char *)szField);
    }
    if (GetField(pData, szField, 4, c_nMaxField) == CNMEAParserData::ERROR_OK)
    {
        m_SentenceData.m_nErrorEllipseOrientation = atof((char *)szField);
    }
    if (GetField(pData, szField, 5, c_nMaxField) == CNMEAParserData::ERROR_OK)
    {
        m_SentenceData.m_nLatitudeSigmaError = atof((char *)szField);
    }
    if (GetField(pData, szField, 6, c_nMaxField) == CNMEAParserData::ERROR_OK)
    {
        m_SentenceData.m_nLongitudeSigmaError = atof((char *)szField);
    }
    if (GetField(pData, szField, 7, c_nMaxField) == CNMEAParserData::ERROR_OK)
    {
        m_SentenceData.m_nAltitudeSigmaError = atof((char *)szField);
    }

	return CNMEAParserData::ERROR_OK;
}

void CNMEASentenceGST::ResetData(void)
{
    m_SentenceData.m_nSecond = 0;
    m_SentenceData.m_nMilliSecond = 0;
    m_SentenceData.m_nRMS = NAN;
    m_SentenceData.m_nErrorEllipseMajor = NAN;
    m_SentenceData.m_nErrorEllipseMinor = NAN;
    m_SentenceData.m_nErrorEllipseOrientation = NAN;
    m_SentenceData.m_nLatitudeSigmaError = NAN;
    m_SentenceData.m_nLongitudeSigmaError = NAN;
    m_SentenceData.m_nAltitudeSigmaError = NAN;
    
}
