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
#include "NMEASentenceTXT.h"
#include <stdlib.h>
#include <string.h>


CNMEASentenceTXT::CNMEASentenceTXT()
{
}


CNMEASentenceTXT::~CNMEASentenceTXT()
{
}

CNMEAParserData::ERROR_E CNMEASentenceTXT::ProcessSentence(char * pCmd, char * pData)
{
    UNUSED_PARAM(pCmd);
    char szField[c_nMaxField];

	// total number of messages
	if (GetField(pData, szField, 0, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_numMsgs = atoi(szField);
	}
	
	// Number of this message in transission
	if (GetField(pData, szField, 1, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_msgNum = atoi(szField);
	}
	
	// Message type
	if (GetField(pData, szField, 2, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		m_SentenceData.m_txtIdentifier = (CNMEAParserData::TXT_IDENTIFIER_E)atoi(szField);
	}

	// Message
	if (GetField(pData, szField, 3, c_nMaxField) == CNMEAParserData::ERROR_OK) {
		strcpy(m_SentenceData.m_txt, szField);
	}

	m_uRxCount++;

	return CNMEAParserData::ERROR_OK;
}

void CNMEASentenceTXT::ResetData(void)
{
	m_uRxCount = 0;
	
	m_SentenceData.m_numMsgs = 0;
	m_SentenceData.m_msgNum = 0;
	m_SentenceData.m_txtIdentifier = CNMEAParserData::TXT_IDENTIFIER_ERROR;
	memset(m_SentenceData.m_txt, 0, sizeof(m_SentenceData.m_txt));

}
