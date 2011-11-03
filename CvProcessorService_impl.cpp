// -*-C++-*-
/*!
 * @file  CvProcessorService_impl.cpp
 * @brief Service implementation code of CvProcessorService.idl
 *
 */

#include "CvProcessorService_impl.h"
#include "CvProcessor.h"

/*
 * Example implementational code for IDL interface OpenHRP::CvProcessorService
 */
CvProcessorService_impl::CvProcessorService_impl()
{
    // Please add extra constructor code here.
}


CvProcessorService_impl::~CvProcessorService_impl()
{
    // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
::CORBA::ULong CvProcessorService_impl::HoughCircles(OpenHRP::darray3Seq_out circles)
{
    m_comp->HoughCircles();
    circles = new OpenHRP::darray3Seq();
    circles->length(m_comp->m_circles->total);
    for (int i=0; i<m_comp->m_circles->total; i++) {
        float *p = (float*)cvGetSeqElem( m_comp->m_circles, i);
        for (int j=0; j<3; j++) {
            circles[i][j] = p[j];
        }
    }
    return 0;
}

::CORBA::ULong CvProcessorService_impl::HoughLinesP(OpenHRP::iarray4Seq_out lines)
{
    m_comp->HoughLinesP();
    lines = new OpenHRP::iarray4Seq();
    lines->length(m_comp->m_lines.size());
    for (int i=0; i<m_comp->m_lines.size(); i++) {
        for (int j=0; j<4; j++) {
            lines[i][j] = m_comp->m_lines[i][j];
        }
    }
    return 0;
}
