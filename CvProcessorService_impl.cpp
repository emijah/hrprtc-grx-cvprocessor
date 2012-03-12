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
::CORBA::ULong CvProcessorService_impl::HoughCircles(::CORBA::Long id, OpenHRP::darray3Seq_out circles)
{
    m_comp->HoughCircles(id);
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

::CORBA::ULong CvProcessorService_impl::HoughLinesP(::CORBA::Long id, OpenHRP::iarray4Seq_out lines)
{
    m_comp->HoughLinesP(id);
    lines = new OpenHRP::iarray4Seq();
    lines->length(m_comp->m_lines.size());
    for (int i=0; i<m_comp->m_lines.size(); i++) {
        for (int j=0; j<4; j++) {
            lines[i][j] = m_comp->m_lines[i][j];
        }
    }
    return 0;
}

::CORBA::ULong CvProcessorService_impl::detectFaces(::CORBA::Long id, OpenHRP::darray3Seq_out faces, ::CORBA::Boolean doSaveImage)
{
    m_comp->detectFaces(id, doSaveImage);
    faces = new OpenHRP::darray3Seq();
    faces->length(m_comp->m_faces->total);
    for (int i=0; i<m_comp->m_faces->total; i++) {
        CvRect *r = (CvRect *) cvGetSeqElem (m_comp->m_faces, i);
        faces[i][0] = cvRound (r->x + r->width * 0.5);
        faces[i][1] = cvRound (r->y + r->height * 0.5);
        faces[i][2] = cvRound ((r->width + r->height) * 0.25);
    }
    return 0;
}
