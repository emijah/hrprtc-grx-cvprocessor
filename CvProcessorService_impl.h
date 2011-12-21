// -*-C++-*-
/*!
 * @file  CvProcessorService_impl.h
 * @brief Service implementation header of CvProcessorService.idl
 *
 */

#include "CvProcessorService.hh"


#ifndef CVPROCESSORSERVICE_IMPL_H
#define CVPROCESSORSERVICE_IMPL_H

/*
 * Example class implementing IDL interface OpenHRP::CvProcessorService
 */
class CvProcessor;

class CvProcessorService_impl
    : public virtual POA_OpenHRP::CvProcessorService,
    public virtual PortableServer::RefCountServantBase
{
private:
    // Make sure all instances are built on the heap by making the
    // destructor non-public
    //virtual ~CvProcessorService_impl();

public:
    // standard constructor
    CvProcessorService_impl();
    virtual ~CvProcessorService_impl();

    void
    setComponent (CvProcessor * i_comp)
    {
        m_comp = i_comp;
    }

    // attributes and operations
    ::CORBA::ULong HoughCircles(::CORBA::Long id, OpenHRP::darray3Seq_out circles);
    ::CORBA::ULong HoughLinesP(::CORBA::Long id, OpenHRP::iarray4Seq_out lines);
    ::CORBA::ULong detectFaces(::CORBA::Long id, OpenHRP::darray3Seq_out faces, ::CORBA::Boolean doSaveImage);

private:
    CvProcessor* m_comp;
};



#endif // CVPROCESSORSERVICE_IMPL_H


