/*
==========================================================================

     Program: MTView

     Copyright (c) 2015 Decision Sciences International Corporation
     All rights reserved.

	 Author: Dustin Dorroh
     Author: Kevin Dean

==========================================================================
*/


#ifndef __GeometryDriftTubeSection_h
#define __GeometryDriftTubeSection_h

// VTK INCLUDE FILES
#include "vtkPVVTKExtensionsDefaultModule.h" //needed for exports
#include "vtkPolyDataAlgorithm.h"
#include "vtkAppendPolyData.h"
#include "vtkSmartPointer.h"

//#include "mtlib-types/scanner/GeometryInfo.h"

// BOOST INCLUDE FILES
#include <boost/shared_ptr.hpp>

// DSC INCLUDE FILES
//#include "mtlib-protobuf/persist/Reader.h"
//#include "mtlib-protobuf/protobuf/VTKPolyDataObjectsEpoch.pb.h"

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT GeometryDriftTubeSection : public vtkPolyDataAlgorithm
{
public:
  static GeometryDriftTubeSection *New();
  vtkTypeMacro(GeometryDriftTubeSection,vtkPolyDataAlgorithm);

  vtkGetStringMacro(FileName);
  vtkSetStringMacro(FileName);

  virtual void PrintSelf(ostream& os, vtkIndent indent);

  virtual int CanReadFile(const char* fname);

  void LoadBinaryGeometryInfo(const char* fname);

  vtkSmartPointer<vtkAppendPolyData> vtk_append_poly_data;

protected:
  GeometryDriftTubeSection();
  ~GeometryDriftTubeSection();

  char *FileName;
  int FillOutputPortInformation(int port, vtkInformation* outInfo);
  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector* outputVector);
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  GeometryDriftTubeSection(const GeometryDriftTubeSection&);  // Not implemented.
  void operator=(const GeometryDriftTubeSection&);  // Not implemented.

};

#endif	/* __GeometryDriftTubeSection_h */
