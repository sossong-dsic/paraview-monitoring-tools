/*
 *==========================================================================
 *
 *   Program: MTView
 *
 *   Copyright (c) 2015 Decision Sciences International Corporation
 *   All rights reserved.
 *
 *   Author: Kevin Dean
 *   Author: Dustin Dorroh
 *
 *==========================================================================
*/

/* PROTOBUF IMAGE READER INCLUDE FILE */
/*----------------------------------------------------------------------------*/
#include "vtkPolyData.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkDataObject.h"
#include "vtkSmartPointer.h"
#include "vtkFloatArray.h"
#include "vtkPointData.h"
#include "vtkXMLFileReadTester.h"
#include "vtkLineSource.h"
#include "vtkAppendPolyData.h"
#include "vtkTubeFilter.h"

/* DSC INCLUDE FILES */
/*----------------------------------------------------------------------------*/
#include "dsclib/system/io/file_utilities.h"
#include "dsclib/system/io/Logger.h"
//#include "mtlib-protobuf/persist/Reader.h"

#include "mtlib-types/scanner/GeometryInfoIOBin.h"
//#include "mtlib-types/scanner/GeometryInfo.h"
//#include "mtlib-types/scanner/GeometryNode.h"

/* SYSTEM INCLUDE FILES */
/*----------------------------------------------------------------------------*/
#include <sys/stat.h>
#include <assert.h>
#include <strstream>
#include <sys/types.h>
#include <errno.h>

/* BOOST INCLUDE FILES */
/*----------------------------------------------------------------------------*/
#include <boost/filesystem/path.hpp>
#include <GeometryDriftTubeSection.h>


/* Use vtkStandardNewMacro for the object class's only allowed constructor */
vtkStandardNewMacro(GeometryDriftTubeSection);

/*----------------------------------------------------------------------------*/
GeometryDriftTubeSection::GeometryDriftTubeSection()
{
  this->FileName = NULL;
  this->SetNumberOfInputPorts(0);

//  DifferentiateProtobufEpoch::New();
}

/*----------------------------------------------------------------------------*/
GeometryDriftTubeSection::~GeometryDriftTubeSection()
{
  if (this->FileName)
  {
	this->SetFileName(0);
  }
}

/*----------------------------------------------------------------------------*/
void GeometryDriftTubeSection::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

/*----------------------------------------------------------------------------*/
void GeometryDriftTubeSection::LoadBinaryGeometryInfo(const char* fname)
{
  vtkSmartPointer<vtkAppendPolyData> DriftTubeSection = vtkSmartPointer<vtkAppendPolyData>::New();

  static mtlib::GeometryInfoSharedPtr GeometryInfo = mtlib::GeometryInfoIOBin::LoadBinaryGeometryInfo(fname);

  for (int i = 0; i < GeometryInfo->NumDriftTubeSections(); i++)
  {
	vtkSmartPointer<vtkLineSource> TubeSection = vtkSmartPointer<vtkLineSource>::New();
	TubeSection->SetPoint1(GeometryInfo->GetDriftTubeSection(i)->GetExtents().GetPointX(),
						   GeometryInfo->GetDriftTubeSection(i)->GetExtents().GetPointY(),
						   GeometryInfo->GetDriftTubeSection(i)->GetExtents().GetPointZ());
	TubeSection->SetPoint2(GeometryInfo->GetDriftTubeSection(i)->GetExtents().GetTip().GetX(),
						   GeometryInfo->GetDriftTubeSection(i)->GetExtents().GetTip().GetY(),
						   GeometryInfo->GetDriftTubeSection(i)->GetExtents().GetTip().GetZ());
	TubeSection->Update();

	vtkSmartPointer<vtkTubeFilter> tube_filter = vtkSmartPointer<vtkTubeFilter>::New();
	tube_filter->SetInputConnection(TubeSection->GetOutputPort());
	tube_filter->SetRadius(2.54);
	tube_filter->SetNumberOfSides(50);
	tube_filter->Update();

	DriftTubeSection->AddInputData(TubeSection->GetOutput());
	DriftTubeSection->AddInputData(tube_filter->GetOutput());
  }

  DriftTubeSection->Update();
  vtk_append_poly_data = DriftTubeSection;
}

/*----------------------------------------------------------------------------*/
int GeometryDriftTubeSection::FillOutputPortInformation(int port, vtkInformation* outInfo)
{
  if (port == 0)
  {
	outInfo->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData");

	return 1;
  }

  return 0;
}

/*----------------------------------------------------------------------------*/
int GeometryDriftTubeSection::RequestInformation(vtkInformation*,
                                             vtkInformationVector**,
                                             vtkInformationVector* outputVector)
{
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  vtkSmartPointer<vtkAppendPolyData> append_poly_data = vtkSmartPointer<vtkAppendPolyData>::New();
  vtk_append_poly_data = append_poly_data;

  this->LoadBinaryGeometryInfo(FileName);

  int extents[6] = {0, 0, 0, 0, 0, 0};

  outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), extents, 6);
  outInfo->Set(vtkDataObject::ORIGIN(), 0.0, 0.0, 0.0);
  outInfo->Set(vtkDataObject::SPACING(), 1, 1, 1);
  vtkDataObject::SetPointDataActiveScalarInfo(outInfo, VTK_UNSIGNED_CHAR, 1);
  outInfo->Set(CAN_PRODUCE_SUB_EXTENT(), 1);

  if (FileName == NULL)
  {
    //~ char *error_message = std::strerror(errno);
    //~ dsclib::Log(LOG_CRIT, "\n%s: Unable to read FileName '%s': %s\n", __PRETTY_FUNCTION__, FileName, error_message);
  }

  else
  {
	printf("FileName: %s\n", FileName);
  }

  return 1;
}

int GeometryDriftTubeSection::RequestData(vtkInformation *vtkNotUsed(request),
                                        vtkInformationVector **vtkNotUsed(inputVector),
                                        vtkInformationVector *outputVector)
{
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPolyData* output = vtkPolyData::GetData(outputVector,0);

  // Acquire PolyData output
  vtkSmartPointer<vtkPolyData> polyData = vtk_append_poly_data->GetOutput();

  output->ShallowCopy(polyData);

  return 1;
}

int GeometryDriftTubeSection::CanReadFile(const char *fname)
{
  return 1;
}
