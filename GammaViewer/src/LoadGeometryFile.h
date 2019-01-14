#ifndef __LoadGeometryFile_h
#define __LoadGeometryFile_h

#include "vtkPVVTKExtensionsDefaultModule.h"
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkSmartPointer.h>

#include "mtlib-types/scanner/GeometryInfoIOBin.h"

class VTKPVVTKEXTENSIONSDEFAULT_EXPORT LoadGeometryFile : public vtkPolyDataAlgorithm
{

public:

  vtkTypeMacro(LoadGeometryFile, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static LoadGeometryFile *New();

  vtkGetMacro(FileName, std::string);
  void SetFileName(std::string filename);

  void SetTubeRange(double first, double last);
  vtkGetVectorMacro(TubeRange, double, 2);

  bool isTubeWithinRange(int tube_index);

  mtlib::GeometryInfoSharedPtr GeometryInfo_;
  vtkSmartPointer<vtkAppendPolyData> vtk_append_poly_data;

protected:

  virtual int RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector);

  LoadGeometryFile();
  virtual ~LoadGeometryFile();

private:

  LoadGeometryFile(const LoadGeometryFile&);
  void operator=(const LoadGeometryFile&);

  std::string FileName;
  double TubeRange[2];

};

#endif /* __LoadGeometryFile_h */
