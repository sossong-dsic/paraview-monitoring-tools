####################################################################################
cmake_minimum_required (VERSION 2.8)
project (paraview_vtk_image_filters_plugin)
FILE(GLOB XML_FILTER_PROPERTIES_FILES
  "*.xml"
)
INSTALL(FILES ${XML_FILTER_PROPERTIES_FILES} DESTINATION "${CMAKE_INSTALL_PREFIX}/share/paraview-xml/")
