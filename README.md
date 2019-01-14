# paraview-monitoring-tools
Set of tools to read data and display various parameters, data types, etc. from the Discovery System

Building these requires DSIC libraries and headers from various projects including inf/icomm, inf/persist/das-avro-types, and others compiled against a compiled
paraview using it's included VTK.  In the paraview build, INSTALL_DEVELOPMENT_FILES needs to be YES.
