#!/bin/bash
RED='\033[0;31m'
NC='\033[0m' # No Color
for project in `find . -maxdepth 1 -mindepth 1 -type d \( ! -iname ".*" \)`
do
  echo -e "${RED}XXXXXXXXXXXXX rebuilding ${project} XXXXXXXXXXXX${NC}" 
  pushd $project/build
  cmake .. -DCMAKE_INSTALL_PREFIX=$INST_DIR -DMMPDS_QT_VERSION=5
  make -j8 install
  popd
done
