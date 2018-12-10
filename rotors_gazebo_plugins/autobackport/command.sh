#!/usr/bin/env bash
find . -type f -name "*.cpp" | xargs sed -i.bak -r -f autobackport/seds.txt
find . -type f -name "*.h" | xargs sed -i.bak -r -f autobackport/seds.txt
find . -type f -name "*.hpp" | xargs sed -i.bak -r -f autobackport/seds.txt

#sed -i.bak -r -f seds_cmake.txt ../CMakeLists.txt


