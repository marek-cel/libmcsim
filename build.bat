call clean.bat

mkdir build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release -DMCUTILS_INCLUDE_DIR="%LIBMCUTILS_DIR%\include" -DMCUTILS_LIBRARY="%LIBMCUTILS_DIR%\lib\mcutils.lib" -DCMAKE_INSTALL_PREFIX=%LIBMCSIM_DIR%
cmake --build . --config Release