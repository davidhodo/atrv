find_path(atrv_INCLUDE_DIRS atrv.h /usr/include "$ENV{NAMER_ROOT}")

find_library(atrv_LIBRARIES atrv /usr/lib "$ENV{NAMER_ROOT}")

set(atrv_FOUND TRUE)

if (NOT atrv_INCLUDE_DIRS)
    set(atrv_FOUND FALSE)
endif (NOT atrv_INCLUDE_DIRS)

if (NOT atrv_LIBRARIES)
    set(atrv_FOUND FALSE)
endif (NOT atrv_LIBRARIES)