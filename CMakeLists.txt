set ( SOURCES
	"src/trajectory_point_follower.cpp"
	"src/interface.cpp"
)

set (HEADERS
	"include/trajectory_point_follower.h"
)
set(LENKWINKEL "lenkwinkel")
include_directories("include")
include_directories("${LENKWINKEL}")
include_directories("dlib_files")
file(GLOB lenkwinkel "${LENKWINKEL}/*.c" "${LENKWINKEL}/*.h")


add_library (trajectory_point_controller MODULE ${SOURCES} ${HEADERS} ${lenkwinkel} "dlib_files")
target_link_libraries(trajectory_point_controller PRIVATE lmscore sensor_utils)
