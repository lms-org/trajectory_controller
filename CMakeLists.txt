set ( SOURCES
	"src/trajectory_point_follower.cpp"
	"src/interface.cpp"
)

set (HEADERS
	"include/trajectory_point_follower.h"
)
include_directories("include")
include_directories("dlib_files")
#add_definitions(-DENABLE_ASSERTS)

add_library (trajectory_point_controller MODULE ${SOURCES} ${HEADERS} "dlib_files")
target_link_libraries(trajectory_point_controller PRIVATE lmscore lms_sensor_utils street_environment lms_math phoenix_CC2016_service)
