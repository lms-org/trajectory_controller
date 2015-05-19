set ( SOURCES 
	"src/trajectory_point_follower.cpp"
	"src/interface.cpp"
)

set (HEADERS
	"include/trajectory_point_follower.h"
)
include_directories("include")

add_library (trajectory_point_follower MODULE ${SOURCES} ${HEADERS})
target_link_libraries(trajectory_point_follower PRIVATE lms)
