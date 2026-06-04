px4_add_module(
	MODULE modules__spoofing_detector
	MAIN spoofing_detector
	SRCS
		SpoofingDetector.cpp
		SpoofingDetector.hpp
	DEPENDS
		px4_work_queue
)
