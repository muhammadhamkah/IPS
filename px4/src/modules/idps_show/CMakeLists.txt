px4_add_module(
	MODULE modules__idps_show
	MAIN idps_show
	MODULE_CONFIG
		module.yaml
	SRCS
		IdpsShow.cpp
		IdpsShow.hpp
		TrjFile.cpp
		TrjFile.hpp
	)
