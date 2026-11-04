# IDPS modules built into PX4 as external modules:
#   make px4_sitl EXTERNAL_MODULES_LOCATION=<IPS repo>/px4
#   make px4_fmu-v6c_default EXTERNAL_MODULES_LOCATION=<IPS repo>/px4
set(config_module_list_external
	modules/idps_show
	PARENT_SCOPE
	)
