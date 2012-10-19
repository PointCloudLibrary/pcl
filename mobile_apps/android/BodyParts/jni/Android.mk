include $(call all-subdir-makefiles)

$(call import-module,OpenNI)
$(call import-module,XnCore)
$(call import-module,XnDDK)
$(call import-module,XnFormats)
$(call import-module,XnDeviceSensorV2)

$(call import-module,boost)
$(call import-module,boost_filesystem)

$(call import-module,tbb)
