# Author: Suat Gedikli
# Date:   26. October 2011

all : ../openni_drivers
	cd ../openni_drivers && make ps_engine_lib && cd -
	cp ../openni_drivers/openni/build/openni/Platform/Linux/CreateRedist/Final/OpenNI-Bin-*.tar.bz2 .
	cp ../openni_drivers/ps_engine/build/ps_engine/Platform/Linux/CreateRedist/Final/Sensor-Bin-*.tar.bz2 .

install : ../openni_drivers
	cd ../openni_drivers && make install_openni && make install_engine && cd -

uninstall:
	cd ../openni_drivers && make uninstall && cd -

../openni_drivers:
	hg clone https://kforge.ros.org/openni/drivers -r 12111edd3b15 ../openni_drivers
		
clean:
	rm -rf ../openni_drivers
	rm -rf *.bz2
	

