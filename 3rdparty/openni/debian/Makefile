# Author: Suat Gedikli
# Date:   4. April 2011

debian : ../openni_drivers
	cd ../openni_drivers && VERBOSE=1 make debian_openni && VERBOSE=1 make debian_engine && cd -
	cp ../openni_drivers/*openni*.deb .
	cp ../openni_drivers/*ps_engine*.deb .
	rename "s|ros-||" ros-*

../openni_drivers:
	hg clone https://kforge.ros.org/openni/drivers -r 12111edd3b15 ../openni_drivers
	
clean:
	rm -rf *.deb
	rm -rf ../openni_drivers

