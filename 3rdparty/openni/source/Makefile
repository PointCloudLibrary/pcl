# Author: Suat Gedikli
# Date:   4. April 2011

install : ../openni_drivers
	cd ../openni_drivers && make install_openni && make install_engine && cd -

uninstall:
	cd ../openni_drivers && make uninstall && cd -

../openni_drivers:
	hg clone https://kforge.ros.org/openni/drivers ../openni_drivers -r 251
		
clean:
	rm -rf ../openni_drivers
	

