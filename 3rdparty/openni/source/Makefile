# Author: Suat Gedikli
# Date:   4. April 2011

install : openni_lib
	cd ../openni_drivers && make install && cd -

openni_lib: ../openni_drivers
	cd ../openni_drivers && make && cd -
	
../openni_drivers:
	hg clone https://kforge.ros.org/openni/drivers ../openni_drivers
		
clean:
	rm -rf ../openni_drivers
	

