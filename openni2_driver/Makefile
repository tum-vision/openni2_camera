all: openni2_driver

#	GIT_DIR: name of directory into which you want to clone to 
GIT_DIR = openni2_src
# GIT_URL: full URL to download
GIT_URL=-b develop git://github.com/OpenNI/OpenNI2.git
#	GIT_PATCH: your (list of) patch file(s) to patch the downloaded software
GIT_REVISION=8795dda219d662f6f1397c66b8486e263df1ce5f
#df26018885ec357f4e418be80fa735a8c011215a
#a5cb66c516361ec5e0c9051fab1fda5562d1e06f

include $(shell rospack find mk)/git_checkout.mk

openni2_driver_build: download
	cd $(GIT_DIR) && \
	make && \
	cd Packaging && ./ReleaseVersion.py x64 && \
	cd ../..

openni2_driver: openni2_driver_build 
	$(eval ONI_DIR=`pwd`/$(GIT_DIR)/Packaging/`ls $(GIT_DIR)/Packaging/ | grep OpenNI-`)
	echo $(ONI_DIR) && \
	mkdir -p include/openni2 && mkdir -p lib && \
	cp -r $(ONI_DIR)/Include/* ./include/openni2/ && \
	cp -r $(ONI_DIR)/Redist/* ./lib/ && \
	cp ./cfg/OpenNI.ini ./lib/ && \
	cp ./cfg/PS1080.ini ./lib/OpenNI2/Drivers

clean: 
	rm -rf $(GIT_DIR) patched build include lib rospack_nosubdirs
