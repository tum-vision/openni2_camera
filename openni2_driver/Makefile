all: openni2_driver

#	GIT_DIR: name of directory into which you want to clone to 
GIT_DIR = openni2_src
# GIT_URL: full URL to download
GIT_URL=-b develop git://github.com/OpenNI/OpenNI2.git
#	GIT_PATCH: your (list of) patch file(s) to patch the downloaded software
#	GIT_REVISION:  -

include $(shell rospack find mk)/git_checkout.mk

openni2_driver_build: download
	cd $(GIT_DIR) && \
	make && \
	cd Packaging && ./ReleaseVersion.py x64 && \
	cd ../..

openni2_driver: openni2_driver_build 
	$(eval ONI_DIR=`pwd`/$(GIT_DIR)/Packaging/`ls $(GIT_DIR)/Packaging/ | grep OpenNI-`)
	echo $(ONI_DIR) && \
	mkdir -p include/OpenNI2 && mkdir -p lib && \
	cp -r $(ONI_DIR)/Include/* ./include/OpenNI2/ && \
	cp -r $(ONI_DIR)/Redist/* ./lib/ && \
	cp ./cfg/OpenNI.ini ./lib/ && \
	cp ./cfg/PS1080.ini ./lib/OpenNI2/Drivers

clean: 
	rm -rf $(GIT_DIR) patched build include lib rospack_nosubdirs
