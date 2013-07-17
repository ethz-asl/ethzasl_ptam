ifeq ("$(shell arch)", "armv7l")
     CXXFLAGS+= -mfpu=neon
endif
CXXFLAGS+= -fsee -funsafe-loop-optimizations -ffinite-math-only -fno-signed-zeros -fno-math-errno -funroll-loops

all: git_checkedout libcvd_built

git_checkedout:
	mkdir -p sources
	git clone https://github.com/edrosten/libcvd.git sources
	cd sources && git checkout 26bb46b9845383a4724a5e24db42dc27d41477d4
	touch git_checkedout
	
libcvd_built: 
	cp -r sources build
	cd build && export CPATH=../../TooN/include && export CXXFLAGS="$(CXXFLAGS)" && ./configure --without-ffmpeg --without-dc1394v1 --without-dc1394v2 --with-toon --with-lapack
	cd build && export CPATH=../../TooN/include && make -j4 -l4
	mkdir -p include
	mkdir -p lib
	cp build/*.so* lib
	cp -r build/cvd include
	touch libcvd_built

distclean:
	rm -rf sources
	rm git_checkedout
	rm -rf include lib
	rm -rf build
	rm -f libcvd_built
	
clean:
	rm -rf include lib
	rm -rf build
	rm -f libcvd_built

