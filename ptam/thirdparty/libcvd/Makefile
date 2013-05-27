all: libcvd_built

libcvd_built: 
	mkdir -p build
	git clone https://github.com/edrosten/libcvd.git build
	cd build && git checkout 26bb46b9845383a4724a5e24db42dc27d41477d4
	cd build && export CPATH=../../TooN/include && ./configure --without-ffmpeg --without-dc1394v1 --without-dc1394v2 --with-toon --with-lapack
	cd build && export CPATH=../../TooN/include && make -j4 -l4
	mkdir -p include
	mkdir -p lib
	cp build/*.so* lib
	cp -r build/cvd include
	touch libcvd_built

clean:
	rm -rf include lib
	rm -rf build
	rm -f libcvd_built

