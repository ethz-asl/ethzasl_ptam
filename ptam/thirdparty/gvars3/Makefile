all: gvars_built

gvars_built: 
	mkdir -p build
	git clone https://github.com/edrosten/gvars.git build
	cd build && git checkout 1f325cead6117b4fbd8016e46dc2f654f0ab7f25
	cd build && export CPATH=../../TooN/include && ./configure --without-ffmpeg --without-dc1394v1 --without-dc1394v2 --with-lapack
	cd build && export CPATH=../../TooN/include && make -j4 -l4
	mkdir -p include
	mkdir -p lib
	cp build/*.so* lib
	cp -r build/gvars3 include
	touch gvars_built

clean:
	rm -rf include lib
	rm -rf build
	rm -f gvars_built
