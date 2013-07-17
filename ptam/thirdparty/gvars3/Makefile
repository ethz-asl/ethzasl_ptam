all: git_checkedout gvars_built

git_checkedout:
	mkdir -p sources
	git clone https://github.com/edrosten/gvars.git sources
	cd sources && git checkout 1f325cead6117b4fbd8016e46dc2f654f0ab7f25
	touch git_checkedout

gvars_built: 
	cp -r sources build
	cd build && if [ "`arch`" = "armv7l" ]; then sed -i 's/CXX=@CXX@/CXX=@CXX@ -fPIC/g' Makefile.in; fi
	cd build && export CPATH=../../TooN/include && ./configure --without-ffmpeg --without-dc1394v1 --without-dc1394v2 --with-lapack --with-pic=no && make -j4 -l4
	mkdir -p include
	mkdir -p lib
	cp build/*.so* lib
	cp -r build/gvars3 include
	touch gvars_built

distclean:
	rm -rf sources
	rm git_checkedout
	rm -rf include lib
	rm -rf build
	rm -f gvars_built
	
clean:
	rm -rf include lib
	rm -rf build
	rm -f gvars_built
