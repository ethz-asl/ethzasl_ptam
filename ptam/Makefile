all:
	cd thirdparty && make
	make -j8 -f Makefile_ptam

.DEFAULT:
	cd thirdparty && make $@
	make -j8 -f Makefile_ptam $@


