all: toon_built

toon_built: 
	mkdir -p include/
	git clone https://github.com/edrosten/TooN.git include/TooN
	cd include/TooN && git checkout 35bb77741e7e2b7de5d75d0605f104f2ad728708
	touch toon_built

clean:
	rm -rf include
	rm -f toon_built

