# enable out-of-source builds with cmake
all:
	mkdir -p Build
	cd Build && cmake -DCMAKE_BUILD_TYPE=Debug ..
	cd Build && make
	cd Build && ln -sf ../source2sink.txt .
	cd Build && ln -sf ../sink2source.txt .

docs:
	doxygen Doxyfile

clean:
	rm -rf Build
	rm -rf Docs
