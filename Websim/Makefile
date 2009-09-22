# enable out-of-source builds with cmake
all:
	mkdir -p Build
	cd Build && cmake ..
	cd Build && make

docs:
	doxygen Doxyfile

clean:
	rm -rf Build
	rm -rf Docs
