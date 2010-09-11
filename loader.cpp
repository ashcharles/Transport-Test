/*
 * loader.cpp
 *
 *  Created on: 2010-09-10
 *      Author: ash
 */

#include <ltdl.h>
#include <stdio.h>
#include <string>

//------------------------------------------------------------------------------
// handle errors from ltdl
int ltdlError() {
	printf("Libtool error: %s\nQuitting...\n", lt_dlerror());
	lt_dlexit(); // okay if it fails...we're just trying to clean up.
	return -1;
}
//------------------------------------------------------------------------------
// dlopen chatterbox controller and run
int main(int argc, char* argv[]) {
	// build a string of command line args to pass to controller.
	std::string args;
	for (int i = 1; i < argc; i++) {
		args.append(argv[i]);
	}

	// load module
	if (lt_dlinit() != 0) {
		return ltdlError();
	}
	lt_dlhandle handle = lt_dlopenext("transport");
	if (!handle) {
		return ltdlError();
	}
	int (*initCB_func)(std::string) = (int(*)(std::string)) lt_dlsym(handle, "InitCB");
	if (!initCB_func) {
		return ltdlError();
	}
	initCB_func(args);

	if (lt_dlexit() != 0) {
		return ltdlError();
	}
	return 0;
}
