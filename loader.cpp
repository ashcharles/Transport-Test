/*
 * loader.cpp
 *
 *  Created on: 2010-09-10
 *      Author: ash
 */

#include <ltdl.h>
#include <stdio.h>

int main()
{
	int errors = lt_dlinit();
	if (errors) {
		printf("Libtool error: %s\nQuitting...\n", lt_dlerror());
		exit(-1);
	}
	lt_dlhandle handle = lt_dlopenext( "transport" );
	if ( ! handle ) {
		printf("Libtool error: %s\nQuitting...\n", lt_dlerror());
		exit(-1);
	}
	void * func = lt_dlsym( handle, "Init" );
	if (! handle ) {
		printf("Libtool error: %s\nQuitting...\n", lt_dlerror());
		exit(-1);
	}
	printf("Successfully opened!\n");
	if ( lt_dlexit() != 0 ) {
		printf("Libtool error: %s\nQuitting...\n", lt_dlerror());
		exit(-1);
	}
	return 0;
}
