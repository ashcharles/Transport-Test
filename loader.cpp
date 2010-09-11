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
	lt_dlhandle handle;
	void * func;
	int (*myfunc)(int, int);
	int z = 0;

	if ( lt_dlinit() != 0 ) { goto ltdl_fail; }
	handle = lt_dlopenext( "libmymod" );
	if ( ! handle ) { goto ltdl_fail; }
	func = lt_dlsym( handle, "Init" );
	if (! func ) { goto ltdl_fail; }
	myfunc = (int(*)(int,int)) func;

    z = myfunc(2,3);
	printf("Successfully opened! %d\n", z);

	if ( lt_dlexit() != 0 ) { goto ltdl_fail; }
	return 0;

// GOTO label to handle ltdl error. An irritating construct but we don't
// have exceptions available in ltdl.
ltdl_fail:
    printf("Libtool error: %s\nQuitting...\n", lt_dlerror());
    lt_dlexit(); // okay if it fails...we're just trying to clean up.
    return -1;
}
