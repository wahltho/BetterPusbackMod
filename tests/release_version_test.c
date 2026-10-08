#include <assert.h>
#include <stdio.h>

#include "release_version.h"

int
main(void)
{
    assert(!bp_release_version_is_newer("v1.15", "v1.16"));
    assert(!bp_release_version_is_newer("v1.15", "beta1.16"));
    assert(!bp_release_version_is_newer("v1.16", "v1.16"));
    assert(!bp_release_version_is_newer("v1.16.0", "v1.16"));
    assert(!bp_release_version_is_newer("v1.16", "1.16"));
    assert(bp_release_version_is_newer("v1.16", "v1.15"));
    assert(bp_release_version_is_newer("v1.16.1", "v1.16"));
    assert(bp_release_version_is_newer("v1.100", "v1.99"));
    assert(bp_release_version_is_newer("v1.10", "v1.9"));
    assert(bp_release_version_is_newer("v2.0", "v1.100"));
    assert(!bp_release_version_is_newer("v1.99", "v2.0"));
    assert(bp_release_version_is_newer("v1.16", "beta1.16"));
    assert(!bp_release_version_is_newer("beta1.16", "v1.16"));
    assert(!bp_release_version_is_newer("beta1.16", "beta1.16"));

    const char *invalid[] = {
        NULL, "", "v", "beta", "unknown", "v1.", "v1..16", "v1.16.",
        "v1.16.0.1", "v1.16garbage", "v-1.16", "v1.-16", "v+1.16",
        "v1.16\n", "v999999999999999999999999999999", "v1.99999999999999999999"
    };
    for (unsigned int i = 0; i < sizeof (invalid) / sizeof (invalid[0]); i++) {
        assert(!bp_release_version_is_newer(invalid[i], "v1.16"));
        assert(!bp_release_version_is_newer("v1.17", invalid[i]));
    }

    puts("Release version comparison tests passed.");
    return 0;
}
