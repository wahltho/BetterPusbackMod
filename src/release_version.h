#ifndef BP_RELEASE_VERSION_H
#define BP_RELEASE_VERSION_H

#include <limits.h>
#include <stdbool.h>
#include <string.h>

typedef struct {
    unsigned int part[3];
    bool beta;
} bp_release_version_t;

/* BPB release tags use vMAJOR.MINOR[.PATCH] or betaMAJOR.MINOR[.PATCH]. */
static inline bool
bp_release_version_parse(const char *tag, bp_release_version_t *version)
{
    if (tag == NULL)
        return false;

    memset(version, 0, sizeof (*version));
    if (strncmp(tag, "beta", 4) == 0) {
        version->beta = true;
        tag += 4;
    } else if (*tag == 'v') {
        tag++;
    }

    for (unsigned int i = 0; i < 3; i++) {
        if (*tag < '0' || *tag > '9')
            return false;
        while (*tag >= '0' && *tag <= '9') {
            unsigned int digit = (unsigned int)(*tag++ - '0');
            if (version->part[i] > (UINT_MAX - digit) / 10)
                return false;
            version->part[i] = version->part[i] * 10 + digit;
        }
        if (*tag == '\0')
            return true;
        if (*tag++ != '.')
            return false;
    }
    return false;
}

static inline bool
bp_release_version_is_newer(const char *available, const char *installed)
{
    bp_release_version_t candidate, current;

    /* Unknown or malformed tags must not produce an update notification. */
    if (!bp_release_version_parse(available, &candidate) ||
        !bp_release_version_parse(installed, &current))
        return false;

    for (unsigned int i = 0; i < 3; i++) {
        if (candidate.part[i] != current.part[i])
            return candidate.part[i] > current.part[i];
    }
    return current.beta && !candidate.beta;
}

#endif /* BP_RELEASE_VERSION_H */
