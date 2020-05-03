#ifndef CONFIG_H
    #define CONFIG_H

    /* Define INDI Data Dir */
    #cmakedefine INDI_DATA_DIR "@INDI_DATA_DIR@"

    /* Define Driver version */
    #define AFF_MAJOR_VERSION @VERSION_MAJOR@
    #define AFF_MINOR_VERSION @VERSION_MINOR@
#endif // CONFIG_H
