#ifndef GETOPT_H
#define GETOPT_H

#ifdef __cplusplus
extern "C" {
#endif

extern char* optarg;

int getopt(int nargc, char * const nargv[], const char *ostr) ;

#ifdef __cplusplus
}
#endif

#endif
