// ****************************************************************
// (C) Copyright International Business Machines Corporation 2017
// Author: Gou Peng Fei (shgoupf@cn.ibm.com)
// ****************************************************************
#ifndef _NVDLA_CAPI_TEST_H_
#define _NVDLA_CAPI_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif
int nvdla_capi_test(char* loadable, char* input_path, char* image, int normalize, int rawdump, float mean[4]);
#ifdef __cplusplus
}
#endif

#endif
