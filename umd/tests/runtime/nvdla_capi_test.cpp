// ****************************************************************
// (C) Copyright International Business Machines Corporation 2017
// Author: Gou Peng Fei (shgoupf@cn.ibm.com)
// ****************************************************************

#include "nvdla_capi_test.h"
#include "ErrorMacros.h"
#include "RuntimeTest.h"
#include "Server.h"

#include "nvdla_os_inf.h"

#include <cstring>
#include <iostream>
#include <cstdlib> // system

static TestAppArgs defaultTestAppArgs = TestAppArgs();

static NvDlaError testSetup(const TestAppArgs* appArgs, TestInfo* i)
{
    NvDlaError e = NvDlaSuccess;

    std::string imagePath = "";
    NvDlaStatType stat;

    // Do input paths exist?
    if (std::strcmp(appArgs->inputName.c_str(), "") != 0)
    {
        e = NvDlaStat(appArgs->inputPath.c_str(), &stat);
        if (e != NvDlaSuccess)
            ORIGINATE_ERROR_FAIL(NvDlaError_BadParameter, "Input path does not exist: \"%s\"", appArgs->inputPath.c_str());

        imagePath = /* appArgs->inputPath + "/images/" + */appArgs->inputName;
        e = NvDlaStat(imagePath.c_str(), &stat);
        if (e != NvDlaSuccess)
            ORIGINATE_ERROR_FAIL(NvDlaError_BadParameter, "Image path does not exist: \"%s/%s\"", imagePath.c_str());
    }

    return NvDlaSuccess;

fail:
    return e;
}

static NvDlaError launchServer(const TestAppArgs* appArgs)
{
    NvDlaError e = NvDlaSuccess;
    TestInfo testInfo;

    testInfo.dlaServerRunning = false;
    PROPAGATE_ERROR_FAIL(runServer(appArgs, &testInfo));

fail:
    return e;
}

static NvDlaError launchTest(const TestAppArgs* appArgs)
{
    NvDlaError e = NvDlaSuccess;
    TestInfo testInfo;

    testInfo.dlaServerRunning = false;
    PROPAGATE_ERROR_FAIL(testSetup(appArgs, &testInfo));

    PROPAGATE_ERROR_FAIL(run(appArgs, &testInfo));
    
fail:
    return e;
}

int nvdla_capi_test(char* loadable, char* input_path, char* image, int normalize, int rawdump)
{
    NvDlaError e = NvDlaError_TestApplicationFailed;
    bool inputPathSet = false;
    //bool serverMode = false;
    TestAppArgs testAppArgs = defaultTestAppArgs;
    testAppArgs.inputPath = std::string(input_path);
    testAppArgs.inputName = std::string(image);
    testAppArgs.loadableName = std::string(loadable);
    testAppArgs.normalize_value = normalize;
    testAppArgs.rawOutputDump = (rawdump > 0);
    NVDLA_UNUSED(inputPathSet);

    ///* Check if any mandatory arguments are missing */
    //if (strcmp(testAppArgs.loadableName.c_str(), "") == 0 && !serverMode) {
    //    showHelp = true;
    //    missingArg = true;
    //}

    //if (serverMode)
    //{
    //    e = launchServer(&testAppArgs);
    //}
    //else
    //{
        // Launch
        e = launchTest(&testAppArgs);
    //}

    if (e != NvDlaSuccess)
    {
        return EXIT_FAILURE;
    }
    else
    {
        NvDlaDebugPrintf("Test pass\n");
        return EXIT_SUCCESS;
    }

    return EXIT_SUCCESS;
}
