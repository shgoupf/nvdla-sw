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

int nvdla_capi_test(int argc, char* argv[])
{
    NvDlaError e = NvDlaError_TestApplicationFailed;
    TestAppArgs testAppArgs = defaultTestAppArgs;
    bool showHelp = false;
    bool unknownArg = false;
    bool missingArg = false;
    bool inputPathSet = false;
    bool serverMode = false;
    NVDLA_UNUSED(inputPathSet);

    NvS32 ii = 1;
    while(true)
    {
        if (ii >= argc)
            break;

        const char* arg = argv[ii];

        if (std::strcmp(arg, "-h") == 0) // help
        {
            // Print usage
            showHelp = true;
            break;
        }
        if (std::strcmp(arg, "-s") == 0) // server mode
        {
            // Print usage
            serverMode = true;
            break;
        }
        else if (std::strcmp(arg, "-i") == 0) // input path
        {
            if (ii+1 >= argc)
            {
                // Expecting another parameter
                showHelp = true;
                break;
            }

            testAppArgs.inputPath = std::string(argv[++ii]);
            inputPathSet = true;
        }
        else if (std::strcmp(arg, "--image") == 0) // imagename
        {
            if (ii+1 >= argc)
            {
                // Expecting another parameter
                showHelp = true;
                break;
            }

            testAppArgs.inputName = std::string(argv[++ii]);
        }
        else if (std::strcmp(arg, "--loadable") == 0)
        {
            if (ii+1 >= argc)
            {
                // Expecting another parameter
                showHelp = true;
                break;
            }

            testAppArgs.loadableName = std::string(argv[++ii]);
        }
        else if (std::strcmp(arg, "--normalize") == 0) // normalize value
        {
            if (ii+1 >= argc)
            {
                // Expecting another parameter
                showHelp = true;
                break;
            }

            testAppArgs.normalize_value = atoi(argv[++ii]);
        }
        else if (std::strcmp(arg, "--rawdump") == 0)
        {
            testAppArgs.rawOutputDump = true;
        }
        else // unknown
        {
            // Unknown argument
            unknownArg = true;
            showHelp = true;
            break;
        }

        ii++;
    }

    NvDlaDebugPrintf("Hello NVDLA!\n");
    std::cout << "Hello NVDLA!" << std::endl;

    /* Check if any mandatory arguments are missing */
    if (strcmp(testAppArgs.loadableName.c_str(), "") == 0 && !serverMode) {
        showHelp = true;
        missingArg = true;
    }

    if (showHelp)
    {
        NvDlaDebugPrintf("Usage: %s [-options] --loadable <loadable_file>\n", argv[0]);
        NvDlaDebugPrintf("where options include:\n");
        NvDlaDebugPrintf("    -h                    print this help message\n");
        NvDlaDebugPrintf("    -s                    launch test in server mode\n");
        NvDlaDebugPrintf("    --image <file>        input jpg/pgm file\n");
        NvDlaDebugPrintf("    --normalize <value>   normalize value for input image\n");
        NvDlaDebugPrintf("    --rawdump             dump raw dimg data\n");

        if (unknownArg || missingArg)
            return EXIT_FAILURE;
        else
            return EXIT_SUCCESS;
    }

    if (serverMode)
    {
        e = launchServer(&testAppArgs);
    }
    else
    {
        // Launch
        e = launchTest(&testAppArgs);
    }

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
