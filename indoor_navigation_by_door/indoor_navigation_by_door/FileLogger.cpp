#include "FileLogger.hpp"
#include <fstream>
#include <ctime>
#include <string>
// Use the namespace you want

ige::FileLogger * pGlobalLogger = nullptr;
int init_file_logger(const char *engine_version, const char *fname)
{
	pGlobalLogger = new ige::FileLogger(engine_version, fname);
	return 0;
}
int del_file_logger()
{
	delete pGlobalLogger;
	return 0;
}
