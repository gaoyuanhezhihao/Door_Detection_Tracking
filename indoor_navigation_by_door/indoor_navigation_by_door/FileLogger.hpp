#ifndef FILELOGGER_HPP_INCLUDED
#define FILELOGGER_HPP_INCLUDED
#include <fstream>
#include <ctime>
#include <string>
#include <chrono>
using namespace std;
namespace ige {

	class FileLogger {

	public:


		// If you can´t/dont-want-to use C++11, remove the "class" word after enum
		enum class e_logType { LOG_ERROR, LOG_WARNING, LOG_INFO };


		// ctor (remove parameters if you don´t need them)
		explicit FileLogger(const char *engine_version, const char *fname = "ige_log.txt")
			: numWarnings(0U),
			numErrors(0U)
		{

			myFile.open(fname);

			// Write the first lines
			if (myFile.is_open()) {
				myFile << "My Game Engine, version " << engine_version << std::endl;
				myFile << "Log file created" << std::endl << std::endl;
			} // if

		}


		// dtor
		~FileLogger() {

			if (myFile.is_open()) {
				myFile << std::endl << std::endl;

				// Report number of errors and warnings
				myFile << numWarnings << " warnings" << std::endl;
				myFile << numErrors << " errors" << std::endl;

				myFile.close();
			} // if

		}


		// Overload << operator using log type
		friend FileLogger &operator << (FileLogger &logger, const e_logType l_type) {
			time_t now_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
			char now_time_format[100];
			ctime_s(now_time_format, 100, &now_time);
			switch (l_type) {
			case ige::FileLogger::e_logType::LOG_ERROR:
				logger.myFile << "[ERROR]: " << "  (" << now_time_format  << ") ";
				++logger.numErrors;
				break;

			case ige::FileLogger::e_logType::LOG_WARNING:
				logger.myFile << "[WARNING]: " << "(" << now_time_format << ") ";
				++logger.numWarnings;
				break;

			default:
				logger.myFile << "[INFO]: " << "   (" << now_time_format << ") ";
				break;
			} // sw


			return logger;

		}


		// Overload << operator using C style strings
		// No need for std::string objects here
		template <typename T>
		friend FileLogger &operator << (FileLogger &logger, T text) {

			logger.myFile << text << std::endl;
			return logger;

		}


		// Make it Non Copyable (or you can inherit from sf::NonCopyable if you want)
		FileLogger(const FileLogger &) = delete;
		FileLogger &operator= (const FileLogger &) = delete;



	private:

		std::ofstream           myFile;

		unsigned int            numWarnings;
		unsigned int            numErrors;

	}; // class end

}  // namespace
int del_file_logger();
int init_file_logger(const char *engine_version, const char *fname = "ige_log.txt");
extern ige::FileLogger * pGlobalLogger;
template <typename T>
int write_global_logger(T msg, char log_type)
{
	switch (log_type)
	{
	case 'e'://error
		*pGlobalLogger << ige::FileLogger::e_logType::LOG_ERROR << msg;
		break;
	case 'w'://warning
		*pGlobalLogger << ige::FileLogger::e_logType::LOG_WARNING << msg;
		break;
	case 'i':// information
		*pGlobalLogger << ige::FileLogger::e_logType::LOG_INFO << msg;
		break;
	default:
		return -1;
	}
	return 0;
}
#endif // FILELOGGER_HPP_INCLUDED
