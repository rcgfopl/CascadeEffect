#define LOG_SIZE 26

/**
 * This represents current logging status
 *
 * -1 means no file open.
 *  0 means open but no room.
 *  Otherwise, represents the number of times addLog can be called before filling the file.
 */
int logsStatus = -1;

TFileHandle logFile;

/**
 * Open a log file, deleting it if a previous one exists
 *
 * This must be called before addLog and finishLogs.
 */
void startLogs(const string filename, int numLogs)
{
	TFileIOResult ioResult;

	Delete(filename, ioResult);

	word size = LOG_SIZE * numLogs;
	OpenWrite(logFile, ioResult, filename, size);

	logsStatus = numLogs;
}

/**
 * Adds a line to the log file, in the format:
 *
 * LABEL 000 111 222 333 444
 *
 * The name label should be five or fewer bytes.
 */
void addLog(const string name, int r0, int r1, int r2, int r3, int r4)
{
	if (logsStatus > 0) {
		TFileIOResult ioResult;

		string logResult;
		sprintf(logResult, "%s %d %d %d %d %d\n", name, r0, r1, r2, r3, r4);
		WriteText(logFile, ioResult, logResult);

		--logsStatus;
	}
}

/**
 * Close the log file and reset logsStatus to -1
 */
void finishLogs()
{
	TFileIOResult ioResult;
	Close(logFile, ioResult);
	logsStatus = -1;
}
