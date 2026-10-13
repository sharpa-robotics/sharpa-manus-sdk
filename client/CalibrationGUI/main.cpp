#include "CalibrationApp.hpp"
#include "ClientLogging.hpp"

using namespace ManusSDK;

int main(int argc, char* argv[])
{
    ClientLog::print("Starting Calibration GUI Application");

    CalibrationApp app;

    if (!app.Initialize())
    {
        ClientLog::error("Failed to initialize application");
        return 1;
    }

    ClientLog::print("Application initialized successfully");
    app.Run();

    ClientLog::print("Application shutting down");
    return 0;
}

