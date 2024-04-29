

#include <iostream>
#include <cmath>
#include <limits>

#include <kord/api/kord_receive_interface.h>
#include <kord/api/kord_control_interface.h>
#include <kord/api/kord.h>
#include <kord/utils/utils.h>

#include <csignal>
#include <chrono>
#include <sstream>
#include <iomanip>

#include <fstream>


using namespace kr2;
static volatile bool g_run = true;

void signal_handler( int signum ) {
    psignal(signum, "[KORD-API]");
    g_run = false;
}

void saveToCSV(const std::vector<int64_t>& values, const std::vector<std::string>& columnNames, const std::string& filename) {
    const int columns_number = columnNames.size();
    std::ofstream outputFile(filename);
    if (outputFile.is_open()) {
        // Write column names
        for (size_t i = 0; i < columnNames.size(); ++i) {
            outputFile << columnNames[i];
            if (i != columnNames.size() - 1) {
                outputFile << ",";
            }
        }
        outputFile << "\n";
        for (size_t i = 0; i < values.size(); i += columns_number) {
            for (size_t j = i; j < i + columns_number && j < values.size(); ++j) {
                outputFile << values[j];
                if (j != i + (columns_number - 1) && j != values.size() - 1) {
                    outputFile << ",";
                }
            }
            outputFile << "\n";
        }
        outputFile.close();
        std::cout << "Values have been saved to " << filename << "\n";
    } else {
        std::cerr << "Unable to open the file\n";
    }
}

std::vector<int64_t> stats_values_;


int main(int argc, char * argv[])
{

        stats_values_.clear();
    stats_values_.reserve(10000000); // for 5 minutes at least

    kr2::utils::LaunchParameters lp = kr2::utils::LaunchParameters::processLaunchArguments(argc, argv);

    if (lp.help_ || !lp.valid_) {
        //lp.printUsage(true);
        return EXIT_SUCCESS;
    }

    signal(SIGINT, signal_handler); 

    if (lp.useRealtime()){
        if (!kr2::utils::realtime::init_realtime_params(lp.rt_prio_)){
            std::cerr << "Failed to start with realtime priority\n";
            lp.printUsage(true);
            return EXIT_FAILURE;
        }
    }
    
    std::cout << "Connecting to: " << lp.remote_controller_ << ":" << lp.port_ << "\n";
    std::cout << "[KORD-API] Session ID: " << lp.session_id_ << std::endl;

    std::shared_ptr<kord::KordCore> kord(new kord::KordCore(
        lp.remote_controller_,
        lp.port_,
        lp.session_id_,
        kord::UDP_CLIENT));
    
    // insert code here...


    kord::ControlInterface ctl_iface(kord);
    kord::ReceiverInterface rcv_iface(kord);


    if (!kord->connect()) {
        std::cout << "Connecting to KR failed\n";
        return EXIT_FAILURE;
    }

    // TODO: prepare initial q
    std::array<double, 6UL> tcp_target;

    g_run = true;
    
    
    // Obtain initial q values
    if (!kord->syncRC()){
        std::cout << "Sync RC failed.\n";
        return EXIT_FAILURE;
    }
    std::cout << "Sync Captured \n";
    rcv_iface.fetchData();
    std::vector<std::variant<double, int>> returned_vec_start_tcp = rcv_iface.getFrame(kord::EFrameID::TCP_FRAME,kord::EFrameValue::POSE_VAL_REF_WF);
    std::array<double, 6UL> start_tcp;
    for (size_t i = 0; i < returned_vec_start_tcp.size(); ++i) {
        start_tcp[i] = std::get<double>(returned_vec_start_tcp[i]);
    }


    for( double &p: start_tcp )
        std::cout << p << ";";

    std::cout << std::endl;

    unsigned int i   = 0;
    double a         = 0.1;
    double t         = 0.0;
    double tt_value  = 0.008;
    double bt_value  = 0.004;

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    while(g_run) {
        //
        // calculation
        // update tcp target
        //
        tcp_target[0] = start_tcp[0];
        tcp_target[1] = start_tcp[1];
        tcp_target[2] = (std::cos(t * 2e-4)-1)*a + start_tcp[2];
        tcp_target[3] = start_tcp[3];
        tcp_target[4] = start_tcp[4];
        tcp_target[5] = start_tcp[5];
        t = i * 7;
        i++;
        


        { // before waitSync
            auto now_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
            stats_values_.push_back((now_ts.time_since_epoch().count()));
        }


        //kord->spin();
        if (!kord->waitSync(std::chrono::milliseconds(10))){
            std::cout << "Sync wait timed out, exit \n";
            break;
        }


        { // after waitSync
           auto now_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
            stats_values_.push_back((now_ts.time_since_epoch().count()));
        }


        ctl_iface.moveL(tcp_target,
                        kr2::kord::TrackingType::TT_TIME,    tt_value,
                        kr2::kord::BlendType::BT_TIME,       bt_value,
                        kr2::kord::OverlayType::OT_VIAPOINT);

        { // after moveL
           auto now_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
            stats_values_.push_back((now_ts.time_since_epoch().count()));
        }
        rcv_iface.fetchData();

        { // after fetchData
           auto now_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
            stats_values_.push_back((now_ts.time_since_epoch().count()));
        }
        if (rcv_iface.systemAlarmState() ||
            lp.runtimeElapsed())
        {
            break;
        }
    }

    std::cout << "Robot stopped\n";
    std::cout << rcv_iface.getFormattedInputBits()  << std::endl;
    std::cout << rcv_iface.getFormattedOutputBits() << std::endl;
    std::cout << "SystemAlarmState:\n";
    std::cout << rcv_iface.systemAlarmState() << "\n";
        switch (rcv_iface.systemAlarmState() & 0b1111){
        case kr2::kord::protocol::EEventGroup::eUnknown:
            std::cout << "No alarms" << '\n';
            break;
        case kr2::kord::protocol::EEventGroup::eSafetyEvent:
            std::cout << "Safety Event" << '\n';
            break;
        case kr2::kord::protocol::EEventGroup::eSoftStopEvent:
            std::cout << "Soft Stop Event" << '\n';
            break;
        case kr2::kord::protocol::EEventGroup::eKordEvent:
            std::cout << "Kord Event" << '\n';
            break;
    }

    std::cout << "Safety flags: " << rcv_iface.getRobotSafetyFlags() << "\n";
    std::cout << "Runtime: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()/1000.0 << " [s]\n";
    std::cout << "SafetyFlags: " << rcv_iface.getRobotSafetyFlags() << "\n";
    std::cout << "MotionFlags: " << rcv_iface.getMotionFlags() << "\n";
    std::cout << "RCState: " << rcv_iface.getRobotSafetyFlags();

    kord->printStats(rcv_iface.getStatisticsStructure());

    std::string filePath = "./stats_file_kord_api.csv";
    std::vector<std::string> columnNames = { "before_wait_sync", "after_wait_sync", "after_movel", "after_fetch_data"};
    saveToCSV(stats_values_, columnNames, filePath);

    //NA when no stats
    return 0;
}
