#include <iostream>
#include <iomanip>
#include <cmath>

#include <kord/api/kord_receive_interface.h>
#include <kord/api/kord_control_interface.h>
#include <kord/api/kord.h>
#include <kord/utils/utils.h>

#include <csignal>
#include <chrono>
#include <sstream>

#include <fstream>


using namespace kr2;
static bool g_run = true;

void signal_handler( int a_signum ) {
    psignal(a_signum, "[KORD-API]");
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
    struct ExtraOptions {
        double A = 0.1;
        double T = 16;
    } extras;
    static kr2::utils::LaunchParameters::ExternalArgParser ep = [argc, &argv, &extras](int index)->void {
    static kr2::utils::SOALongOptions ex_options{std::array<kr2::utils::LongOption, 2>{
        kr2::utils::LongOption{{  "amplitude",   required_argument, nullptr,  'a' }, "amplitude in radians for every joint"},
        kr2::utils::LongOption{{  "period",  required_argument, nullptr,  'f' }, "move period in sec"},
    }};
    
    if (index <= kr2::utils::LaunchParameters::INVALID_INDEX) {
        std::cout << ex_options.helpString() << "\n";
        return;
    }

    int option_index = 0;
    optind = index;
    int opt = getopt_long(argc, argv, "a:f:", ex_options.getLongOptions(), &option_index);

    switch (opt) {
        case  'a':
            if (optarg){
            extras.A = std::stod(optarg);
            }
            break;
        case 'f':
            if (optarg){
            extras.T = std::stod(optarg);
            }
            break;
        default:
            std::cout << "Unknown option found" << " optidx: " << optind << ", argc: " << argc << "\n";
            exit(EXIT_FAILURE);
            break;
    }
};
    kr2::utils::LaunchParameters lp = kr2::utils::LaunchParameters::processLaunchArguments(argc, argv, ep);

    std::cout << "Executing with\n";
    std::cout << "Amplitude: " << extras.A << '\n';
    std::cout << "Period: " << extras.T << '\n';

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


    kord::ControlInterface ctl_iface(kord);
    kord::ReceiverInterface rcv_iface(kord);


    if (!kord->connect()) {
        std::cout << "Connecting to KR failed\n";
        return EXIT_FAILURE;
    }

    std::array<double, 7UL> q, qd, qdd, torque;
    
    g_run = true;
    
    // Obtain initial q values
    if (!kord->syncRC()){
        std::cout << "Sync RC failed.\n";
        return EXIT_FAILURE;
    }
    std::cout << "Sync Captured \n";
    rcv_iface.fetchData();
    std::array<double, 7UL> start_q = rcv_iface.getJoint(kord::ReceiverInterface::EJointValue::S_ACTUAL_Q);

    std::cout << "Read initial joint configuration:\n";
    for( double angl: start_q )
        std::cout << (angl/3.14)*180 << " ";

    std::cout << "\n";

    unsigned int k = 0; // main time counter
    double A       = extras.A;
    double t       = extras.T;
    double w = 2 * M_PI / t;

    if (extras.A != -1){

    }

    double ts = 0.004; // 500 Hz, for 250Hz choose 0.004

    double now_time = 0;
    bool mode_switch = false;

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    while(true) {
        now_time = k * ts;
        if (now_time > t){
            mode_switch = !mode_switch;
            k = 0;
            now_time = 0;
            if (!g_run){
                break;
            }
        }
        if (!mode_switch){
            for (size_t i = 0; i<7; i++) 
            {
                //double b = i * 1.0e-4 * 2;
                q  [i]    =  A  / t * now_time - A  / t / w * std::sin(w * now_time) + start_q[i];
                qd [i]    = A  / t - A  / t / w * std::cos(w * now_time) * w;
                qdd[i]    = A  / t / w * std::sin(w * now_time) * w;
                torque[i] = 0.0; // it will be computed automatically in case of zero
            }
        }
        else{
            for (size_t i = 0; i<7; i++) 
            {
                //double b = i * 1.0e-4 * 2;
                q  [i]    = 2 * A - A  / t * (now_time + t) + A  / t / w * std::sin(w * (now_time + t)) + start_q[i];
                qd [i]    = -1 * A  / t  +  A  / t / w * std::cos(w * now_time) * w;
                qdd[i]    = -1 * A  / t / w * std::sin(w * now_time) * w;
                torque[i] = 0.0; // it will be computed automatically in case of zero
            }
        }
        
        // torque = {-3.0, -16., 0.15, 11.2, 0.7, -0.36, 0.5}; // some torque for testing
        k++;

        { // before waitSync
            auto now_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
            stats_values_.push_back((now_ts.time_since_epoch().count()));
        }

        if (!kord->waitSync(std::chrono::milliseconds(10)))
        {
            std::cout << "Sync wait timed out, exit \n";
            break;
        }

        { // after waitSync
            auto now_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
            stats_values_.push_back((now_ts.time_since_epoch().count()));
        }

        ctl_iface.directJControl(q, qd, qdd, torque);

        { // after direct control
            auto now_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
            stats_values_.push_back((now_ts.time_since_epoch().count()));
        }
        
        rcv_iface.fetchData();

        { // after fetch data
            auto now_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
            stats_values_.push_back((now_ts.time_since_epoch().count()));
        }
        if (rcv_iface.systemAlarmState()  ||
            lp.runtimeElapsed())
        { break; }
    }

    std::cout << "Robot stopped\n";
    std::cout << rcv_iface.getFormattedInputBits()  << std::endl;
    std::cout << rcv_iface.getFormattedOutputBits() << std::endl;
    std::cout << "SystemAlarmState:\n";
    std::cout << rcv_iface.systemAlarmState() << "\n";
    std::cout << "SystemAlarmState's Category:\n";
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
    std::cout << "RCState: " << rcv_iface.getRobotSafetyFlags() << "\n";
    
    kord->printStats(rcv_iface.getStatisticsStructure());


    std::string filePath = "./stats_file_kord_api.csv";
    std::vector<std::string> columnNames = { "before_wait_sync", "after_wait_sync", "after_movej", "after_fetch_data"};
    saveToCSV(stats_values_, columnNames, filePath);

    return 0;
}
