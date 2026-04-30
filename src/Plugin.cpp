#include "XPlaneTruthCapture/Version.h"

#include "XPLMDataAccess.h"
#include "XPLMDefs.h"
#include "XPLMMenus.h"
#include "XPLMPlanes.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;

struct DataRefSpec {
    const char *path;
    const char *group;
    bool required;
};

struct DataRefEntry {
    std::string path;
    std::string group;
    bool required{false};
    XPLMDataRef ref{nullptr};
    int type_mask{0};
    int writable{0};
    int array_len{0};
};

struct Config {
    std::string capture_rate{"every_frame"};
    int max_array_values{32};
    bool include_default_datarefs{true};
};

struct FrameSample {
    std::uint64_t frame_id{0};
    std::int64_t host_ns{0};
    bool first_frame{false};
    float elapsed_since_last_call{0.0f};
    float elapsed_since_last_flight_loop{0.0f};
    int loop_counter{0};
    int xplm_cycle_number{0};
    std::vector<std::string> values;
};

struct NumericSeries {
    std::vector<double> values;

    void reset()
    {
        values.clear();
    }

    void observe(double value)
    {
        if (std::isfinite(value)) {
            values.push_back(value);
        }
    }

    double mean() const
    {
        if (values.empty()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        double sum = 0.0;
        for (const double value : values) {
            sum += value;
        }
        return sum / static_cast<double>(values.size());
    }

    double percentile(double q) const
    {
        if (values.empty()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        std::vector<double> copy = values;
        std::sort(copy.begin(), copy.end());
        const auto idx = static_cast<std::size_t>(std::round(q * static_cast<double>(copy.size() - 1)));
        return copy[std::min(idx, copy.size() - 1)];
    }
};

struct RunStats {
    std::int64_t host_start_ns{0};
    std::int64_t host_stop_ns{0};
    NumericSeries elapsed_call_s;
    NumericSeries elapsed_flight_loop_s;
    NumericSeries frame_period_s;
    double first_flight_time_s{std::numeric_limits<double>::quiet_NaN()};
    double last_flight_time_s{std::numeric_limits<double>::quiet_NaN()};
    double previous_flight_time_s{std::numeric_limits<double>::quiet_NaN()};
    double monotonic_flight_duration_s{0.0};
    std::uint64_t sim_time_resets{0};
    std::uint64_t paused_rows{0};
    std::uint64_t sim_speed_not_1_rows{0};
    std::uint64_t auto_markers{0};
};

struct AutoEventState {
    bool stationary{false};
    bool takeoff_roll{false};
    bool airborne{false};
    bool hover_candidate{false};
    bool landing{false};
    bool stopped_after_landing{false};
    int stationary_count{0};
    int takeoff_roll_count{0};
    int airborne_count{0};
    int hover_count{0};
    int landing_count{0};
    int stopped_count{0};
};

const DataRefSpec kDefaultDataRefs[] = {
    {"sim/time/total_flight_time_sec", "timing", true},
    {"sim/time/total_running_time_sec", "timing", true},
    {"sim/time/zulu_time_sec", "timing", false},
    {"sim/time/local_time_sec", "timing", false},
    {"sim/time/local_date_days", "timing", false},
    {"sim/time/paused", "timing", false},
    {"sim/time/sim_speed", "timing", false},
    {"sim/operation/misc/frame_rate_period", "timing", false},

    {"sim/aircraft/view/acf_descrip", "aircraft", false},
    {"sim/aircraft/view/acf_author", "aircraft", false},
    {"sim/aircraft/view/acf_tailnum", "aircraft", false},
    {"sim/aircraft/view/acf_ICAO", "aircraft", false},
    {"sim/aircraft/weight/acf_m_empty", "aircraft", false},
    {"sim/aircraft/weight/acf_m_fuel_tot", "aircraft", false},
    {"sim/aircraft/weight/acf_m_total", "aircraft", false},

    {"sim/flightmodel/position/latitude", "position", true},
    {"sim/flightmodel/position/longitude", "position", true},
    {"sim/flightmodel/position/elevation", "position", true},
    {"sim/flightmodel/position/y_agl", "position", false},
    {"sim/flightmodel/position/local_x", "position", true},
    {"sim/flightmodel/position/local_y", "position", true},
    {"sim/flightmodel/position/local_z", "position", true},
    {"sim/flightmodel/position/lat_ref", "position", false},
    {"sim/flightmodel/position/lon_ref", "position", false},

    {"sim/flightmodel/position/local_vx", "velocity", true},
    {"sim/flightmodel/position/local_vy", "velocity", true},
    {"sim/flightmodel/position/local_vz", "velocity", true},
    {"sim/flightmodel/position/local_ax", "velocity", false},
    {"sim/flightmodel/position/local_ay", "velocity", false},
    {"sim/flightmodel/position/local_az", "velocity", false},
    {"sim/flightmodel/position/groundspeed", "velocity", false},
    {"sim/flightmodel/position/hpath", "velocity", false},
    {"sim/flightmodel/position/vpath", "velocity", false},
    {"sim/cockpit2/tcas/targets/position/vx", "velocity", false},
    {"sim/cockpit2/tcas/targets/position/vy", "velocity", false},
    {"sim/cockpit2/tcas/targets/position/vz", "velocity", false},

    {"sim/flightmodel/position/q", "attitude", true},
    {"sim/flightmodel/position/psi", "attitude", true},
    {"sim/flightmodel/position/theta", "attitude", true},
    {"sim/flightmodel/position/phi", "attitude", true},
    {"sim/flightmodel/position/Prad", "attitude", false},
    {"sim/flightmodel/position/Qrad", "attitude", false},
    {"sim/flightmodel/position/Rrad", "attitude", false},
    {"sim/flightmodel/position/P", "attitude", false},
    {"sim/flightmodel/position/Q", "attitude", false},
    {"sim/flightmodel/position/R", "attitude", false},
    {"sim/flightmodel/position/alpha", "attitude", false},
    {"sim/flightmodel/position/beta", "attitude", false},

    {"sim/flightmodel/forces/g_axil", "accel", true},
    {"sim/flightmodel/forces/g_side", "accel", true},
    {"sim/flightmodel/forces/g_nrml", "accel", true},
    {"sim/flightmodel2/misc/gforce_axil", "accel", false},
    {"sim/flightmodel2/misc/gforce_side", "accel", false},
    {"sim/flightmodel2/misc/gforce_normal", "accel", false},
    {"sim/flightmodel/forces/faxil_plug_acf", "accel", false},
    {"sim/flightmodel/forces/fside_plug_acf", "accel", false},
    {"sim/flightmodel/forces/fnrml_plug_acf", "accel", false},

    {"sim/flightmodel/position/mag_psi", "magnetic", false},
    {"sim/flightmodel/position/magpsi", "magnetic", false},
    {"sim/flightmodel/position/magnetic_variation", "magnetic", false},
    {"sim/cockpit2/gauges/indicators/ground_track_mag_pilot", "magnetic", false},
    {"sim/cockpit2/gauges/indicators/ground_track_mag_copilot", "magnetic", false},
    {"sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot", "magnetic", false},
    {"sim/cockpit2/gauges/indicators/compass_heading_deg_mag", "magnetic", false},

    {"sim/flightmodel/position/indicated_airspeed", "airdata", false},
    {"sim/flightmodel/position/indicated_airspeed2", "airdata", false},
    {"sim/flightmodel/position/true_airspeed", "airdata", false},
    {"sim/cockpit2/gauges/indicators/airspeed_kts_pilot", "airdata", false},
    {"sim/cockpit2/gauges/indicators/true_airspeed_kts_pilot", "airdata", false},
    {"sim/weather/barometer_current_inhg", "weather", false},
    {"sim/weather/barometer_sealevel_inhg", "weather", false},
    {"sim/weather/aircraft/barometer_current_pas", "weather", false},
    {"sim/weather/aircraft/qnh_pas", "weather", false},
    {"sim/weather/region/qnh_pas", "weather", false},
    {"sim/weather/aircraft/temperature_ambient_deg_c", "weather", false},
    {"sim/weather/temperature_ambient_c", "weather", false},
    {"sim/weather/temperature_sealevel_c", "weather", false},
    {"sim/cockpit2/temperature/outside_air_temp_degc", "weather", false},
    {"sim/weather/region/atmosphere_alt_levels_m", "weather", false},
    {"sim/weather/aircraft/wind_speed_msc", "weather", false},
    {"sim/weather/aircraft/wind_direction_degt", "weather", false},
    {"sim/weather/wind_speed_kt", "weather", false},
    {"sim/weather/wind_direction_degt", "weather", false},

    {"sim/flightmodel/failures/onground_any", "ground", false},
    {"sim/flightmodel2/gear/on_ground", "ground", false},
    {"sim/flightmodel2/gear/tire_vertical_deflection_mtr", "ground", false},
    {"sim/flightmodel2/gear/tire_rotation_speed_rad_sec", "ground", false},

    {"sim/joystick/yoke_roll_ratio", "controls", false},
    {"sim/joystick/yoke_pitch_ratio", "controls", false},
    {"sim/joystick/yoke_heading_ratio", "controls", false},
    {"sim/cockpit2/controls/yoke_roll_ratio", "controls", false},
    {"sim/cockpit2/controls/yoke_pitch_ratio", "controls", false},
    {"sim/cockpit2/controls/yoke_heading_ratio", "controls", false},
    {"sim/cockpit2/controls/total_pitch_ratio", "controls", false},
    {"sim/cockpit2/controls/total_roll_ratio", "controls", false},
    {"sim/cockpit2/controls/total_heading_ratio", "controls", false},
    {"sim/cockpit2/controls/flap_ratio", "controls", false},
    {"sim/cockpit2/controls/speedbrake_ratio", "controls", false},
    {"sim/cockpit2/controls/wheel_brake_ratio", "controls", false},
    {"sim/cockpit2/controls/left_brake_ratio", "controls", false},
    {"sim/cockpit2/controls/right_brake_ratio", "controls", false},
    {"sim/cockpit2/controls/park_brake_valve", "controls", false},
    {"sim/cockpit2/controls/parking_brake_ratio", "controls", false},
    {"sim/cockpit2/engine/actuators/throttle_ratio_all", "controls", false},
    {"sim/cockpit2/engine/actuators/throttle_ratio", "controls", false},
    {"sim/joystick/joystick_axis_assignments", "controls", false},
    {"sim/joystick/joy_mapped_axis_value", "controls", false},
    {"sim/joystick/joy_mapped_axis_avail", "controls", false},
    {"sim/cockpit2/engine/actuators/prop_ratio", "controls", false},
    {"sim/cockpit2/engine/actuators/prop_angle_degrees", "controls", false},
    {"sim/cockpit2/engine/actuators/prop_pitch_deg", "controls", false},
    {"sim/cockpit2/engine/actuators/prop_mode", "controls", false},
    {"sim/flightmodel/engine/POINT_pitch_deg", "controls", false},
    {"sim/flightmodel/engine/POINT_pitch_deg_use", "controls", false},
    {"sim/flightmodel/engine/ENGN_thro", "controls", false},
    {"sim/flightmodel/engine/ENGN_thro_use", "controls", false},
    {"sim/flightmodel2/engines/throttle_used_ratio", "controls", false},
    {"sim/flightmodel2/wing/aileron1_deg", "controls", false},
    {"sim/flightmodel2/wing/aileron2_deg", "controls", false},
    {"sim/flightmodel2/wing/elevator1_deg", "controls", false},
    {"sim/flightmodel2/wing/elevator2_deg", "controls", false},
    {"sim/flightmodel2/wing/rudder1_deg", "controls", false},
    {"sim/flightmodel2/wing/rudder2_deg", "controls", false},
    {"sim/flightmodel2/controls/flap_handle_deploy_ratio", "controls", false},
    {"sim/flightmodel2/gear/is_chocked", "controls", false},
    {"sim/flightmodel2/gear/tire_abs_gain", "controls", false},
    {"sim/flightmodel2/gear/tire_steer_command_deg", "controls", false},

    {"sim/flightmodel/engine/ENGN_N1_", "engine", false},
    {"sim/flightmodel/engine/ENGN_N2_", "engine", false},
    {"sim/flightmodel/engine/ENGN_thro_use", "engine", false},
    {"sim/flightmodel/engine/ENGN_TRQ", "engine", false},
    {"sim/flightmodel/engine/ENGN_power", "engine", false},
    {"sim/flightmodel/engine/ENGN_rpm", "engine", false},
    {"sim/flightmodel/engine/ENGN_driv_TRQ", "engine", false},
    {"sim/flightmodel2/engines/prop_rotation_speed_rad_sec", "engine", false},
    {"sim/flightmodel2/engines/prop_pitch_deg", "engine", false},
    {"sim/flightmodel2/engines/prop_rotation_angle_deg", "engine", false},
    {"sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec", "engine", false},
    {"sim/flightmodel/engine/POINT_tacrad", "engine", false},
    {"sim/flightmodel/engine/POINT_prop_ang_deg", "engine", false},
    {"sim/flightmodel/engine/POINT_thrust", "engine", false},
    {"sim/flightmodel/engine/POINT_drag_TRQ", "engine", false},
    {"sim/flightmodel/engine/POINT_driv_TRQ", "engine", false},

    {"sim/weather/aircraft/wind_now_speed_msc", "weather", false},
    {"sim/weather/aircraft/wind_now_direction_degt", "weather", false},

    {"sim/flightmodel/weight/m_total", "mass", false},
    {"sim/flightmodel/weight/m_fixed", "mass", false},
    {"sim/flightmodel/weight/m_fuel_total", "mass", false},
    {"sim/flightmodel/misc/cgx_ref_to_default", "mass", false},
    {"sim/flightmodel/misc/cgz_ref_to_default", "mass", false},
};

Config gConfig;
std::vector<DataRefEntry> gDataRefs;
fs::path gPluginRoot;
fs::path gRunDir;
XPLMMenuID gMenuId = nullptr;
XPLMFlightLoopID gFlightLoopId = nullptr;

std::atomic<bool> gCapturing{false};
std::atomic<bool> gWriterStopping{false};
std::atomic<std::uint64_t> gFrameId{0};
std::atomic<std::uint64_t> gDroppedRows{0};
std::atomic<std::uint64_t> gWrittenRows{0};

std::mutex gQueueMutex;
std::condition_variable gQueueCv;
std::deque<FrameSample> gQueue;
std::thread gWriterThread;
RunStats gRunStats;
AutoEventState gAutoEvents;

std::mutex gEventMutex;
std::ofstream gFramesFile;
std::ofstream gEventsFile;
XPLMCommandRef gMarkCommand = nullptr;

std::string trim(const std::string &value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::string lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool parseBool(const std::string &value, bool fallback)
{
    const auto v = lower(trim(value));
    if (v == "true" || v == "1" || v == "yes" || v == "on") {
        return true;
    }
    if (v == "false" || v == "0" || v == "no" || v == "off") {
        return false;
    }
    return fallback;
}

double parseDouble(const std::string &value)
{
    if (value.empty() || value.find(';') != std::string::npos || value.front() == '<') {
        return std::numeric_limits<double>::quiet_NaN();
    }
    char *end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return parsed;
}

std::string jsonEscape(const std::string &value)
{
    std::ostringstream out;
    for (const char c : value) {
        switch (c) {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default: out << c; break;
        }
    }
    return out.str();
}

std::string csvEscape(const std::string &value)
{
    const bool must_quote = value.find_first_of(",\"\n\r") != std::string::npos;
    if (!must_quote) {
        return value;
    }
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (const char c : value) {
        if (c == '"') {
            escaped += "\"\"";
        } else {
            escaped.push_back(c);
        }
    }
    escaped.push_back('"');
    return escaped;
}

std::string hexByte(unsigned char value)
{
    constexpr char kHex[] = "0123456789ABCDEF";
    std::string out;
    out.push_back(kHex[(value >> 4) & 0x0F]);
    out.push_back(kHex[value & 0x0F]);
    return out;
}

std::string dataBytesToText(const std::vector<char> &bytes, int read)
{
    if (read <= 0) {
        return {};
    }

    int textLength = read;
    while (textLength > 0 && bytes[static_cast<std::size_t>(textLength - 1)] == '\0') {
        --textLength;
    }
    const auto nul = std::find(bytes.begin(), bytes.begin() + textLength, '\0');
    if (nul != bytes.begin() + textLength) {
        textLength = static_cast<int>(std::distance(bytes.begin(), nul));
    }

    int printable = 0;
    for (int i = 0; i < textLength; ++i) {
        const auto c = static_cast<unsigned char>(bytes[static_cast<std::size_t>(i)]);
        if (std::isprint(c) || c == '\t' || c == '\r' || c == '\n') {
            ++printable;
        }
    }

    if (textLength > 0 && printable >= std::max(1, (textLength * 9) / 10)) {
        return std::string(bytes.data(), static_cast<std::size_t>(textLength));
    }

    std::ostringstream out;
    out << "<data:" << read << " bytes hex=";
    const int preview = std::min(read, 32);
    for (int i = 0; i < preview; ++i) {
        out << hexByte(static_cast<unsigned char>(bytes[static_cast<std::size_t>(i)]));
    }
    if (read > preview) {
        out << "...";
    }
    out << ">";
    return out.str();
}

std::string unitHint(const std::string &path)
{
    if (path.find("latitude") != std::string::npos || path.find("longitude") != std::string::npos ||
        path.find("lat_ref") != std::string::npos || path.find("lon_ref") != std::string::npos) {
        return "deg";
    }
    if (path.find("local_x") != std::string::npos || path.find("local_y") != std::string::npos ||
        path.find("local_z") != std::string::npos || path.find("elevation") != std::string::npos ||
        path.find("y_agl") != std::string::npos || path.find("_alt_levels_m") != std::string::npos) {
        return "m";
    }
    if (path.find("local_v") != std::string::npos || path.find("groundspeed") != std::string::npos ||
        path.find("true_airspeed") != std::string::npos || path.find("wind_speed_msc") != std::string::npos) {
        return "m/s";
    }
    if (path.find("indicated_airspeed") != std::string::npos || path.find("airspeed_kts") != std::string::npos ||
        path.find("true_airspeed_kts") != std::string::npos || path.find("wind_speed_kt") != std::string::npos) {
        return "kt";
    }
    if (path.find("local_a") != std::string::npos) {
        return "m/s^2";
    }
    if (path.find("g_axil") != std::string::npos || path.find("g_side") != std::string::npos ||
        path.find("g_nrml") != std::string::npos || path.find("gforce") != std::string::npos) {
        return "g";
    }
    if (path.find("Prad") != std::string::npos || path.find("Qrad") != std::string::npos ||
        path.find("Rrad") != std::string::npos) {
        return "rad/s";
    }
    if (path.find("/position/P") != std::string::npos || path.find("/position/Q") != std::string::npos ||
        path.find("/position/R") != std::string::npos || path.find("_deg") != std::string::npos ||
        path.find("psi") != std::string::npos || path.find("theta") != std::string::npos ||
        path.find("phi") != std::string::npos || path.find("hpath") != std::string::npos ||
        path.find("vpath") != std::string::npos || path.find("heading") != std::string::npos ||
        path.find("magnetic_variation") != std::string::npos || path.find("alpha") != std::string::npos ||
        path.find("beta") != std::string::npos) {
        return "deg";
    }
    if (path.find("barometer") != std::string::npos && path.find("_pas") != std::string::npos) {
        return "Pa";
    }
    if (path.find("barometer") != std::string::npos && path.find("_inhg") != std::string::npos) {
        return "inHg";
    }
    if (path.find("temperature") != std::string::npos || path.find("temp_degc") != std::string::npos) {
        return "degC";
    }
    if (path.find("_ratio") != std::string::npos || path.find("throttle_used") != std::string::npos) {
        return "ratio";
    }
    if (path.find("_rad_sec") != std::string::npos) {
        return "rad/s";
    }
    if (path.find("_rpm") != std::string::npos || path.find("ENGN_N1_") != std::string::npos ||
        path.find("ENGN_N2_") != std::string::npos) {
        return "rpm_or_percent";
    }
    if (path.find("_TRQ") != std::string::npos) {
        return "N*m";
    }
    if (path.find("q") == path.size() - 1 || path.find("/q") != std::string::npos) {
        return "quat";
    }
    return {};
}

std::int64_t steadyNowNs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

std::string utcStampForFolder()
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &time);
#else
    gmtime_r(&time, &tm);
#endif
    std::ostringstream out;
    out << std::put_time(&tm, "%Y%m%d-%H%M%SZ");
    return out.str();
}

void debug(const std::string &message)
{
    XPLMDebugString(("XPlaneTruthCapture: " + message + "\n").c_str());
}

fs::path systemPath()
{
    char path[1024] = {};
    XPLMGetSystemPath(path);
    return fs::path(path);
}

fs::path detectPluginRoot()
{
    char name[256] = {};
    char path[1024] = {};
    char signature[256] = {};
    char description[512] = {};
    XPLMGetPluginInfo(XPLMGetMyID(), name, path, signature, description);
    fs::path binary(path);
    if (!binary.empty() && binary.has_parent_path()) {
        const auto parent = binary.parent_path();
        if (parent.filename() == "64" && parent.has_parent_path()) {
            return parent.parent_path();
        }
        return parent;
    }
    return systemPath() / "Resources" / "plugins" / XTC_PLUGIN_NAME;
}

void loadConfig()
{
    gConfig = Config{};
    const auto configPath = gPluginRoot / "config" / "capture_config.ini";
    std::ifstream file(configPath);
    if (!file) {
        debug("capture_config.ini not found; using defaults");
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            continue;
        }
        const auto key = lower(trim(line.substr(0, equals)));
        const auto value = trim(line.substr(equals + 1));
        if (key == "capture_rate") {
            gConfig.capture_rate = lower(value);
        } else if (key == "max_array_values") {
            try {
                gConfig.max_array_values = std::max(1, std::stoi(value));
            } catch (...) {
                debug("Invalid max_array_values; keeping default");
            }
        } else if (key == "include_default_datarefs") {
            gConfig.include_default_datarefs = parseBool(value, true);
        }
    }
}

void addDataRefSpec(std::vector<DataRefEntry> &entries, const std::string &path, const std::string &group, bool required)
{
    if (path.empty()) {
        return;
    }
    const auto existing = std::find_if(entries.begin(), entries.end(), [&](const DataRefEntry &entry) {
        return entry.path == path;
    });
    if (existing != entries.end()) {
        return;
    }
    DataRefEntry entry;
    entry.path = path;
    entry.group = group;
    entry.required = required;
    entries.push_back(std::move(entry));
}

bool loadDataRefSpecFile(const fs::path &path, const std::string &defaultGroup, bool defaultRequired, std::vector<DataRefEntry> &entries)
{
    std::ifstream file(path);
    if (!file) {
        return false;
    }

    bool loadedAny = false;
    std::string line;
    while (std::getline(file, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        std::vector<std::string> fields;
        std::stringstream stream(line);
        std::string field;
        while (std::getline(stream, field, '|')) {
            fields.push_back(trim(field));
        }

        const std::string refPath = fields.empty() ? std::string{} : fields[0];
        const std::string group = fields.size() > 1 && !fields[1].empty() ? fields[1] : defaultGroup;
        const bool required = fields.size() > 2 ? parseBool(fields[2], defaultRequired) : defaultRequired;
        addDataRefSpec(entries, refPath, group, required);
        loadedAny = true;
    }
    return loadedAny;
}

void buildDataRefList()
{
    std::vector<DataRefEntry> entries;
    if (gConfig.include_default_datarefs) {
        const auto defaultPath = gPluginRoot / "config" / "default_datarefs.txt";
        if (!loadDataRefSpecFile(defaultPath, "default", false, entries)) {
            debug("default_datarefs.txt not found; using compiled fallback datarefs");
            for (const auto &spec : kDefaultDataRefs) {
                addDataRefSpec(entries, spec.path, spec.group, spec.required);
            }
        }
    }

    const auto customPath = gPluginRoot / "config" / "datarefs.txt";
    loadDataRefSpecFile(customPath, "custom", false, entries);

    gDataRefs = std::move(entries);
}

int arrayLength(XPLMDataRef ref, int typeMask)
{
    if (!ref) {
        return 0;
    }
    if ((typeMask & xplmType_FloatArray) != 0) {
        return XPLMGetDatavf(ref, nullptr, 0, 0);
    }
    if ((typeMask & xplmType_IntArray) != 0) {
        return XPLMGetDatavi(ref, nullptr, 0, 0);
    }
    if ((typeMask & xplmType_Data) != 0) {
        return XPLMGetDatab(ref, nullptr, 0, 0);
    }
    return 0;
}

void resolveDataRefs()
{
    for (auto &entry : gDataRefs) {
        entry.ref = XPLMFindDataRef(entry.path.c_str());
        entry.type_mask = entry.ref ? XPLMGetDataRefTypes(entry.ref) : 0;
        entry.writable = entry.ref ? XPLMCanWriteDataRef(entry.ref) : 0;
        entry.array_len = arrayLength(entry.ref, entry.type_mask);
    }
}

std::string typeMaskString(int typeMask)
{
    if (typeMask == 0) {
        return "missing";
    }
    std::vector<std::string> types;
    if ((typeMask & xplmType_Int) != 0) types.push_back("int");
    if ((typeMask & xplmType_Float) != 0) types.push_back("float");
    if ((typeMask & xplmType_Double) != 0) types.push_back("double");
    if ((typeMask & xplmType_FloatArray) != 0) types.push_back("float_array");
    if ((typeMask & xplmType_IntArray) != 0) types.push_back("int_array");
    if ((typeMask & xplmType_Data) != 0) types.push_back("data");

    std::ostringstream out;
    for (std::size_t i = 0; i < types.size(); ++i) {
        if (i != 0) {
            out << "|";
        }
        out << types[i];
    }
    return out.str();
}

std::string readDataRefValue(const DataRefEntry &entry)
{
    if (!entry.ref) {
        return {};
    }

    const int typeMask = entry.type_mask;
    std::ostringstream out;
    out << std::setprecision(10);

    if ((typeMask & xplmType_FloatArray) != 0) {
        const int count = std::min(entry.array_len, gConfig.max_array_values);
        std::vector<float> values(static_cast<std::size_t>(std::max(0, count)));
        const int read = count > 0 ? XPLMGetDatavf(entry.ref, values.data(), 0, count) : 0;
        for (int i = 0; i < read; ++i) {
            if (i != 0) out << ';';
            out << values[static_cast<std::size_t>(i)];
        }
        return out.str();
    }

    if ((typeMask & xplmType_IntArray) != 0) {
        const int count = std::min(entry.array_len, gConfig.max_array_values);
        std::vector<int> values(static_cast<std::size_t>(std::max(0, count)));
        const int read = count > 0 ? XPLMGetDatavi(entry.ref, values.data(), 0, count) : 0;
        for (int i = 0; i < read; ++i) {
            if (i != 0) out << ';';
            out << values[static_cast<std::size_t>(i)];
        }
        return out.str();
    }

    if ((typeMask & xplmType_Double) != 0) {
        out << XPLMGetDatad(entry.ref);
        return out.str();
    }

    if ((typeMask & xplmType_Float) != 0) {
        out << XPLMGetDataf(entry.ref);
        return out.str();
    }

    if ((typeMask & xplmType_Int) != 0) {
        out << XPLMGetDatai(entry.ref);
        return out.str();
    }

    if ((typeMask & xplmType_Data) != 0) {
        const int count = std::min(entry.array_len, std::max(1, gConfig.max_array_values * 64));
        std::vector<char> values(static_cast<std::size_t>(std::max(0, count)));
        const int read = count > 0 ? XPLMGetDatab(entry.ref, values.data(), 0, count) : 0;
        return dataBytesToText(values, read);
    }

    return {};
}

float captureInterval()
{
    const auto rate = lower(gConfig.capture_rate);
    if (rate == "30hz" || rate == "30") {
        return 1.0f / 30.0f;
    }
    if (rate == "10hz" || rate == "10") {
        return 1.0f / 10.0f;
    }
    return -1.0f;
}

void writeEvent(const std::string &type, const std::string &message)
{
    std::lock_guard<std::mutex> lock(gEventMutex);
    if (!gEventsFile) {
        return;
    }
    gEventsFile << "{\"host_ns\":" << steadyNowNs()
                << ",\"frame_id\":" << gFrameId.load()
                << ",\"type\":\"" << jsonEscape(type)
                << "\",\"message\":\"" << jsonEscape(message) << "\"}\n";
    gEventsFile.flush();
}

std::string sampleValue(const FrameSample &sample, const std::string &path)
{
    for (std::size_t i = 0; i < gDataRefs.size() && i < sample.values.size(); ++i) {
        if (gDataRefs[i].path == path) {
            return sample.values[i];
        }
    }
    return {};
}

double sampleDouble(const FrameSample &sample, const std::string &path)
{
    return parseDouble(sampleValue(sample, path));
}

bool sampleTruthy(const FrameSample &sample, const std::string &path)
{
    const double value = sampleDouble(sample, path);
    return std::isfinite(value) && value > 0.5;
}

void writeJsonNumber(std::ostream &out, double value)
{
    if (std::isfinite(value)) {
        out << std::setprecision(10) << value;
    } else {
        out << "null";
    }
}

void writeSeriesSummary(std::ostream &out, const std::string &name, const NumericSeries &series, bool trailingComma)
{
    out << "    \"" << name << "\": {\n";
    out << "      \"count\": " << series.values.size() << ",\n";
    out << "      \"min\": "; writeJsonNumber(out, series.percentile(0.0)); out << ",\n";
    out << "      \"p50\": "; writeJsonNumber(out, series.percentile(0.5)); out << ",\n";
    out << "      \"p95\": "; writeJsonNumber(out, series.percentile(0.95)); out << ",\n";
    out << "      \"p99\": "; writeJsonNumber(out, series.percentile(0.99)); out << ",\n";
    out << "      \"max\": "; writeJsonNumber(out, series.percentile(1.0)); out << ",\n";
    out << "      \"mean\": "; writeJsonNumber(out, series.mean()); out << "\n";
    out << "    }" << (trailingComma ? "," : "") << "\n";
}

void maybeAutoEvent(bool condition, int &counter, bool &latched, const std::string &label)
{
    if (latched) {
        return;
    }
    if (condition) {
        ++counter;
    } else {
        counter = 0;
    }
    if (counter >= 5) {
        latched = true;
        ++gRunStats.auto_markers;
        writeEvent("auto_phase", label);
    }
}

void observeSample(const FrameSample &sample)
{
    if (sample.first_frame) {
        gRunStats.host_start_ns = sample.host_ns;
    } else {
        gRunStats.elapsed_call_s.observe(sample.elapsed_since_last_call);
        gRunStats.elapsed_flight_loop_s.observe(sample.elapsed_since_last_flight_loop);
    }

    const double simTime = sampleDouble(sample, "sim/time/total_flight_time_sec");
    if (std::isfinite(simTime)) {
        if (!std::isfinite(gRunStats.first_flight_time_s)) {
            gRunStats.first_flight_time_s = simTime;
        }
        if (std::isfinite(gRunStats.previous_flight_time_s)) {
            if (simTime >= gRunStats.previous_flight_time_s) {
                gRunStats.monotonic_flight_duration_s += simTime - gRunStats.previous_flight_time_s;
            } else {
                ++gRunStats.sim_time_resets;
                writeEvent("warning", "Simulation time reset detected; summary duration is segmented");
            }
        }
        gRunStats.last_flight_time_s = simTime;
        gRunStats.previous_flight_time_s = simTime;
    }

    gRunStats.frame_period_s.observe(sampleDouble(sample, "sim/operation/misc/frame_rate_period"));

    if (sampleTruthy(sample, "sim/time/paused")) {
        ++gRunStats.paused_rows;
    }
    const double simSpeed = sampleDouble(sample, "sim/time/sim_speed");
    if (std::isfinite(simSpeed) && std::fabs(simSpeed - 1.0) > 0.01) {
        ++gRunStats.sim_speed_not_1_rows;
    }

    const double gs = sampleDouble(sample, "sim/flightmodel/position/groundspeed");
    const double agl = sampleDouble(sample, "sim/flightmodel/position/y_agl");
    const double vy = sampleDouble(sample, "sim/flightmodel/position/local_vy");
    const bool onGroundAny = sampleTruthy(sample, "sim/flightmodel/failures/onground_any");
    const bool gearOnGround = sampleTruthy(sample, "sim/flightmodel2/gear/on_ground");
    const bool onGround = onGroundAny || gearOnGround || (std::isfinite(agl) && agl < 0.75);
    const bool moving = std::isfinite(gs) && gs > 2.0;
    const bool fastOnGround = !gAutoEvents.airborne && onGround && std::isfinite(gs) && gs > 5.0;
    const bool airborne = !onGround && std::isfinite(agl) && agl > 3.0;
    const bool hover = airborne && std::isfinite(gs) && gs < 3.0 && (!std::isfinite(vy) || std::fabs(vy) < 1.5);
    const bool landing = gAutoEvents.airborne && onGround;
    const bool stopped = gAutoEvents.landing && onGround && std::isfinite(gs) && gs < 0.75;

    maybeAutoEvent(onGround && !moving, gAutoEvents.stationary_count, gAutoEvents.stationary, "stationary_or_taxi_idle");
    maybeAutoEvent(fastOnGround, gAutoEvents.takeoff_roll_count, gAutoEvents.takeoff_roll, "takeoff_or_fast_ground_roll");
    maybeAutoEvent(airborne, gAutoEvents.airborne_count, gAutoEvents.airborne, "airborne");
    maybeAutoEvent(hover, gAutoEvents.hover_count, gAutoEvents.hover_candidate, "hover_candidate");
    maybeAutoEvent(landing, gAutoEvents.landing_count, gAutoEvents.landing, "landing_or_ground_contact");
    maybeAutoEvent(stopped, gAutoEvents.stopped_count, gAutoEvents.stopped_after_landing, "stopped_after_landing");
}

void writeManifest()
{
    int xplaneVersion = 0;
    int xplmVersion = 0;
    int hostId = 0;
    XPLMGetVersions(&xplaneVersion, &xplmVersion, &hostId);

    char aircraftName[256] = {};
    char aircraftPath[1024] = {};
    XPLMGetNthAircraftModel(0, aircraftName, aircraftPath);

    char pluginName[256] = {};
    char pluginPath[1024] = {};
    char pluginSignature[256] = {};
    char pluginDescription[512] = {};
    XPLMGetPluginInfo(XPLMGetMyID(), pluginName, pluginPath, pluginSignature, pluginDescription);

    std::ofstream manifest(gRunDir / "manifest.json");
    manifest << "{\n";
    manifest << "  \"schema\": 1,\n";
    manifest << "  \"plugin\": \"" << XTC_PLUGIN_NAME << "\",\n";
    manifest << "  \"version\": \"" << XTC_VERSION << "\",\n";
    manifest << "  \"run_id\": \"" << jsonEscape(gRunDir.filename().string()) << "\",\n";
    manifest << "  \"xplane_version\": " << xplaneVersion << ",\n";
    manifest << "  \"xplm_version\": " << xplmVersion << ",\n";
    manifest << "  \"host_id\": " << hostId << ",\n";
    manifest << "  \"system_path\": \"" << jsonEscape(systemPath().string()) << "\",\n";
    manifest << "  \"plugin_path\": \"" << jsonEscape(pluginPath) << "\",\n";
    manifest << "  \"aircraft_name\": \"" << jsonEscape(aircraftName) << "\",\n";
    manifest << "  \"aircraft_path\": \"" << jsonEscape(aircraftPath) << "\",\n";
    manifest << "  \"capture_rate\": \"" << jsonEscape(gConfig.capture_rate) << "\",\n";
    manifest << "  \"max_array_values\": " << gConfig.max_array_values << ",\n";
    manifest << "  \"include_default_datarefs\": " << (gConfig.include_default_datarefs ? "true" : "false") << "\n";
    manifest << "}\n";
}

void writeDataRefTable()
{
    std::ofstream file(gRunDir / "datarefs.csv");
    file << "path,group,required,exists,type_mask,type_names,writable,array_len,unit_hint\n";
    for (const auto &entry : gDataRefs) {
        file << csvEscape(entry.path) << ','
             << csvEscape(entry.group) << ','
             << (entry.required ? "true" : "false") << ','
             << (entry.ref ? "true" : "false") << ','
             << entry.type_mask << ','
             << csvEscape(typeMaskString(entry.type_mask)) << ','
             << entry.writable << ','
             << entry.array_len << ','
             << csvEscape(unitHint(entry.path)) << '\n';
    }
}

void openFrameFile()
{
    gFramesFile.open(gRunDir / "frames.csv", std::ios::out | std::ios::trunc);
    gFramesFile << "frame_id,host_ns,first_frame,elapsed_since_last_call_s,elapsed_since_last_flight_loop_s,loop_counter,xplm_cycle_number";
    for (const auto &entry : gDataRefs) {
        gFramesFile << ',' << csvEscape(entry.path);
    }
    gFramesFile << '\n';
}

void writerLoop()
{
    for (;;) {
        FrameSample sample;
        {
            std::unique_lock<std::mutex> lock(gQueueMutex);
            gQueueCv.wait(lock, [] {
                return gWriterStopping.load() || !gQueue.empty();
            });
            if (gQueue.empty()) {
                if (gWriterStopping.load()) {
                    break;
                }
                continue;
            }
            sample = std::move(gQueue.front());
            gQueue.pop_front();
        }

        if (gFramesFile) {
            gFramesFile << sample.frame_id << ','
                        << sample.host_ns << ','
                        << (sample.first_frame ? "true" : "false") << ','
                        << sample.elapsed_since_last_call << ','
                        << sample.elapsed_since_last_flight_loop << ','
                        << sample.loop_counter << ','
                        << sample.xplm_cycle_number;
            for (const auto &value : sample.values) {
                gFramesFile << ',' << csvEscape(value);
            }
            gFramesFile << '\n';
            ++gWrittenRows;
        }
    }
    if (gFramesFile) {
        gFramesFile.flush();
    }
}

void queueSample(FrameSample &&sample)
{
    {
        std::lock_guard<std::mutex> lock(gQueueMutex);
        if (gQueue.size() > 4096) {
            ++gDroppedRows;
            return;
        }
        gQueue.push_back(std::move(sample));
    }
    gQueueCv.notify_one();
}

void copyXPlaneLog()
{
    const auto source = systemPath() / "Log.txt";
    const auto dest = gRunDir / "Log.txt";
    try {
        if (fs::exists(source)) {
            fs::copy_file(source, dest, fs::copy_options::overwrite_existing);
        }
    } catch (...) {
        writeEvent("warning", "Could not copy X-Plane Log.txt");
    }
}

void copyFileIfExists(const fs::path &source, const fs::path &dest)
{
    try {
        if (fs::exists(source)) {
            fs::create_directories(dest.parent_path());
            fs::copy_file(source, dest, fs::copy_options::overwrite_existing);
        }
    } catch (...) {
        writeEvent("warning", "Could not copy support file: " + source.string());
    }
}

void copyRunSupportFiles()
{
    copyFileIfExists(gPluginRoot / "config" / "capture_config.ini", gRunDir / "config" / "capture_config.ini");
    copyFileIfExists(gPluginRoot / "config" / "default_datarefs.txt", gRunDir / "config" / "default_datarefs.txt");
    copyFileIfExists(gPluginRoot / "config" / "datarefs.txt", gRunDir / "config" / "datarefs.txt");
    copyFileIfExists(gPluginRoot / "tools" / "viewer.html", gRunDir / "viewer.html");
    copyFileIfExists(gPluginRoot / "tools" / "analyze_capture.py", gRunDir / "tools" / "analyze_capture.py");
}

void writeSummary(const std::string &reason)
{
    std::ofstream summary(gRunDir / "summary.json");
    const double hostDurationS = (gRunStats.host_start_ns > 0 && gRunStats.host_stop_ns > gRunStats.host_start_ns)
                                     ? static_cast<double>(gRunStats.host_stop_ns - gRunStats.host_start_ns) / 1e9
                                     : std::numeric_limits<double>::quiet_NaN();
    const double simDurationS = (std::isfinite(gRunStats.first_flight_time_s) && std::isfinite(gRunStats.last_flight_time_s))
                                    ? gRunStats.monotonic_flight_duration_s
                                    : std::numeric_limits<double>::quiet_NaN();
    const double medianFramePeriod = gRunStats.frame_period_s.percentile(0.5);
    const double medianFps = std::isfinite(medianFramePeriod) && medianFramePeriod > 0.0 ? 1.0 / medianFramePeriod : std::numeric_limits<double>::quiet_NaN();
    const double meanCall = gRunStats.elapsed_call_s.mean();
    const double meanCallHz = std::isfinite(meanCall) && meanCall > 0.0 ? 1.0 / meanCall : std::numeric_limits<double>::quiet_NaN();

    summary << "{\n";
    summary << "  \"schema\": 1,\n";
    summary << "  \"stop_reason\": \"" << jsonEscape(reason) << "\",\n";
    summary << "  \"frames_seen\": " << gFrameId.load() << ",\n";
    summary << "  \"rows_written\": " << gWrittenRows.load() << ",\n";
    summary << "  \"rows_dropped\": " << gDroppedRows.load() << ",\n";
    summary << "  \"datarefs_requested\": " << gDataRefs.size() << ",\n";
    summary << "  \"host_duration_s\": "; writeJsonNumber(summary, hostDurationS); summary << ",\n";
    summary << "  \"sim_duration_s\": "; writeJsonNumber(summary, simDurationS); summary << ",\n";
    summary << "  \"sim_first_time_s\": "; writeJsonNumber(summary, gRunStats.first_flight_time_s); summary << ",\n";
    summary << "  \"sim_last_time_s\": "; writeJsonNumber(summary, gRunStats.last_flight_time_s); summary << ",\n";
    summary << "  \"sim_time_resets\": " << gRunStats.sim_time_resets << ",\n";
    summary << "  \"effective_hz_mean_from_callback\": "; writeJsonNumber(summary, meanCallHz); summary << ",\n";
    summary << "  \"effective_hz_median_from_frame_period\": "; writeJsonNumber(summary, medianFps); summary << ",\n";
    summary << "  \"paused_rows\": " << gRunStats.paused_rows << ",\n";
    summary << "  \"sim_speed_not_1_rows\": " << gRunStats.sim_speed_not_1_rows << ",\n";
    summary << "  \"auto_markers\": " << gRunStats.auto_markers << ",\n";
    summary << "  \"timing\": {\n";
    writeSeriesSummary(summary, "elapsed_since_last_call_s", gRunStats.elapsed_call_s, true);
    writeSeriesSummary(summary, "elapsed_since_last_flight_loop_s", gRunStats.elapsed_flight_loop_s, true);
    writeSeriesSummary(summary, "frame_rate_period_s", gRunStats.frame_period_s, false);
    summary << "  }\n";
    summary << "}\n";
}

void startWriter()
{
    gWriterStopping.store(false);
    gWriterThread = std::thread(writerLoop);
}

void stopWriter()
{
    gWriterStopping.store(true);
    gQueueCv.notify_all();
    if (gWriterThread.joinable()) {
        gWriterThread.join();
    }
}

void startCapture()
{
    if (gCapturing.load()) {
        debug("Capture already running");
        return;
    }

    loadConfig();
    buildDataRefList();
    resolveDataRefs();

    gRunDir = systemPath() / "Output" / XTC_PLUGIN_NAME / utcStampForFolder();
    try {
        fs::create_directories(gRunDir);
    } catch (const std::exception &e) {
        debug(std::string("Could not create output directory: ") + e.what());
        return;
    }

    gFrameId.store(0);
    gDroppedRows.store(0);
    gWrittenRows.store(0);
    gRunStats = RunStats{};
    gAutoEvents = AutoEventState{};
    {
        std::lock_guard<std::mutex> lock(gQueueMutex);
        gQueue.clear();
    }

    gEventsFile.open(gRunDir / "events.jsonl", std::ios::out | std::ios::trunc);
    writeManifest();
    writeDataRefTable();
    openFrameFile();
    copyXPlaneLog();
    copyRunSupportFiles();
    startWriter();

    gCapturing.store(true);
    writeEvent("start", "Capture started");
    XPLMScheduleFlightLoop(gFlightLoopId, captureInterval(), 1);
    debug("Capture started at " + gRunDir.string());
}

void stopCapture(const std::string &reason)
{
    if (!gCapturing.load()) {
        debug("Capture is not running");
        return;
    }

    gCapturing.store(false);
    XPLMScheduleFlightLoop(gFlightLoopId, 0.0f, 1);
    writeEvent("stop", reason);
    gRunStats.host_stop_ns = steadyNowNs();
    stopWriter();
    copyXPlaneLog();

    if (gFramesFile) {
        gFramesFile.close();
    }
    if (gEventsFile) {
        gEventsFile.close();
    }
    writeSummary(reason);
    debug("Capture stopped: " + reason);
}

float flightLoopCallback(float elapsedSinceLastCall, float elapsedSinceLastFlightLoop, int counter, void *)
{
    if (!gCapturing.load()) {
        return 0.0f;
    }

    FrameSample sample;
    sample.frame_id = ++gFrameId;
    sample.host_ns = steadyNowNs();
    sample.first_frame = sample.frame_id == 1;
    sample.elapsed_since_last_call = elapsedSinceLastCall;
    sample.elapsed_since_last_flight_loop = elapsedSinceLastFlightLoop;
    sample.loop_counter = counter;
    sample.xplm_cycle_number = XPLMGetCycleNumber();
    sample.values.reserve(gDataRefs.size());

    for (const auto &entry : gDataRefs) {
        sample.values.push_back(readDataRefValue(entry));
    }

    observeSample(sample);
    queueSample(std::move(sample));
    return captureInterval();
}

enum class MenuAction {
    Start = 1,
    Stop = 2,
    Mark = 3,
    EveryFrame = 4,
    Rate30Hz = 5,
    Rate10Hz = 6,
};

void menuHandler(void *, void *itemRef)
{
    const auto action = static_cast<MenuAction>(reinterpret_cast<std::intptr_t>(itemRef));
    switch (action) {
    case MenuAction::Start:
        startCapture();
        break;
    case MenuAction::Stop:
        stopCapture("menu_stop");
        break;
    case MenuAction::Mark:
        writeEvent("marker", "User marker");
        debug("Marker recorded");
        break;
    case MenuAction::EveryFrame:
        gConfig.capture_rate = "every_frame";
        writeEvent("config", "capture_rate=every_frame");
        debug("Capture rate set to every frame for this run");
        break;
    case MenuAction::Rate30Hz:
        gConfig.capture_rate = "30hz";
        writeEvent("config", "capture_rate=30hz");
        debug("Capture rate set to 30 Hz for this run");
        break;
    case MenuAction::Rate10Hz:
        gConfig.capture_rate = "10hz";
        writeEvent("config", "capture_rate=10hz");
        debug("Capture rate set to 10 Hz for this run");
        break;
    }
}

void createMenu()
{
    const int item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), XTC_PLUGIN_NAME, nullptr, 1);
    gMenuId = XPLMCreateMenu(XTC_PLUGIN_NAME, XPLMFindPluginsMenu(), item, menuHandler, nullptr);
    XPLMAppendMenuItem(gMenuId, "Start Capture", reinterpret_cast<void *>(static_cast<std::intptr_t>(MenuAction::Start)), 1);
    XPLMAppendMenuItem(gMenuId, "Stop Capture", reinterpret_cast<void *>(static_cast<std::intptr_t>(MenuAction::Stop)), 1);
    XPLMAppendMenuSeparator(gMenuId);
    XPLMAppendMenuItem(gMenuId, "Mark Event", reinterpret_cast<void *>(static_cast<std::intptr_t>(MenuAction::Mark)), 1);
    XPLMAppendMenuSeparator(gMenuId);
    XPLMAppendMenuItem(gMenuId, "Rate: Every Frame", reinterpret_cast<void *>(static_cast<std::intptr_t>(MenuAction::EveryFrame)), 1);
    XPLMAppendMenuItem(gMenuId, "Rate: 30 Hz", reinterpret_cast<void *>(static_cast<std::intptr_t>(MenuAction::Rate30Hz)), 1);
    XPLMAppendMenuItem(gMenuId, "Rate: 10 Hz", reinterpret_cast<void *>(static_cast<std::intptr_t>(MenuAction::Rate10Hz)), 1);
}

void destroyMenu()
{
    if (gMenuId) {
        XPLMDestroyMenu(gMenuId);
        gMenuId = nullptr;
    }
}

int markCommandHandler(XPLMCommandRef, XPLMCommandPhase phase, void *)
{
    if (phase == xplm_CommandBegin) {
        writeEvent("marker", "Command marker");
        debug("Command marker recorded");
    }
    return 1;
}

void createCommands()
{
    gMarkCommand = XPLMCreateCommand("xplane_truth_capture/mark_event", "Mark the current XPlaneTruthCapture frame");
    if (gMarkCommand) {
        XPLMRegisterCommandHandler(gMarkCommand, markCommandHandler, 1, nullptr);
    }
}

void destroyCommands()
{
    if (gMarkCommand) {
        XPLMUnregisterCommandHandler(gMarkCommand, markCommandHandler, 1, nullptr);
        gMarkCommand = nullptr;
    }
}

void createFlightLoop()
{
    XPLMCreateFlightLoop_t params{};
    params.structSize = sizeof(params);
    params.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    params.callbackFunc = flightLoopCallback;
    params.refcon = nullptr;
    gFlightLoopId = XPLMCreateFlightLoop(&params);
}

void destroyFlightLoop()
{
    if (gFlightLoopId) {
        XPLMDestroyFlightLoop(gFlightLoopId);
        gFlightLoopId = nullptr;
    }
}

} // namespace

PLUGIN_API int XPluginStart(char *outName, char *outSignature, char *outDescription)
{
    std::strcpy(outName, XTC_PLUGIN_NAME);
    std::strcpy(outSignature, XTC_PLUGIN_SIGNATURE);
    std::strcpy(outDescription, XTC_PLUGIN_DESCRIPTION);

    if (XPLMHasFeature("XPLM_USE_NATIVE_PATHS")) {
        XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    }

    gPluginRoot = detectPluginRoot();
    createMenu();
    createCommands();
    createFlightLoop();
    debug(std::string("Started v") + XTC_VERSION + " at " + gPluginRoot.string());
    return 1;
}

PLUGIN_API void XPluginStop()
{
    if (gCapturing.load()) {
        stopCapture("plugin_stop");
    }
    destroyFlightLoop();
    destroyCommands();
    destroyMenu();
    debug("Stopped");
}

PLUGIN_API int XPluginEnable()
{
    debug("Enabled");
    return 1;
}

PLUGIN_API void XPluginDisable()
{
    if (gCapturing.load()) {
        stopCapture("plugin_disable");
    }
    debug("Disabled");
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int inMessage, void *)
{
    if (inMessage == XPLM_MSG_PLANE_LOADED && gCapturing.load()) {
        writeEvent("aircraft_reload", "Aircraft loaded; refreshing datarefs");
        resolveDataRefs();
        writeDataRefTable();
    }
}
