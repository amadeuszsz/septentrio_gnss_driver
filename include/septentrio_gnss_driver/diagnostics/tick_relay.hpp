// *****************************************************************************
//
// © Copyright 2025, TIER IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#pragma once

#include "septentrio_gnss_driver/abstraction/typedefs.hpp"
#include "septentrio_gnss_driver/communication/settings.hpp"
#include "septentrio_gnss_driver/diagnostics/rate_bound_status.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <string>
#include <unordered_map>

namespace custom_diagnostic_tasks {
    struct DiagnosticSettings
    {
        std::string hardware_id;
        double ok_min_freq;
        double ok_max_freq;
        double warn_min_freq;
        double warn_max_freq;
        bool monitor_gpst;
        bool monitor_navsatfix;
        bool monitor_gpsfix;
        bool monitor_pose;
        bool monitor_aimplusstatus;
        bool monitor_galauthstatus;
        bool monitor_gpgga;
        bool monitor_gprmc;
        bool monitor_gpgsa;
        bool monitor_gpgsv;
        bool monitor_measepoch;
        bool monitor_pvtcartesian;
        bool monitor_pvtgeodetic;
        bool monitor_basevectorcart;
        bool monitor_basevectorgeod;
        bool monitor_poscovcartesian;
        bool monitor_poscovgeodetic;
        bool monitor_velcovgeodetic;
        bool monitor_atteuler;
        bool monitor_attcoveuler;
        bool monitor_insnavcart;
        bool monitor_insnavgeod;
        bool monitor_imusetup;
        bool monitor_velsensorsetup;
        bool monitor_exteventinsnavgeod;
        bool monitor_exteventinsnavcart;
        bool monitor_extsensormeas;
        bool monitor_imu;
        bool monitor_localization;
        bool monitor_localization_ecef;
        bool monitor_twist;
    };

    class TickRelay
    {
    public:
        TickRelay(ROSaicNodeBase* node) : node_(node), settings_(node->settings())
        {
            DiagnosticSettings diagnostic_settings{};
            diagnostic_settings.hardware_id = node_->declare_parameter<std::string>(
                "diagnostics.hardware_id", "gnss");
            diagnostic_settings.ok_min_freq = node_->declare_parameter<double>(
                "diagnostics.rate_bound_status.relative_frequency_ok.min", 0.95);
            diagnostic_settings.ok_max_freq = node_->declare_parameter<double>(
                "diagnostics.rate_bound_status.relative_frequency_ok.max", 1.05);
            diagnostic_settings.warn_min_freq = node_->declare_parameter<double>(
                "diagnostics.rate_bound_status.relative_frequency_warn.min", 0.9);
            diagnostic_settings.warn_max_freq = node_->declare_parameter<double>(
                "diagnostics.rate_bound_status.relative_frequency_warn.max", 1.1);
            diagnostic_settings.monitor_gpst =
                node_->declare_parameter<bool>("diagnostics.monitor_gpst", false);
            diagnostic_settings.monitor_navsatfix = node_->declare_parameter<bool>(
                "diagnostics.monitor_navsatfix", false);
            diagnostic_settings.monitor_gpsfix =
                node_->declare_parameter<bool>("diagnostics.monitor_gpsfix", false);
            diagnostic_settings.monitor_pose =
                node_->declare_parameter<bool>("diagnostics.monitor_pose", false);
            diagnostic_settings.monitor_aimplusstatus =
                node_->declare_parameter<bool>("diagnostics.monitor_aimplusstatus",
                                               false);
            diagnostic_settings.monitor_galauthstatus =
                node_->declare_parameter<bool>("diagnostics.monitor_galauthstatus",
                                               false);
            diagnostic_settings.monitor_gpgga =
                node_->declare_parameter<bool>("diagnostics.monitor_gpgga", false);
            diagnostic_settings.monitor_gprmc =
                node_->declare_parameter<bool>("diagnostics.monitor_gprmc", false);
            diagnostic_settings.monitor_gpgsa =
                node_->declare_parameter<bool>("diagnostics.monitor_gpgsa", false);
            diagnostic_settings.monitor_gpgsv =
                node_->declare_parameter<bool>("diagnostics.monitor_gpgsv", false);
            diagnostic_settings.monitor_measepoch = node_->declare_parameter<bool>(
                "diagnostics.monitor_measepoch", false);
            diagnostic_settings.monitor_pvtcartesian =
                node_->declare_parameter<bool>("diagnostics.monitor_pvtcartesian",
                                               false);
            diagnostic_settings.monitor_pvtgeodetic = node_->declare_parameter<bool>(
                "diagnostics.monitor_pvtgeodetic", false);
            diagnostic_settings.monitor_basevectorcart =
                node_->declare_parameter<bool>("diagnostics.monitor_basevectorcart",
                                               false);
            diagnostic_settings.monitor_basevectorgeod =
                node_->declare_parameter<bool>("diagnostics.monitor_basevectorgeod",
                                               false);
            diagnostic_settings.monitor_poscovcartesian =
                node_->declare_parameter<bool>("diagnostics.monitor_poscovcartesian",
                                               false);
            diagnostic_settings.monitor_poscovgeodetic =
                node_->declare_parameter<bool>("diagnostics.monitor_poscovgeodetic",
                                               false);
            diagnostic_settings.monitor_velcovgeodetic =
                node_->declare_parameter<bool>("diagnostics.monitor_velcovgeodetic",
                                               false);
            diagnostic_settings.monitor_atteuler = node_->declare_parameter<bool>(
                "diagnostics.monitor_atteuler", false);
            diagnostic_settings.monitor_attcoveuler = node_->declare_parameter<bool>(
                "diagnostics.monitor_attcoveuler", false);
            diagnostic_settings.monitor_insnavcart = node_->declare_parameter<bool>(
                "diagnostics.monitor_insnavcart", false);
            diagnostic_settings.monitor_insnavgeod = node_->declare_parameter<bool>(
                "diagnostics.monitor_insnavgeod", false);
            diagnostic_settings.monitor_imusetup = node_->declare_parameter<bool>(
                "diagnostics.monitor_imusetup", false);
            diagnostic_settings.monitor_velsensorsetup =
                node_->declare_parameter<bool>("diagnostics.monitor_velsensorsetup",
                                               false);
            diagnostic_settings.monitor_exteventinsnavgeod =
                node_->declare_parameter<bool>(
                    "diagnostics.monitor_exteventinsnavgeod", false);
            diagnostic_settings.monitor_exteventinsnavcart =
                node_->declare_parameter<bool>(
                    "diagnostics.monitor_exteventinsnavcart", false);
            diagnostic_settings.monitor_extsensormeas =
                node_->declare_parameter<bool>("diagnostics.monitor_extsensormeas",
                                               false);
            diagnostic_settings.monitor_imu =
                node_->declare_parameter<bool>("diagnostics.monitor_imu", false);
            diagnostic_settings.monitor_localization =
                node_->declare_parameter<bool>("diagnostics.monitor_localization",
                                               false);
            diagnostic_settings.monitor_localization_ecef =
                node_->declare_parameter<bool>(
                    "diagnostics.monitor_localization_ecef", false);
            diagnostic_settings.monitor_twist =
                node_->declare_parameter<bool>("diagnostics.monitor_twist", false);

            static constexpr double nominal_freq = 5.0;
            diagnostics_updater_ =
                std::make_unique<diagnostic_updater::Updater>(node);
            diagnostics_updater_->setPeriod(1.0 / nominal_freq);
            diagnostics_updater_->setHardwareID(diagnostic_settings.hardware_id);

            auto ok_min_freq = diagnostic_settings.ok_min_freq * nominal_freq;
            auto ok_max_freq = diagnostic_settings.ok_max_freq * nominal_freq;
            auto warn_min_freq = diagnostic_settings.warn_min_freq * nominal_freq;
            auto warn_max_freq = diagnostic_settings.warn_max_freq * nominal_freq;
            auto ok_params = RateBoundStatusParam{ok_min_freq, ok_max_freq};
            auto warn_params = RateBoundStatusParam{warn_min_freq, warn_max_freq};

            if (diagnostic_settings.monitor_gpst)
                update_map("gpst", ok_params, warn_params);
            if (diagnostic_settings.monitor_navsatfix)
                update_map("navsatfix", ok_params, warn_params);
            if (diagnostic_settings.monitor_gpsfix)
                update_map("gpsfix", ok_params, warn_params);
            if (diagnostic_settings.monitor_pose)
                update_map("pose", ok_params, warn_params);
            if (diagnostic_settings.monitor_aimplusstatus)
                update_map("aimplusstatus", ok_params, warn_params);
            if (diagnostic_settings.monitor_galauthstatus)
                update_map("galauthstatus", ok_params, warn_params);
            if (diagnostic_settings.monitor_gpgga)
                update_map("gpgga", ok_params, warn_params);
            if (diagnostic_settings.monitor_gprmc)
                update_map("gprmc", ok_params, warn_params);
            if (diagnostic_settings.monitor_gpgsa)
                update_map("gpgsa", ok_params, warn_params);
            if (diagnostic_settings.monitor_gpgsv)
                update_map("gpgsv", ok_params, warn_params);
            if (diagnostic_settings.monitor_measepoch)
                update_map("measepoch", ok_params, warn_params);
            if (diagnostic_settings.monitor_pvtcartesian)
                update_map("pvtcartesian", ok_params, warn_params);
            if (diagnostic_settings.monitor_pvtgeodetic)
                update_map("pvtgeodetic", ok_params, warn_params);
            if (diagnostic_settings.monitor_basevectorcart)
                update_map("basevectorcart", ok_params, warn_params);
            if (diagnostic_settings.monitor_basevectorgeod)
                update_map("basevectorgeod", ok_params, warn_params);
            if (diagnostic_settings.monitor_poscovcartesian)
                update_map("poscovcartesian", ok_params, warn_params);
            if (diagnostic_settings.monitor_poscovgeodetic)
                update_map("poscovgeodetic", ok_params, warn_params);
            if (diagnostic_settings.monitor_velcovgeodetic)
                update_map("velcovgeodetic", ok_params, warn_params);
            if (diagnostic_settings.monitor_atteuler)
                update_map("atteuler", ok_params, warn_params);
            if (diagnostic_settings.monitor_attcoveuler)
                update_map("attcoveuler", ok_params, warn_params);
            if (diagnostic_settings.monitor_insnavcart)
                update_map("insnavcart", ok_params, warn_params);
            if (diagnostic_settings.monitor_insnavgeod)
                update_map("insnavgeod", ok_params, warn_params);
            if (diagnostic_settings.monitor_imusetup)
                update_map("imusetup", ok_params, warn_params);
            if (diagnostic_settings.monitor_velsensorsetup)
                update_map("velsensorsetup", ok_params, warn_params);
            if (diagnostic_settings.monitor_exteventinsnavgeod)
                update_map("exteventinsnavgeod", ok_params, warn_params);
            if (diagnostic_settings.monitor_exteventinsnavcart)
                update_map("exteventinsnavcart", ok_params, warn_params);
            if (diagnostic_settings.monitor_extsensormeas)
                update_map("extsensormeas", ok_params, warn_params);
            if (diagnostic_settings.monitor_imu)
                update_map("imu", ok_params, warn_params);
            if (diagnostic_settings.monitor_localization)
                update_map("localization", ok_params, warn_params);
            if (diagnostic_settings.monitor_localization_ecef)
                update_map("localization_ecef", ok_params, warn_params);
            if (diagnostic_settings.monitor_twist)
                update_map("twist", ok_params, warn_params);

            diagnostics_updater_->force_update();
        }

        void tick(const std::string& topic)
        {
            auto it = rate_bound_status_map_.find(topic);
            if (it != rate_bound_status_map_.end())
                it->second->tick();
        }

    private:
        void update_map(const std::string& topic,
                        const RateBoundStatusParam& ok_params,
                        const RateBoundStatusParam& warn_params)
        {
            rate_bound_status_map_.emplace(
                topic, std::make_unique<custom_diagnostic_tasks::RateBoundStatus>(
                           node_, ok_params, warn_params, 3, true,
                           topic + " rate bound status"));
            diagnostics_updater_->add(*rate_bound_status_map_.at(topic));
        }

        ROSaicNodeBase* node_;
        const Settings* settings_;
        std::unique_ptr<diagnostic_updater::Updater> diagnostics_updater_;
        std::unordered_map<std::string,
                           std::unique_ptr<custom_diagnostic_tasks::RateBoundStatus>>
            rate_bound_status_map_;
    };

} // namespace custom_diagnostic_tasks