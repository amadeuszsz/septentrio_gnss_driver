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
    class TickRelay
    {
    public:
        TickRelay(ROSaicNodeBase* node) : node_(node), settings_(node->settings())
        {
            if (!settings_->publish_diagnostics)
                return;

            static constexpr double nominal_freq = 5.0;
            diagnostics_updater_ =
                std::make_unique<diagnostic_updater::Updater>(node);
            diagnostics_updater_->setPeriod(1.0 / nominal_freq);
            diagnostics_updater_->setHardwareID(settings_->frame_id);

            auto ok_min_freq = settings_->diagnostics_ok_min * nominal_freq;
            auto ok_max_freq = settings_->diagnostics_ok_max * nominal_freq;
            auto warn_min_freq = settings_->diagnostics_warn_min * nominal_freq;
            auto warn_max_freq = settings_->diagnostics_warn_max * nominal_freq;
            auto ok_params = RateBoundStatusParam{ok_min_freq, ok_max_freq};
            auto warn_params = RateBoundStatusParam{warn_min_freq, warn_max_freq};

            if (settings_->publish_gpst && settings_->monitor_gpst)
                update_map("gpst", ok_params, warn_params);
            if (settings_->publish_navsatfix && settings_->monitor_navsatfix)
                update_map("navsatfix", ok_params, warn_params);
            if (settings_->publish_gpsfix && settings_->monitor_gpsfix)
                update_map("gpsfix", ok_params, warn_params);
            if (settings_->publish_pose && settings_->monitor_pose)
                update_map("pose", ok_params, warn_params);
            if (settings_->publish_aimplusstatus && settings_->monitor_aimplusstatus)
                update_map("aimplusstatus", ok_params, warn_params);
            if (settings_->publish_galauthstatus && settings_->monitor_galauthstatus)
                update_map("galauthstatus", ok_params, warn_params);
            if (settings_->publish_gpgga && settings_->monitor_gpgga)
                update_map("gpgga", ok_params, warn_params);
            if (settings_->publish_gprmc && settings_->monitor_gprmc)
                update_map("gprmc", ok_params, warn_params);
            if (settings_->publish_gpgsa && settings_->monitor_gpgsa)
                update_map("gpgsa", ok_params, warn_params);
            if (settings_->publish_gpgsv && settings_->monitor_gpgsv)
                update_map("gpgsv", ok_params, warn_params);
            if (settings_->publish_measepoch && settings_->monitor_measepoch)
                update_map("measepoch", ok_params, warn_params);
            if (settings_->publish_pvtcartesian && settings_->monitor_pvtcartesian)
                update_map("pvtcartesian", ok_params, warn_params);
            if (settings_->publish_pvtgeodetic && settings_->monitor_pvtgeodetic)
                update_map("pvtgeodetic", ok_params, warn_params);
            if (settings_->publish_basevectorcart &&
                settings_->monitor_basevectorcart)
                update_map("basevectorcart", ok_params, warn_params);
            if (settings_->publish_basevectorgeod &&
                settings_->monitor_basevectorgeod)
                update_map("basevectorgeod", ok_params, warn_params);
            if (settings_->publish_poscovcartesian &&
                settings_->monitor_poscovcartesian)
                update_map("poscovcartesian", ok_params, warn_params);
            if (settings_->publish_poscovgeodetic &&
                settings_->monitor_poscovgeodetic)
                update_map("poscovgeodetic", ok_params, warn_params);
            if (settings_->publish_velcovcartesian &&
                settings_->monitor_velcovcartesian)
                update_map("velcovcartesian", ok_params, warn_params);
            if (settings_->publish_velcovgeodetic &&
                settings_->monitor_velcovgeodetic)
                update_map("velcovgeodetic", ok_params, warn_params);
            if (settings_->publish_atteuler && settings_->monitor_atteuler)
                update_map("atteuler", ok_params, warn_params);
            if (settings_->publish_attcoveuler && settings_->monitor_attcoveuler)
                update_map("attcoveuler", ok_params, warn_params);
            if (settings_->publish_insnavcart && settings_->monitor_insnavcart)
                update_map("insnavcart", ok_params, warn_params);
            if (settings_->publish_insnavgeod && settings_->monitor_insnavgeod)
                update_map("insnavgeod", ok_params, warn_params);
            if (settings_->publish_imusetup && settings_->monitor_imusetup)
                update_map("imusetup", ok_params, warn_params);
            if (settings_->publish_velsensorsetup &&
                settings_->monitor_velsensorsetup)
                update_map("velsensorsetup", ok_params, warn_params);
            if (settings_->publish_exteventinsnavgeod &&
                settings_->monitor_exteventinsnavgeod)
                update_map("exteventinsnavgeod", ok_params, warn_params);
            if (settings_->publish_exteventinsnavcart &&
                settings_->monitor_exteventinsnavcart)
                update_map("exteventinsnavcart", ok_params, warn_params);
            if (settings_->publish_extsensormeas && settings_->monitor_extsensormeas)
                update_map("extsensormeas", ok_params, warn_params);
            if (settings_->publish_imu && settings_->monitor_imu)
                update_map("imu", ok_params, warn_params);
            if (settings_->publish_localization && settings_->monitor_localization)
                update_map("localization", ok_params, warn_params);
            if (settings_->publish_localization_ecef &&
                settings_->monitor_localization_ecef)
                update_map("localization_ecef", ok_params, warn_params);
            if (settings_->publish_twist && settings_->monitor_twist)
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