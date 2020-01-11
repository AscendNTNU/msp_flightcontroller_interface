#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/RateThrust.h>
#include "msp.hpp"

#include <algorithm>
#include <cmath>

class MspInterface {
    MSP msp;
    std::vector<uint16_t> rcData;
    double max_roll_r  = 200;
    double max_pitch_r = 200;
    double max_yaw_r   = 200;
    ros::Publisher pub_armed;
    ros::Publisher pub_offboard;

    std::vector<unsigned char> serialize_rc_data() {
        std::vector<unsigned char> result(rcData.size()*2);
        for (int i = 0; i < rcData.size(); i++) {
            result[i*2] = rcData[i] & 0xff;
            result[i*2+1] = (rcData[i] >> 8) & 0xff;
        }

        return result;
    }

public:
    MspInterface(ros::NodeHandle &n, const std::string &path)
        : msp(path), rcData(5, 1500)
    {
        // Set thurst to 0
        rcData[3] = 1000;

        pub_armed = n.advertise<std_msgs::Bool>("/uav/state/armed", 1, true);
        pub_offboard = n.advertise<std_msgs::Bool>("/uav/state/offboard", 1, true);

        msp.register_callback(MSP::RC, [this](std::vector<unsigned char> &payload) {
            std::vector<uint16_t> droneRcData(payload.size() / 2);
            for (int i = 0; i < droneRcData.size(); i++) {
                droneRcData[i] = payload[i*2] | (payload[i*2 + 1] << 8);
            }

            std_msgs::Bool is_offboard;
            is_offboard.data = droneRcData[6] > 1800;
            pub_offboard.publish(is_offboard);

            std_msgs::Bool is_armed;
            is_armed.data = droneRcData[5] > 1800;
            pub_armed.publish(is_armed);
        });

        // Get rateprofile params
        n.param<double>("rc_rates/roll",  max_roll_r, 1.0);
        n.param<double>("rc_rates/pitch", max_pitch_r, 1.0);
        n.param<double>("rc_rates/yaw",   max_yaw_r, 1.0);
        max_roll_r  *= 200;
        max_pitch_r *= 200;
        max_yaw_r   *= 200;

        // TODO: get imu data?
    }

    void set_armed(std_msgs::Bool armed) {
        if (armed.data) {
            rcData[4] = 2000;
        }
        else {
            rcData[4] = 1000;
        }
    }

    void set_rates(mav_msgs::RateThrust rates) {
        double roll_r    = rates.angular_rates.x * 360 / M_PI / max_roll_r;
        double pitch_r   = rates.angular_rates.y * 360 / M_PI / max_pitch_r;
        double yaw_r     = rates.angular_rates.z * 360 / M_PI / max_yaw_r;
        rcData[0] = (uint16_t) std::min(500,  std::max(-500, (int) round(roll_r  * 500))) + 1500;
        rcData[1] = (uint16_t) std::min(500,  std::max(-500, (int) round(pitch_r * 500))) + 1500;
        rcData[2] = (uint16_t) std::min(500,  std::max(-500, (int) round(yaw_r   * 500))) + 1500;
        rcData[3] = (uint16_t) std::min(1000, std::max(0, (int) round(rates.thrust.z * 1000))) + 1000;
    }

    void step() {
        // Send rc data
        msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

        // Request rc data
        msp.send_msg(MSP::RC, {});

        // Request imu data
        msp.send_msg(MSP::RAW_IMU, {});

        // Recive new msp messages
        msp.recv_msgs();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "msp_fc_interface");
    ros::NodeHandle n;
    ros::Rate rate(10);

    MspInterface iface(n, "/dev/ttyACM0");

    ros::Subscriber sub_arm   = n.subscribe("/uav/control/arm", 1, &MspInterface::set_armed, &iface);
    ros::Subscriber sub_rates = n.subscribe("/uav/control/rate_thrust", 1, &MspInterface::set_rates, &iface);

    while (ros::ok()) {
        iface.step();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
