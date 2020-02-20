#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/RateThrust.h>
#include <msp_fc_driver/RcData.h>
#include "msp.hpp"

#include <algorithm>
#include <cmath>

class MspInterface {
    MSP msp;
    std::vector<uint16_t> rcData;
    double max_roll_r  = 200;
    double max_pitch_r = 200;
    double max_yaw_r   = 200;
    double hover_thrust = 0.3;
    double mass = 1.0;
    ros::Publisher pub_armed;
    ros::Publisher pub_offboard;
    ros::Publisher pub_rc;

    Payload serialize_rc_data() {
        Payload result;
        for (int i = 0; i < rcData.size(); i++) {
            result.put_u16(rcData[i]);
        }

        return result;
    }

public:
    MspInterface(ros::NodeHandle &n, const std::string &path)
        : msp(path), rcData(5, 1500)
    {
        // Set thurst to 0
        rcData[2] = 1000;

        pub_armed = n.advertise<std_msgs::Bool>("/uav/state/armed", 1, true);
        pub_offboard = n.advertise<std_msgs::Bool>("/uav/state/offboard", 1, true);
        pub_rc = n.advertise<msp_fc_driver::RcData>("/uav/state/rc", 1, true);

        msp.register_callback(MSP::RC, [this](Payload payload) {
            std::vector<uint16_t> droneRcData(payload.size() / 2);
            for (int i = 0; i < droneRcData.size(); i++) {
                droneRcData[i] = payload.get_u16();
            }

            std_msgs::Bool is_offboard;
            is_offboard.data = droneRcData[5] > 1800;
            pub_offboard.publish(is_offboard);

            std_msgs::Bool is_armed;
            is_armed.data = droneRcData[4] > 1800;
            pub_armed.publish(is_armed);

            msp_fc_driver::RcData rc_msg;
            for (int i = 0; i < std::min(6, (int) droneRcData.size()); i++)
                rc_msg.channels[i] = droneRcData[i];
            pub_rc.publish(rc_msg);
        });

        // Get rateprofile params
        n.param<double>("/rc_rates/roll",  max_roll_r, 1.0);
        n.param<double>("/rc_rates/pitch", max_pitch_r, 1.0);
        n.param<double>("/rc_rates/yaw",   max_yaw_r, 1.0);
        n.param<double>("/rc_rates/hover_thrust", hover_thrust, 1.0);
        n.param<double>("/uav/mass", mass, 1.0);
        max_roll_r  *= 200;
        max_pitch_r *= 200;
        max_yaw_r   *= 200;
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
        double roll_r    = rates.angular_rates.x * 180 / M_PI / max_roll_r;
        double pitch_r   = rates.angular_rates.y * 180 / M_PI / max_pitch_r;
        double yaw_r     = rates.angular_rates.z * 180 / M_PI / max_yaw_r;
        rcData[0] = (uint16_t) std::min(500,  std::max(-500, (int) round(roll_r  * 500))) + 1500;
        rcData[1] = (uint16_t) std::min(500,  std::max(-500, (int) round(pitch_r * 500))) + 1500;
        rcData[3] = (uint16_t) std::min(500,  std::max(-500, (int) round(yaw_r   * 500))) + 1500;
        
        double thrust = rates.thrust.z / 9.81 / mass * hover_thrust;
        rcData[2] = (uint16_t) std::min(1000, std::max(0, (int) round(thrust * 1000))) + 1000;
    }

    void step() {
        // Send rc data
        msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

        // Request rc data
        msp.send_msg(MSP::RC, {});

        // Recive new msp messages
        msp.recv_msgs();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "msp_fc_interface");
    ros::NodeHandle n;
    ros::Rate rate(20);

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
