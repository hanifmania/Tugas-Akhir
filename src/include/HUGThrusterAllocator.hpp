
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __HUGTHRUSTERALLOCATOR_CLASS__
#define __HUGTHRUSTERALLOCATOR_CLASS__
#include <algorithm>
#include <map>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <sstream>
#include <cola2_lib/cola2_control/Request.h>
#include <math.h>

#include "ros/ros.h"                                                                                                                            //****
#include "cola2_msgs/ThrustersInfo.h"                                                                                                        //****

bool simulation = true;

class ThrusterAllocatorHUG {
public:
    ThrusterAllocatorHUG( unsigned int n_thrusters ):
        _n_thrusters( n_thrusters ),
        _th_voltages( n_thrusters ),                                                                                                            // ****
        _th_currents( n_thrusters ),                                                                                                            // ****
        _th_rpms( n_thrusters ),                                                                                                                // ****
        _is_init( false )
    {
        // Load params
        _max_force_thruster_forward = 77.0;
        _max_force_thruster_backward = 43.0;

        // Thrusters model coeficients
        _c1_f = 28.89;
        _c1_b = 60.12;
        _c2 = 8.5;
        _c3 = 1.2 * 0.00442747 * 1.54;
        _c4 = 2 * 0.1 * 1.54;

        Eigen::MatrixXd tcm( 6, _n_thrusters );
        tcm << 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        //std::cout << "tcm: \n" << tcm << "\n";

        // Compute pseudoinverse
        _tcm_inv = ( tcm.transpose() * tcm ).inverse() * tcm.transpose();

        // Subscribers
        _th_info = _n.subscribe( "/cola2_control/thrusters_info", 2, &ThrusterAllocatorHUG::updateThInfo, this );                            //****
    }

    ~ThrusterAllocatorHUG()
    {}

    void
    setParams( const double max_force_thruster_forward,
               const double max_force_thruster_backward,
               const std::vector< double > tcm_values )
    {
        std::cout << "ThrusterAllocatorHUG set params\n";

        _max_force_thruster_forward = max_force_thruster_forward;
        _max_force_thruster_backward = max_force_thruster_backward;
        

        // Init TCM inverse
        std::cout << "tcm_values.size(): " << tcm_values.size() << "\n";
        assert(tcm_values.size() == 6*_n_thrusters);
        Eigen::MatrixXd tcm( 6, _n_thrusters );
        for( unsigned int i = 0; i < 6; i++ ) {
            for( unsigned int j = 0; j < _n_thrusters; j++ ) {
                tcm(i, j) = tcm_values.at( i*_n_thrusters + j );
            }
        }

        //std::cout << "TCM:\n"<< tcm << "\n";
        _tcm_inv = ( tcm.transpose() * tcm ).inverse() * tcm.transpose();
        //std::cout << "TCM inv:\n" << _tcm_inv << "\n";

        std::cout << "ThrusterAllocatorHUG initialized!\n";
        _is_init = true;
    }

    void
    updateThInfo(const cola2_msgs::ThrustersInfo& msg) {                                                                                     // ****
        _th_voltages = msg.thruster_voltages;
        _th_currents = msg.thruster_currents;
        _th_rpms = msg.thruster_rpms;
    }

    Eigen::VectorXd
    compute( Request& wrench, const std::vector< double >& twist )                                                                                      // ****
    {
        if( !_is_init ) return Eigen::MatrixXd::Zero( _n_thrusters, 1 );

        // Take wrench request if disabled false, otherwise, get 0.0.
        //std::cout << "thruster allocator\n" << wrench << "\n";
        Eigen::VectorXd wrench_req( 6 );
        for( unsigned int i = 0; i < wrench.getDisabledAxis().size(); i++ ) {
            if( wrench.getDisabledAxis().at(i) ) {
                wrench_req[i] = 0.0;
            }
            else {
                wrench_req[i] = wrench.getValues().at(i);
            }
        }
        //std::cout << "Wrench:\n" << wrench_req << "\n";

        // Keep the surge velocity and the thruster feedback for the hole iteration
        _AllocatorFeedback feedback_info;
        feedback_info.surge_velocity = twist[0];
        feedback_info.thrusters_voltage = _th_voltages;
        feedback_info.thrusters_current = _th_currents;
        feedback_info.thrusters_rpm = _th_rpms;

        // Merge Surge and Yaw
        //mergeSurgeYaw( wrench_req[0], wrench_req[5], feedback_info );
        // std::cout << "wrench:\n" << wrench_req << "\n";

        // Multiply wrench by thruster allocation matrix
        Eigen::VectorXd force_per_thruster;
        force_per_thruster = _tcm_inv * wrench_req;
        //std::cout << "force_per_thruster: \n" << force_per_thruster << "\n";

        // Force to setpoint
        Eigen::VectorXd setpoint = forceToSetpoint( force_per_thruster, feedback_info );
        //std::cout << "setpoint: \n" << setpoint << "\n";

        // Publish
        return setpoint;
    }


private:
    unsigned int _n_thrusters;
    double _max_force_thruster_forward;
    double _max_force_thruster_backward;
    Eigen::MatrixXd _tcm_inv;
    std::vector< double > _th_voltages;                                                                                                         // ****
    std::vector< double > _th_currents;                                                                                                         // ****
    std::vector< double > _th_rpms;                                                                                                             // ****
    bool _is_init;

    double _c1_f;
    double _c1_b;
    double _c2;
    double _c3;
    double _c4;

    // Node handle
    ros::NodeHandle _n;                                                                                                                         //****

    // Subscriber
    ros::Subscriber _th_info;                                                                                                                   // ****

    struct _AllocatorFeedback {
        double surge_velocity;
        std::vector< double > thrusters_voltage;
        std::vector< double > thrusters_current;
        std::vector< double > thrusters_rpm;
    };

    Eigen::VectorXd
    forceToSetpoint( Eigen::VectorXd& wrench,
                     const struct _AllocatorFeedback& feedback ) {

        Eigen::VectorXd ret( wrench.size() );
        // Compute newtons to setpoints for each thruster
        for( unsigned int i = 0; i < wrench.size(); i++ ) {
            //std::cout << "ret " << i << ": " << ret[i] << "\n";

            // Initialize setpoint
            ret[i] = 0.0;

            if (i == 0) { //vertical thruster

                if ( simulation == false ) {                                                                                                // ***
                    // Adjust force according to voltage level
                    double v_th_correction = 3.4882 - 0.0754 * feedback.thrusters_voltage[i];
                    saturate( v_th_correction, 1.34, 1 );
                    wrench[i] *= v_th_correction;
                }

                // Checking the wrench limit
                saturate( wrench[i], 30, -30 );

                // Applying the thruster model
                if ( wrench[i] > 0.381 ) { // Minimum setpoint of 0.11 justified by wrench request, not by the model offset
                    ret[i] = 0.1 + ( -37.66796 + pow( ( pow( 37.66796 , 2 ) - 4 * 45.69876 * -wrench[i] ) , 0.5 ) ) / ( 2.0 * 45.69876 );
                }
                else if ( wrench[i] < -0.013 ) { // Minimum setpoint of 0.11 justified by wrench request, not by the model offset
                    ret[i] = -0.1 + ( -0.85304 + pow( ( pow( 0.85304 , 2 ) - 4 * -44.78478 * -wrench[i] ) , 0.5 ) ) / ( 2.0 * -44.78478 );
                }
                else {
                    ret[i] = 0.0;
                }
            }
            else { //horitzontal thrusters

                // Checking the wrench limit
                saturate( wrench[i], 77, -43 );

                if ( simulation ) {                                                                                                             // ***
                    // Applying the thruster model
                    if ( wrench[i] > 0 ) {
                        ret[i] = ( 0.000003129 * pow( wrench[i] , 2 ) - 0.000468 * wrench[i] + 0.0305 ) * wrench[i];
                    }
                    else if ( wrench[i] < 0 ) {
                        ret[i] = ( 0.000013317 * pow( wrench[i] , 2 ) + 0.0013 * wrench[i] + 0.0528 ) * wrench[i];
                    }
                    else {
                        ret[i] = 0.0;
                    }
                }
                else {

                    // TODO: if there is not feedback use the static model
                                                                                                                                         // ***
                    double th_rpms = 0;
                    double voltage = 0;

                    if ( wrench[i] > 0 ) {
                        // Taking in account the actual surge velocity
                        if ( ( pow( _c2 * feedback.surge_velocity , 2 ) + 4 * _c1_f * wrench[i] ) > 0 ) {
                            th_rpms = ( ( _c2 * feedback.surge_velocity + pow( fabs( pow( ( _c2 * feedback.surge_velocity ) , 2 ) + 4 * _c1_f * wrench[i] ) , 0.5 ) ) / 2 ) * 60;
                        }
                        else if ( ( pow( _c2 * feedback.surge_velocity , 2 ) + 4 * _c1_f * wrench[i] ) < 0 ) {
                            th_rpms = ( ( _c2 * feedback.surge_velocity - pow( fabs( pow( ( _c2 * feedback.surge_velocity ) , 2 ) + 4 * _c1_f * wrench[i] ) , 0.5 ) ) / 2 ) * 60;
                        }

                        // Applying the thruster model (1st part)
                        if (th_rpms > 0 ) {
                            voltage = _c3 * th_rpms + _c4 * feedback.thrusters_current[i];
                        }
                    }
                    else if ( wrench[i] < 0 ) {
                        // Taking in account the actual surge velocity
                        if ( ( pow( _c2 * feedback.surge_velocity , 2 ) - 4 * _c1_b * wrench[i] ) > 0 ) {
                            th_rpms = ( ( _c2 * feedback.surge_velocity - pow( fabs( pow( ( _c2 * feedback.surge_velocity ) , 2 ) - 4 * _c1_b * wrench[i] ) , 0.5 ) ) / 2 ) * 60;
                        }
                        else if ( ( pow( _c2 * feedback.surge_velocity , 2 ) + 4 * _c1_b * wrench[i] ) < 0 ) {
                            th_rpms = ( ( _c2 * feedback.surge_velocity + pow( fabs( pow( ( _c2 * feedback.surge_velocity ) , 2 ) - 4 * _c1_b * wrench[i] ) , 0.5 ) ) / 2 ) * 60;
                        }

                        // Applying the thruster model (1st part)
                        if ( th_rpms < 0 ) {
                            voltage = _c3 * th_rpms - _c4 * feedback.thrusters_current[i];
                        }
                    }
                    else {
                        ret[i] = 0.0;
                    }

                    if ( ( voltage != 0 ) and ( feedback.thrusters_voltage[i] > 0 ) ) {
                        // Applying the thruster model (2nd part)
                        ret[i] = voltage / feedback.thrusters_voltage[i];
                    }
                }
            }
            // Saturate between -1.0 and 1.0
            saturate( ret[i], 1.0, -1.0 );
            // std::cout << "ret saturate " << i << ": " << ret[i] << "\n";
        }
        return ret;
    }


    void
    saturate( double& value,
              const double max_value,
              const double min_value ) {
        if( value > max_value ) value = max_value;
        if( value < min_value ) value = min_value;
    }
};

#endif // __HUGTHRUSTERALLOCATOR_CLASS__

