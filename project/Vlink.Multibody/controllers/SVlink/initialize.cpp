#include "Vlink.h"
Vlink::Vlink() {
    mode = CONTROL;
    timeStep = 8;
    COM = new timeFunc(timeStep);
    //left leg time variable init
    lleg->phi_b = new timeFunc(timeStep);
    lleg->phi_f = new timeFunc(timeStep);
    lleg->dl0 = new timeFunc(timeStep);
    lleg->foot_angle = new timeFunc(timeStep);
    lleg->dtheta = new timeFunc(timeStep);
    //right leg time variable init
    rleg->phi_b = new timeFunc(timeStep);
    rleg->phi_f = new timeFunc(timeStep);
    rleg->dl0 = new timeFunc(timeStep);
    rleg->foot_angle = new timeFunc(timeStep);
    rleg->dtheta = new timeFunc(timeStep);
    //GPS time variable init
    gps_pos[0] = new timeFunc(timeStep);
    gps_pos[1] = new timeFunc(timeStep);
    gps_pos[2] = new timeFunc(timeStep);
    l0_pid_l  = new PID(pid_index[0], pid_index[1], pid_index[2], l0_exp_l, incrementalPID, timeStep);
    l0_pid_r  = new PID(pid_index[0], pid_index[1], pid_index[2], l0_exp_r, incrementalPID, timeStep);
    theta_pid = new PID(pid_index[3], pid_index[4], pid_index[5], 0, incrementalPID, timeStep);
    turn_pid  = new PID(pid_index[6], pid_index[7], pid_index[8], 0, intengralPID, timeStep);
    roll_pid = new PID(pid_index[9], pid_index[10], pid_index[11], 0, incrementalPID, timeStep);

    followtest = new PathFollow(NULL,1,0.5);
    receiver = getReceiver("receiver");
    receiver->enable(timeStep);
    gps = getGPS("gps");
    gps->enable(timeStep);
    ACC = getAccelerometer("ACC");
    ACC->enable(timeStep);
    gyro = getGyro("gyro");
    gyro->enable(timeStep);
    imu = getInertialUnit("IMU");
    imu->enable(timeStep);
    key = getKeyboard();
    key->enable(timeStep);
    lleg->flag = left_leg;
    rleg->flag = right_leg;
    //connector
    
    armature_l=getConnector("armature_l");
    armature_r=getConnector("armature_r");
    magnet_l=getConnector("magnet_l");
    magnet_r=getConnector("magnet_r");
    armature_l->enablePresence(timeStep);
    armature_r->enablePresence(timeStep);
    magnet_l->enablePresence(timeStep);
    magnet_r->enablePresence(timeStep);
    
    //get four hip sensor
    lleg->hipSensor_f = getPositionSensor("hip_lf_sensor");
    rleg->hipSensor_f = getPositionSensor("hip_rf_sensor");
    lleg->hipSensor_b = getPositionSensor("hip_lb_sensor");
    rleg->hipSensor_b = getPositionSensor("hip_rb_sensor");

    lleg->hipSensor_f->enable(timeStep);
    rleg->hipSensor_f->enable(timeStep);
    lleg->hipSensor_b->enable(timeStep);
    rleg->hipSensor_b->enable(timeStep);
    //get four hip motor
    lleg->hip_f = getMotor("hip_lf");
    rleg->hip_f = getMotor("hip_rf");
    lleg->hip_b = getMotor("hip_lb");
    rleg->hip_b = getMotor("hip_rb");
    //get manipulator motor
    Manipulator_L = new manipulator();
    Manipulator_R = new manipulator();
    Manipulator_R->J0 = getMotor("r0");
    Manipulator_R->J1 = getMotor("r1");
    Manipulator_R->J2 = getMotor("r2");
    Manipulator_R->J0_sensor = getPositionSensor("r0_sensor");
    Manipulator_R->J1_sensor = getPositionSensor("r1_sensor");
    Manipulator_R->J2_sensor = getPositionSensor("r2_sensor");
    Manipulator_R->J0_sensor->enable(timeStep);
    Manipulator_R->J1_sensor->enable(timeStep);
    Manipulator_R->J2_sensor->enable(timeStep);

    Manipulator_L->J0 = getMotor("l0");
    Manipulator_L->J1 = getMotor("l1");
    Manipulator_L->J2 = getMotor("l2");
    Manipulator_L->J0_sensor = getPositionSensor("l0_sensor");
    Manipulator_L->J1_sensor = getPositionSensor("l1_sensor");
    Manipulator_L->J2_sensor = getPositionSensor("l2_sensor");
    Manipulator_L->J0_sensor->enable(timeStep);
    Manipulator_L->J1_sensor->enable(timeStep);
    Manipulator_L->J2_sensor->enable(timeStep);

    lleg->footSensor = getPositionSensor("foot_lb_sensor");
    rleg->footSensor = getPositionSensor("foot_rb_sensor");

    lleg->footSensor->enable(timeStep);
    rleg->footSensor->enable(timeStep);
    //get wheel motor
    lleg->wheel = getMotor("foot_lb");
    rleg->wheel = getMotor("foot_rb");
    cout << "Vlink initialized" << endl;
}