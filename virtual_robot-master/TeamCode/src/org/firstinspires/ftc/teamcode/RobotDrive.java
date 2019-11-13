package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class RobotDrive extends LinearOpMode{

    private static double TURN_P = 0.01;
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftrear = null;
    private DcMotor rightrear = null;
    BNO055IMU imu = null;
    private DistanceSensor backdist = null;

    double intendedHeading;
    double turningBuffer  = 2.57;

    enum direction{
        left, right;
    }

    public void runOpMode() {

    }

}
