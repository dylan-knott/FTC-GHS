package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * Left-click to position bot.
 * Right-click to orient bot.
 *
 * CONFIG
 *  Motors:
 *    left_motor
 *    right_motor
 *  Servos:
 *    back_servo
 *  Color Sensors:
 *    color_sensor
 *  Gyro Sensors:
 *    gyro_sensor
 *  Distance Sensors:
 *    left_distance
 *    right_distance
 *    front_distance
 *    back_distance
 *
 */


public class TwoWheelDrive {

    static double TURN_P = 0.01;
    static int wheelDiameter = 4;
    private DcMotor leftWheel = null;
    private DcMotor rightWheel = null;
    BNO055IMU imu = null;
    DistanceSensor distf = null;
    DistanceSensor distb = null;
    DistanceSensor distl = null;
    DistanceSensor distr = null;


    double intendedHeading;
    public double motorPower = 0.5;

    private double turningBuffer = 2.57;

    enum direction {
        left, right
    }


    void initializeRobot(HardwareMap hardwareMap) {
        RobotDrive.direction strafeDirection;
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        distb = hardwareMap.get(DistanceSensor.class, "back_distance");
        distf = hardwareMap.get(DistanceSensor.class, "front_distance");
        distl = hardwareMap.get(DistanceSensor.class, "left_distance");
        distr = hardwareMap.get(DistanceSensor.class, "right_distance");

        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        double degreesError;


    }

/*
    /***************************************FORWARD MOVEMENT***************************************/
    void driveTime(long time) throws InterruptedException {
        leftWheel.setPower(motorPower);
        rightWheel.setPower(motorPower);
        Thread.sleep(time);
        leftWheel.setPower(0);
        rightWheel.setPower(0);

    }

    void driveEncoder(double Inches) {
        DcMotor motors[] = {leftWheel, rightWheel};
        int encoderTicks = (int) ((360 / (wheelDiameter * Math.PI)) * Inches);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(encoderTicks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(motorPower);
        }

        while (leftWheel.isBusy() && rightWheel.isBusy()) {
            //wait until the motors are done running
        }

        for (DcMotor motor : motors) motor.setPower(0);
        for (DcMotor motor : motors) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /*******************************************TURNING********************************************/
    //Handling turning 90 degrees
    double gyroTurn(double degrees, Telemetry telemetry) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target_angle = getHeading() - (degrees + turningBuffer);
        while (Math.abs((target_angle - getHeading()) % 360) > 3) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute turning error
            double motor_output = clamp(error_degrees * TURN_P, -.9, .9); // Get Correction of error
            //Send corresponding value to motors
           leftWheel.setPower(-1 * motor_output);
           rightWheel.setPower(motor_output);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Target Angle : ", target_angle - turningBuffer);
            telemetry.addData("Current Heading : ", String.format(Locale.getDefault(), "%.1f", angles.firstAngle * -1));
            telemetry.update();
        }
        return (Math.abs(target_angle - angles.firstAngle) % 360);

    }

    //Read value for imu and convert to double
    float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    /*******************************************UTILITIES*******************************************/
    //Creating a clamp method for both floats and doubles
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }


    void getEncoderVals(Telemetry telemetry) {
        telemetry.addData("Encoders", "%d %d",
                leftWheel.getCurrentPosition(),
                rightWheel.getCurrentPosition());
    }


    void mixDrive(double forward, double rotate) {

        double LeftSpeed = clamp((forward + rotate), -motorPower, motorPower);
        double RightSpeed = clamp((forward- rotate), -motorPower, motorPower);

    leftWheel.setPower(LeftSpeed);
    rightWheel.setPower(RightSpeed);
    }

}