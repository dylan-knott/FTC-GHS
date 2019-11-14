package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class RobotDrive{

    static double TURN_P = 0.01;
    static int wheelDiameter = 4;
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftrear = null;
    private DcMotor rightrear = null;
    BNO055IMU imu = null;
    DistanceSensor backdist = null;

    double intendedHeading;
    public double motorPower = 0.25;

    private double turningBuffer  = 2.57;

    enum direction{
        left, right;
    }


    void initializeRobot(HardwareMap hardwareMap){
        RobotDrive.direction strafeDirection;
        leftfront = hardwareMap.dcMotor.get("front_left_motor");
        rightfront = hardwareMap.dcMotor.get("front_right_motor");
        leftrear = hardwareMap.dcMotor.get("back_left_motor");
        rightrear = hardwareMap.dcMotor.get("back_right_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backdist = hardwareMap.get(DistanceSensor.class, "back_distance");

        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);

        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        double degreesError;


    }


    /***************************************FORWARD MOVEMENT***************************************/
    void driveForward(long time)  throws InterruptedException {
        leftrear.setPower(motorPower);
        leftfront.setPower(motorPower);
        rightrear.setPower(motorPower);
        rightfront.setPower(motorPower);
        Thread.sleep(time);
        leftrear.setPower(0);
        leftfront.setPower(0);
        rightrear.setPower(0);
        rightfront.setPower(0);


    }

    void driveForward(double Inches) {
        int encoderTicks = (int)((360 / (wheelDiameter * Math.PI)) * Inches);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfront.setTargetPosition(encoderTicks);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*******************************************STRAFING*******************************************/
    void strafe(int time, RobotDrive.direction strafeDirection) throws InterruptedException {
        if (strafeDirection == RobotDrive.direction.left){
            leftfront.setPower(-1 * motorPower);
            leftrear.setPower(motorPower);
            rightrear.setPower(-1 * motorPower);
            rightfront.setPower(motorPower);
        }
        else if (strafeDirection == RobotDrive.direction.right){
            leftfront.setPower(motorPower);
            leftrear.setPower(-1 * motorPower);
            rightrear.setPower(motorPower);
            rightfront.setPower(-1 * motorPower);
        }
        Thread.sleep(time);
        leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);
    }



    /*******************************************TURNING********************************************/
    //Handling turning 90 degrees
    double gyroTurn(double degrees, Telemetry telemetry) {
        Orientation angles =  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target_angle = getHeading() - (degrees + turningBuffer);
        while (Math.abs((target_angle - getHeading()) % 360) > 3) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute turning error
            double motor_output = clamp(error_degrees * TURN_P, -.9, .9); // Get Correction of error
            //Send corresponding value to motors
            leftfront.setPower(-1 * motor_output);
            leftrear.setPower(-1 * motor_output);
            rightfront.setPower(motor_output);
            rightrear.setPower(motor_output);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Target Angle : ", target_angle - turningBuffer);
            telemetry.addData("Current Heading : ",String.format(Locale.getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }
        return(Math.abs(target_angle - angles.firstAngle) % 360);

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


    void getEncoderVals(Telemetry telemetry){
        telemetry.addData("Encoders", "%d %d %d %d",
                leftfront.getCurrentPosition(),
                rightfront.getCurrentPosition(),
                leftrear.getCurrentPosition(),
                rightrear.getCurrentPosition());
    }

}
