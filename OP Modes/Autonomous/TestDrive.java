package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**-----------Hardware Map-------------
 Motors:
 back_right_motor
 front_right_motor
 front_left_motor
 back_left_motor
 Servos:
 back_servo
 Color Sensors:
 color_sensor
 BNO055IMU Sensors:
 imu
 Distance Sensors:
 left_distance
 right_distance
 front_distance
 back_distance
 **/
@Autonomous(name = "TestDrive")
public class TestDrive extends LinearOpMode {
    private static double TURN_P = 0.01;
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftrear = null;
    private DcMotor rightrear = null;
    BNO055IMU imu = null;
    double intendedHeading;

    public void runOpMode() throws InterruptedException {
        //Initialization
        leftfront = hardwareMap.dcMotor.get("front_left_motor");
        rightfront = hardwareMap.dcMotor.get("front_right_motor");
        leftrear = hardwareMap.dcMotor.get("back_left_motor");
        rightrear = hardwareMap.dcMotor.get("back_right_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);

        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        double degreesError;
        double motorPower = 0.25;


        waitForStart();

        while (opModeIsActive()) {
            //Go forward for 2 seconds
            driveForward(0.5, 2000);
            //Turn right 90 degrees
            degreesError = gyroTurn(90);
            telemetry.addData("Degrees Error: ", degreesError);
            telemetry.addData("Imu heading: ", getHeading());
            telemetry.update();
        }
    }



    private void driveForward(double motorPower, int time)  throws InterruptedException {

        leftrear.setPower(motorPower);
        leftfront.setPower(motorPower);
        rightrear.setPower(motorPower);
        rightfront.setPower(motorPower);
        Thread.sleep(time);
    }

    //Handling turning 90 degrees
    private double gyroTurn(double degrees) {
        intendedHeading += degrees;
        Orientation angles =  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target_angle = getHeading() - degrees;
        while (Math.abs((target_angle - getHeading()) % 360) > 3 && opModeIsActive()) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute turning error
            double motor_output = clamp(error_degrees * TURN_P, -.9, .9); // Get Correction of error
            //Send corresponding value to motors
            leftfront.setPower(-1 * motor_output);
            leftrear.setPower(-1 * motor_output);
            rightfront.setPower(motor_output);
            rightrear.setPower(motor_output);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Target Angle : ", target_angle);
            telemetry.addData("Current Heading : ",String.format(Locale .getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }
        return(Math.abs(target_angle - angles.firstAngle) % 360);

    }

    //Read value for imu and convert to double
    private float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    //Creating a clamp method for both floats and doubles
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }

}
