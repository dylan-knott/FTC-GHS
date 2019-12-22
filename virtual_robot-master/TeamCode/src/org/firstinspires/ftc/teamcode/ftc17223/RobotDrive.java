package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import jdk.nashorn.internal.ir.Block;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class RobotDrive {
    //Proportional Processing values
    private final double P_Forward = 0.01;
    private final double P_Strafe = 0.01;
    private final double P_Turn = 0.01;


    Telemetry telemetry = null;
    final double TURN_P = 0.02;
    final double wheelDiameter = 3.93701;

    //Hardware
    private DcMotor leftfront, leftrear, rightfront, rightrear, armLift = null;
    private BNO055IMU imu = null;
    private DistanceSensor dist = null;
    private ColorSensor colorSensor = null;
    private Servo BlockGrips, TopServo, MatServos, SideArm = null;

    public final double motorPower = 0.8;
    public final double liftPower = 0.5;

    //Debug the error angle in order to get this value
    private double turningBuffer = 3.4820556640625;

    enum direction {
        left, right;
    }

    //Assigning software objects to hardware, recieves hardwareMap and telemetry objects from the op mode
    void initializeRobot(HardwareMap hardwareMap, Telemetry telem) {
        telemetry = telem;
        RobotDrive.direction strafeDirection;

        //Initialize Hardware
        leftfront = hardwareMap.dcMotor.get("front_left_motor");
        rightfront = hardwareMap.dcMotor.get("front_right_motor");
        leftrear = hardwareMap.dcMotor.get("back_left_motor");
        rightrear = hardwareMap.dcMotor.get("back_right_motor");
        //armLift = hardwareMap.dcMotor.get("armLift");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //dist = hardwareMap.get(DistanceSensor.class, "distance");
        //BlockGrips = hardwareMap.servo.get("claw_servos");
        //TopServo = hardwareMap.servo.get("top_servo");
        //MatServos = hardwareMap.servo.get("mat_servos");
        //SideArm = hardwareMap.servo.get("side_arm");
        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSense");

        //armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);


        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);


    }


    /***************************************FORWARD MOVEMENT***************************************/
    //Send this function a value of seconds to drive for and it will drive for that period
    void driveTime(long time) throws InterruptedException {
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

    //Send this function a number of inches and it will drive that distance using the encoders on the motors.
    void driveEncoder(double Inches) {
        float initialHeading = getHeading();
        DcMotor motors[] = {leftfront, rightfront, rightrear, leftrear};
        int encoderTicks = (int)((1440 / (wheelDiameter * Math.PI)) * Inches);

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(encoderTicks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        while (leftfront.isBusy()) {
            //wait until the motors are done running
            while (Math.abs((initialHeading - getHeading()) % 360) > 1) {
                double degreesCorrect = (initialHeading - getHeading()) % 360;
                double motorCorrect = clamp(degreesCorrect * TURN_P, -.4, .4);
                leftfront.setPower(motorPower - motorCorrect);
                leftrear.setPower(motorPower - motorCorrect);
                rightfront.setPower(motorPower + motorCorrect);
                rightrear.setPower(motorPower + motorCorrect);
            }
        }

        for (DcMotor motor : motors) motor.setPower(0);
        for (DcMotor motor : motors) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return;
    }

    /*******************************************STRAFING*******************************************/
    //Send this function a time value as well as a direction (Ex: RobotDrive.direction.left) and it will strafe that direction for the specified amount of time
    void strafeTime(int time, RobotDrive.direction strafeDirection) throws InterruptedException {
        if (strafeDirection == RobotDrive.direction.left) {
            leftfront.setPower(-1 * motorPower);
            leftrear.setPower(motorPower);
            rightrear.setPower(-1 * motorPower);
            rightfront.setPower(motorPower);
        } else if (strafeDirection == RobotDrive.direction.right) {
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
    //Send this function a distance in inches as well as a direction (Ex: RobotDrive.direction.right) and it will strafe that direction for the specified distance
    void strafeEncoder(double Inches, RobotDrive.direction direction) {
        float initialHeading = getHeading();
        DcMotor motors[] = {leftfront, rightfront, rightrear, leftrear};
        int encoderTicks = (int) ((1440 / (wheelDiameter * Math.PI)) * Inches);
        if (direction == RobotDrive.direction.left) encoderTicks *= -1;
        for (DcMotor motor : motors) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setTargetPosition(encoderTicks);
        leftrear.setTargetPosition(-1 * encoderTicks);
        rightfront.setTargetPosition(-1 * encoderTicks);
        rightrear.setTargetPosition(encoderTicks);
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (byte i = 10; i > 0; i--) {
            for (DcMotor motor : motors) {
                motor.setPower(motorPower / i);
            }
        }

        while (leftfront.isBusy()) {
            //wait until the motors are done running
            while (Math.abs((initialHeading - getHeading()) % 360) > 1) {
                double degreesCorrect = (initialHeading - getHeading()) % 360;
                double motorCorrect = clamp(degreesCorrect * TURN_P, -.4, .4);
                leftfront.setPower(motorPower - motorCorrect);
                leftrear.setPower(motorPower - motorCorrect);
                rightfront.setPower(motorPower + motorCorrect);
                rightrear.setPower(motorPower + motorCorrect);
            }
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return;
    }


    /*******************************************TURNING********************************************/
    //Send this a value of degrees to turn, positive value turns right and negative value turns left, uses IMU gyroscope to precisely turn that distance
    void gyroTurn(double degrees) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target_angle = getHeading() - degrees;
        if (degrees < 0) {target_angle += turningBuffer;} else if (degrees > 0) {target_angle -= turningBuffer;}
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
            telemetry.addData("Current Heading : ", String.format(Locale.getDefault(), "%.1f", angles.firstAngle * -1));
            telemetry.update();
        }

        telemetry.addData("Error Degrees: ", Math.abs(target_angle - angles.firstAngle) % 360);
        telemetry.update();
        leftfront.setPower(0);
        rightfront.setPower(0);
        leftrear.setPower(0);
        rightrear.setPower(0);
    }

    //Returns the current heading of the robot when it is called, takes reading from the IMU gyroscope
    float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    /*********************************************SERVOS*********************************************/

    //Sets the angle of the servo on the side arm. Give value in degrees as well as the maximum turning degrees of the servo. Converts to a fraction usable by the servo
   void SetSideArm(int desiredRotation, int maxRotation) {
       SideArm.setPosition(desiredRotation / maxRotation);
   }
   //Activates the back servos used to grab the mat, send angle of rotation in degrees as well as the max angle of the servo. Converts to a fraction usable by the servo
   void grabMat(int desiredRotation) { MatServos.setPosition(desiredRotation /280);}

   void controlClaw(int desiredRotation) {
       BlockGrips.setPosition(desiredRotation / 280);
       TopServo.setPosition(desiredRotation / 180 );
   }

    /*******************************************UTILITIES*******************************************/
    //Creating a clamp method for both floats and doubles, used to make sure motor power doesn't go above a certain power level as to saturate the motors
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static double clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }

    //For debugging, displays current encoder values of each wheel
    void getEncoderVals() {
        telemetry.addData("Encoders (LF, RF, LR, RR)", "%d %d %d %d",
                leftfront.getCurrentPosition(),
                rightfront.getCurrentPosition(),
                leftrear.getCurrentPosition(),
                rightrear.getCurrentPosition());
        telemetry.update();
    }

    //Driving function that accepts three values for forward, strafe, and rotate, then mixes those values into 4 usable motor power outputs and sends to the motors.
    void mixDrive(double forward, double strafe, double rotate) {
        double frontLeftSpeed = clamp((forward + strafe + rotate), -motorPower, motorPower);
        double frontRightSpeed = clamp((forward - strafe - rotate), -motorPower, motorPower);
        double backLeftSpeed = clamp((forward - strafe + rotate), -motorPower, motorPower);
        double backRightSpeed = clamp((forward + strafe - rotate), -motorPower, motorPower);

        leftfront.setPower(frontLeftSpeed);
        rightfront.setPower(frontRightSpeed);
        leftrear.setPower(backLeftSpeed);
        rightrear.setPower(backRightSpeed);
    }

    //turning distance measurements into usable motor output via Proportional control)
    void DistanceToDrive(double forward, double right, double turn){

        double ForwardOut = forward * P_Forward;
        double RightOut = right * P_Strafe;
        double TurnOut = turn * P_Turn;
        mixDrive(ForwardOut, RightOut, TurnOut);
    }

    void LiftEncoder(double ticks) {
        armLift.setTargetPosition((int)(armLift.getCurrentPosition() + ticks));
        if (armLift.getTargetPosition() < 0) {armLift.setTargetPosition(0);}
        armLift.setPower(liftPower);
        while (armLift.isBusy());
        armLift.setPower(0);
    }
}