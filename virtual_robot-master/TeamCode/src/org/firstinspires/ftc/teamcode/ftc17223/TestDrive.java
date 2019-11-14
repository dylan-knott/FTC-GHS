package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

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
    RobotDrive robotDrive = new RobotDrive();

    public void runOpMode() throws InterruptedException {
        robotDrive.initializeRobot(hardwareMap);

        waitForStart();
        //Runs 1 time once start is pressed

        //Runs in a loop after start
        while (opModeIsActive()) {
        robotDrive.driveForward(2000);
        robotDrive.gyroTurn(180, telemetry);
        robotDrive.strafe( 2000, RobotDrive.direction.left);
        robotDrive.driveForward( 2000);
        robotDrive.strafe( 2000, RobotDrive.direction.right);
        robotDrive.gyroTurn(270, telemetry);
        }

    }

}
