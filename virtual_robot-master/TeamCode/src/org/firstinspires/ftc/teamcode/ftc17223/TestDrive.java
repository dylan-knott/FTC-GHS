package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        //Initialization Code
        robotDrive.initializeRobot(hardwareMap);


        //Runs 1 time once start is pressed
        waitForStart();


        //Runs in a loop after start
        while (opModeIsActive()) {
        robotDrive.driveEncoder(100);
        robotDrive.gyroTurn(180, telemetry);
        robotDrive.strafeEncoder( 100, RobotDrive.direction.left);
        robotDrive.driveEncoder(100);
        robotDrive.strafeEncoder( 100, RobotDrive.direction.right);
        robotDrive.gyroTurn(270, telemetry);
        }

    }

}
