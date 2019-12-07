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
        robotDrive.initializeRobot(hardwareMap, telemetry);

        //Runs 1 time once start is pressed
        waitForStart();


        //Runs in a loop after start
        //while (opModeIsActive()) {


        robotDrive.driveEncoder(10);
        //robotDrive.gyroTurn(90);
        //this.sleep(10000);
        //}

    }

}
