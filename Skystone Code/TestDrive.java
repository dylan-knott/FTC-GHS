package org.firstinspires.ftc.teamcode.ftc17223;

import android.text.format.Time;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
    VuforiaClass vuforiaClass = new VuforiaClass();

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing");
        telemetry.update();
        //Initialization Code
        robotDrive.initializeRobot(hardwareMap, telemetry, RobotDrive.color.blue);
        vuforiaClass.InitVuforia(hardwareMap, telemetry, RobotDrive.color.blue);
        telemetry.addLine("Robot is initialized");
        telemetry.update();
        //Runs 1 time once start is pressed
        waitForStart();


        //Runs in a loop after start
        //while (opModeIsActive()) {
        vuforiaClass.seekStone();
        Thread.sleep(100);
        robotDrive.strafeEncoder(14, RobotDrive.direction.left);
        robotDrive.mixDrive(0,0,0);
        robotDrive.driveEncoder(15);
        robotDrive.SetSideArm(0, 180);
        robotDrive.mixDrive(0,0.3,0);
        Thread.sleep(100);
        robotDrive.mixDrive(0,0,0);
        Thread.sleep(50);
        robotDrive.driveEncoder(-25);
        //}

    }

}
