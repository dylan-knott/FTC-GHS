package org.firstinspires.ftc.teamcode.ftc17223;

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

@Autonomous(name = "TestTurn")
@Disabled
public class TestTurn extends LinearOpMode {
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


        vuforiaClass.seekStone();

    }

}
