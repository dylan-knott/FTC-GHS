package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@Autonomous(name = "ColorTest")
public class ColorTest extends LinearOpMode {
    RobotDrive robotDrive = new RobotDrive();
    VuforiaClass vuforiaClass = new VuforiaClass();

    public void runOpMode() throws InterruptedException {
        //Initialization Code
        robotDrive.initializeRobot(hardwareMap, telemetry, RobotDrive.color.red);
        vuforiaClass.InitVuforia(hardwareMap, telemetry, RobotDrive.color.red);

        //Runs 1 time once start is pressed
        waitForStart();

        while (opModeIsActive()) {
            if (robotDrive.colorSensor.red() > robotDrive.foundThreshold) robotDrive.grabMat(90);
            else robotDrive.grabMat(0);

            telemetry.addData("Distance: ", robotDrive.dist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }
}