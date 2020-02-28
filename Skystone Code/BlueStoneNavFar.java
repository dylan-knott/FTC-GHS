package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Blue 1 Stone Nav Far")
public class BlueStoneNavFar extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        //Initialization code
        telemetry.addLine("Initializing");
        telemetry.update();
        RobotDrive robotDrive = new RobotDrive();
        VuforiaClass vuforiaClass = new VuforiaClass();
        robotDrive.initializeRobot(hardwareMap, telemetry, RobotDrive.color.blue);
        vuforiaClass.InitVuforia(hardwareMap, telemetry, RobotDrive.color.blue);
        telemetry.addLine("Robot Initialized");
        telemetry.update();
        //Code to run once once start button is pressed
        waitForStart();

        robotDrive.driveEncoder(21);
        robotDrive.gyroTurn(-90);
        robotDrive.mixDrive(0,0,0);
        Thread.sleep(50);
        robotDrive.mixDrive(-0.3, 0, 0);
        while (robotDrive.dist.getDistance(DistanceUnit.INCH) > 15);
        robotDrive.mixDrive(0,0,0);
        telemetry.addLine("Searching for stone");
        telemetry.update();
        vuforiaClass.seekStone();
        Thread.sleep(100);
        robotDrive.strafeEncoder(14, RobotDrive.direction.left);
        robotDrive.mixDrive(0.3, 0, 0);
        //Drive forward until the blue middle line is hit
        while (robotDrive.colorSensor.blue() < 188);
        //Stop the robot
        robotDrive.mixDrive(0,0,0);
        //Drive an additional 15 inches before dropping the stone and returning to center
        robotDrive.driveEncoder(23);
        robotDrive.SetSideArm(80, 180);
        robotDrive.driveEncoder(-17);
        robotDrive.SetSideArm(0, 180);
        robotDrive.mixDrive(0, 0.3, 0);
        Thread.sleep(600);
        robotDrive.mixDrive(0,0,0);
    }
}
