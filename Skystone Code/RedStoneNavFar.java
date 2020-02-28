package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Red 1 Stone Nav Far")
public class RedStoneNavFar extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        //Initialization code
        telemetry.addLine("Initializing");
        telemetry.update();
        RobotDrive robotDrive = new RobotDrive();
        VuforiaClass vuforiaClass = new VuforiaClass();
        robotDrive.initializeRobot(hardwareMap, telemetry, RobotDrive.color.red);
        vuforiaClass.InitVuforia(hardwareMap, telemetry, RobotDrive.color.red);
        telemetry.addLine("Robot Initialized");
        telemetry.update();
        //Code to run once once start button is pressed
        waitForStart();

        robotDrive.driveEncoder(23);
        robotDrive.gyroTurn(-90);
        robotDrive.mixDrive(0,0,0);
        Thread.sleep(50);
        robotDrive.mixDrive(0.3, 0, 0);
        Thread.sleep(800);
        robotDrive.mixDrive(0,0,0);
        telemetry.addLine("Searching for stone");
        telemetry.update();
        vuforiaClass.seekStone();
        Thread.sleep(100);
        robotDrive.strafeEncoder(14, RobotDrive.direction.left);
        robotDrive.gyroTurn(180, 0.4);
        robotDrive.mixDrive(0,0,0);
        Thread.sleep(50);
        robotDrive.mixDrive(0.3, 0, 0);
        //Drive forward until the blue middle line is hit
        while (robotDrive.colorSensor.red() < 188);
        //Stop the robot
        robotDrive.mixDrive(0,0,0);
        //Drive an additional 15 inches before dropping the stone and returning to center
        robotDrive.driveEncoder(23);
        robotDrive.SetSideArm(80, 180);
        robotDrive.driveEncoder(-16);
        robotDrive.SetSideArm(0, 180);

    }
}
