package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.vuforia.Vuforia;

@Autonomous(name = "Red 1 Stone Nav Far")
public class RedStoneNavFar extends LinearOpMode {

    public void runOpMode() {
        //Initialization code
        telemetry.addData("Status","Initializing");
        telemetry.update();
        RobotDrive robotDrive = new RobotDrive();
        //VuforiaClass vuforiaClass = new VuforiaClass();
        robotDrive.initializeRobot(hardwareMap, telemetry, RobotDrive.color.red);
        //vuforiaClass.InitVuforia(hardwareMap, telemetry, RobotDrive.color.red);
        waitForStart();
        telemetry.addData("Caption","Robot Initialized");
        telemetry.update();
        //Code to run once once start button is pressed

        robotDrive.driveEncoder(20);
        robotDrive.strafeEncoder(5, RobotDrive.direction.left);
        robotDrive.gyroTurn(-90);
        telemetry.addData("Caption","Searching for stone");
        telemetry.update();
        //vuforiaClass.seekStone();
        robotDrive.strafeEncoder(7, RobotDrive.direction.left);
        robotDrive.gyroTurn(-180);
        robotDrive.mixDrive(robotDrive.motorPower, 0, 0);
        //Drive forward until the blue middle line is hit
        while (robotDrive.colorSensor.red() < robotDrive.colorThreshold);
        //Stop the robot
        robotDrive.mixDrive(0,0,0);
        //Drive an additional 11 inches before dropping the stone and returning to center
        robotDrive.driveEncoder(11);
        robotDrive.SetSideArm(0, 180);
        robotDrive.driveEncoder(-7);
    }
}
