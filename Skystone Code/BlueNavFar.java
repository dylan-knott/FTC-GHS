package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Park Far")
public class BlueNavFar extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        //Initialization code
        telemetry.addLine("Initializing");
        telemetry.update();
        RobotDrive robotDrive = new RobotDrive();
        robotDrive.initializeRobot(hardwareMap, telemetry, RobotDrive.color.blue);
        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();
        //Code to run once once start button is pressed
        Thread.sleep(10000);
        robotDrive.driveEncoder(22);
        robotDrive.controlClaw(30);
        robotDrive.SetSideArm(80, 180);
        robotDrive.mixDrive(0, -0.2, 0);
        Thread.sleep(600);
        robotDrive.mixDrive(0,0,0);
    }
}
