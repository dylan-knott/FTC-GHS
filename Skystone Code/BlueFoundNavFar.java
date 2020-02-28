package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Foundation Far")
@Disabled
public class BlueFoundNavFar extends LinearOpMode {

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

        robotDrive.driveEncoder(25);
        Thread.sleep(50);
        robotDrive.mixDrive(0, -0.3, 0);
        Thread.sleep(900);
        robotDrive.mixDrive(0,0,0);
        Thread.sleep(50);
        robotDrive.seekMat();
    }
}
