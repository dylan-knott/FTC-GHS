package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Foundation Far")
public class RedFoundNav extends LinearOpMode {

    public void runOpMode() {
        //Initialization code
        telemetry.addLine("Initializing");
        telemetry.update();
        RobotDrive robotDrive = new RobotDrive();
        robotDrive.initializeRobot(hardwareMap, telemetry, RobotDrive.color.red);
        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();
        //Code to run once once start button is pressed

        robotDrive.driveEncoder(25);
        robotDrive.strafeEncoder(7, RobotDrive.direction.right);
        robotDrive.seekMat();
    }
}
