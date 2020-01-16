package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Foundation Far")
public class BlueFoundNav extends LinearOpMode {

    public void runOpMode() {
        //Initialization code
        telemetry.addData("Status","Initializing");
        telemetry.update();
        RobotDrive robotDrive = new RobotDrive();
        robotDrive.initializeRobot(hardwareMap, telemetry, RobotDrive.color.blue);
        telemetry.addData("Status","Robot Initialized");
        telemetry.update();

        waitForStart();
        //Code to run once once start button is pressed

        robotDrive.driveEncoder(25);
        robotDrive.strafeEncoder(7, RobotDrive.direction.left);
        robotDrive.seekMat();
    }
}
