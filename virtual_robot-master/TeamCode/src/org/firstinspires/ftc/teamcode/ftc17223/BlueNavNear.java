package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueNavNear")
public class BlueNavNear extends LinearOpMode {

    public void runOpMode() {
        //Initialization code
        RobotDrive robotDrive = new RobotDrive();
        robotDrive.initializeRobot(hardwareMap, telemetry, RobotDrive.color.blue);
        waitForStart();
        //Code to run once once start button is pressed

        robotDrive.strafeEncoder(2, RobotDrive.direction.right);
    }
}