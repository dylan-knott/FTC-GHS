package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TwoWheelTest")
public class TwoWheelTestOP extends LinearOpMode {
    TwoWheelDrive robot = new TwoWheelDrive();;

    public void runOpMode() throws InterruptedException {
        robot.initializeRobot(hardwareMap);

        //Runs once when start is pressed
        waitForStart();
        robot.driveTime(2000);
        robot.gyroTurn(90, telemetry);
        robot.driveTime(2000);
        robot.gyroTurn(90, telemetry);
        robot.driveTime(2000);
        robot.gyroTurn(90, telemetry);
        robot.driveTime(2000);
        robot.gyroTurn(90, telemetry);
    }
}
