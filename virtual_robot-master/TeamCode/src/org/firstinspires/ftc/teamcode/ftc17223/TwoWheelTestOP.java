package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TwoWheelTest")
public class TwoWheelTestOP extends LinearOpMode {
    TwoWheelDrive robot = new TwoWheelDrive();;

    public void runOpMode() {
        robot.initializeRobot(hardwareMap);

        //Runs once when start is pressed
        waitForStart();


        while (opModeIsActive()){
            robot.driveEncoder(100);
            robot.gyroTurn(90, telemetry);
        }
    }
}
