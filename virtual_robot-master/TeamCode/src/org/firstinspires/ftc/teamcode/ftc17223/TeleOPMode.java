package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "TeleOP")
public class TeleOPMode extends LinearOpMode {
    RobotDrive robot = new RobotDrive();

    public void runOpMode() {
        robot.initializeRobot(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            robot.mixDrive(forward, strafe, rotate);


        }

    }
}
