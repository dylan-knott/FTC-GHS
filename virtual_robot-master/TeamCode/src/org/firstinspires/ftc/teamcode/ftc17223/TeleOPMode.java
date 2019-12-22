package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "TeleOP")
public class TeleOPMode extends LinearOpMode {
    RobotDrive robot = new RobotDrive();

    public void runOpMode() {
        boolean mat = false;
        boolean claw = false;

        robot.initializeRobot(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            //Gamepad 1
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            robot.mixDrive(forward, strafe, rotate);
            if (gamepad1.right_trigger > 0.5) {
               if (mat == false) {
                   robot.grabMat(90);
                   mat = true;
                }
               else {
                   robot.grabMat(0);
                   mat = false;
               }

            }

            //Gamepad 2
            robot.LiftEncoder(gamepad2.right_stick_y);
            if (gamepad2.right_trigger > 0.5) {
                if (claw == false) {
                    robot.controlClaw(25);
                    claw = true;
                    }
                else {
                    robot.controlClaw(0);
                    claw = false;
                }
                }

            }
        }

    }
