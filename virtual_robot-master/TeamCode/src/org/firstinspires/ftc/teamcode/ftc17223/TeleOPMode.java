package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "TeleOP")
public class TeleOPMode extends LinearOpMode {
    RobotDrive robot = new RobotDrive();

    public void runOpMode() {
        boolean mat = false;
        boolean claw = false;
        boolean sideArm = false; //Outside of loop()
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.color.blue);

        waitForStart();

        while (opModeIsActive()) {
            //Gamepad 1
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            //Wheel control
            robot.mixDrive(forward, strafe, rotate);

            //Front servo control
            if (gamepad1.left_bumper && !mat){
                if (robot.MatServos.getPosition() == 0) robot.MatServos.setPosition((float) 9 / 28);
                else robot.MatServos.setPosition(0);
                mat = true;
            } else if(!gamepad1.left_bumper) mat = false;
            robot.grabMat(gamepad1.right_trigger * ((float)9 / 28));

            //SideArm control
            if(gamepad1.a && !sideArm) {
                if(robot.SideArm.getPosition() == 0) robot.SideArm.setPosition((float) 110 / 180);
                else robot.SideArm.setPosition(0);
                sideArm = true;
            } else if(!gamepad1.a) sideArm = false;



            //Gamepad 2
            robot.armLift.setPower(gamepad2.left_stick_y);

            if(gamepad2.right_trigger > 0.7 && !claw) {
                if(robot.BlockGrips.getPosition() == 0) robot.controlClaw(30);
                else robot.controlClaw(0);
                claw = true;
            } else if(gamepad2.right_trigger < 0.7) claw = false;

        }
        }

    }
