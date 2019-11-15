package org.firstinspires.ftc.teamcode.ftc17223;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "FindCenter")
public class FindCenterOP extends LinearOpMode {
    RobotDrive robotDrive = new RobotDrive();

    public void runOpMode() {
        //Initialization Code
        robotDrive.initializeRobot(hardwareMap);


        //Runs once when start is pressed
        waitForStart();
        //Runs continuously after start is pressed
        while (opModeIsActive()) {
            //Not quite working, distance sensors don't reach all the way to the outside of the arena
            double maxPower = robotDrive.motorPower;
            double forward = RobotDrive.clamp(robotDrive.distf.getDistance(DistanceUnit.METER) - robotDrive.distb.getDistance(DistanceUnit.METER), -maxPower, maxPower);
            double strafe = RobotDrive.clamp(robotDrive.distr.getDistance(DistanceUnit.METER) - robotDrive.distl.getDistance(DistanceUnit.METER), -maxPower, maxPower);
            robotDrive.mixDrive(forward, strafe, 0);
            telemetry.addData("Front: ", robotDrive.distf.getDistance(DistanceUnit.METER));
            telemetry.addData("Back: ", robotDrive.distb.getDistance(DistanceUnit.METER));
            telemetry.addData("Left: ", robotDrive.distl.getDistance(DistanceUnit.METER));
            telemetry.addData("Right: ", robotDrive.distr.getDistance(DistanceUnit.METER));
            telemetry.update();
        }
    }
}
