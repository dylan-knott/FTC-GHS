package org.firstinspires.ftc.teamcode.Demo_OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

/**-----------Hardware Map-------------
 Motors:
 back_right_motor
 front_right_motor
 front_left_motor
 back_left_motor
 Servos:
 back_servo
 Color Sensors:
 color_sensor
 BNO055IMU Sensors:
 imu
 Distance Sensors:
 left_distance
 right_distance
 front_distance
 back_distance
 **/

@Autonomous(name = "TestDrive", group = "Mechanum")
public class TestDrive extends LinearOpMode {

    public DcMotor back_right = hardwareMap.dcMotor.get("back_right_motor");
    public DcMotor back_left = hardwareMap.dcMotor.get("back_left_motor");
    public DcMotor front_right = hardwareMap.dcMotor.get("front_right_motor");
    public DcMotor front_left = hardwareMap.dcMotor.get("front_left_motor");
    private BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    public DistanceSensor back_dist = hardwareMap.get(DistanceSensor.class, "back_distance");
    public ColorSensor color_sensor = hardwareMap.colorSensor.get("color_sensor");
    public Servo back_servo = hardwareMap.servo.get("back_servo");


    public void runOpMode() {
        //Code to run during initialization of robot
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        //Code to run after the start button is pressed

        back_right.setPower(1);
        back_left.setPower(1);
        front_left.setPower(1);
        front_right.setPower(1);
        sleep(1000);
        back_right.setPower(0);
        back_left.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);


    }


}
