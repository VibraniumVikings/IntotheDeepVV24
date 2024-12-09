/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Concept;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * OpMode for drivetrain diagnostics
 *
 */

@TeleOp(name="testDrive", group="teleOp")
@Disabled
public class testDrive extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightRear;
    public DcMotor leftRear;
    public DcMotor leftArm;
    public DcMotor rightArm;
    public DcMotor pickUp;
    public CRServo rightWheel;
    public CRServo leftWheel;
    public Servo drone;
    public double RWPower;
    public double LWPower;
    IMU imu;

    public void runOpMode() {
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        int x = 0;
        int y = 0;
        double RWPower = 0;
        double LWPower = 0;
        double RWPowerPU = 0;
        double LWPowerPU = 0;
        double drivePower = 0.5; //global drive power level
        double armPower = 0;
        double armEPower = 0;
        double pickUpPwr = 0;
        double droneSet = 0.25;
        double droneLaunch = 0;
        double servpos = 0;

// Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFront = hardwareMap.get(DcMotor.class, "FLM");
        rightFront = hardwareMap.get(DcMotor.class, "FRM");
        rightRear = hardwareMap.get(DcMotor.class, "RRM");
        leftRear = hardwareMap.get(DcMotor.class, "RLM");
        leftArm = hardwareMap.get(DcMotor.class, "armL");
        rightArm = hardwareMap.get(DcMotor.class, "armR");
        pickUp = hardwareMap.get(DcMotor.class, "pickUp");
        imu = hardwareMap.get(IMU.class, "imu");

        // Define Servos
        rightWheel = hardwareMap.crservo.get("RSW");
        leftWheel = hardwareMap.crservo.get("LSW");
        drone = hardwareMap.get(Servo.class,"drone");

        drone.scaleRange(0,1);
        drone.setDirection(Servo.Direction.REVERSE);
        drone.setPosition(droneSet);

        //define and initialize sensors
        //colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "CLR");

        // Adjust the orientation parameters to match your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //distFront = myOpMode.hardwareMap.get(DistanceSensor.class, "FDS");
        //distRear = myOpMode.hardwareMap.get(DistanceSensor.class, "RDS");

        //Set the motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pickUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pickUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData(">", "Hardware Initialized");
        telemetry.addData("Drone Servo", drone.getPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            while (opModeIsActive()) {
                //double driveY = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                //double strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                //double turn = gamepad1.right_stick_x;

                //driveY = -gamepad1.left_stick_y;
                //strafe = gamepad1.left_stick_x * 1.1;
                //turn = gamepad1.right_stick_x;
                LWPower = gamepad2.right_stick_y;
                RWPower = gamepad2.right_stick_x;
                armPower = -gamepad2.left_stick_y;
                pickUpPwr = -gamepad2.left_stick_x * 0.5;

                y = rightFront.getCurrentPosition();
                x = -leftFront.getCurrentPosition(); //parallel, forward encoder distance is 0

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                //double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe) + Math.abs(turn), 1);
                //double frontLeftPower = (driveY + strafe + turn) / denominator;
                //double backLeftPower = (driveY - strafe + turn) / denominator;
                //double frontRightPower = (driveY - strafe - turn) / denominator;
                //double backRightPower = (driveY + strafe - turn) / denominator;
                //Testing purposes
                double frontLeftPower = -gamepad1.left_stick_y;
                double backLeftPower = gamepad1.left_stick_x;
                double frontRightPower = -gamepad1.right_stick_y;
                double backRightPower = gamepad1.right_stick_x;
                // Remember that the right is opposite of the left
                leftFront.setPower(frontLeftPower);
                leftRear.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightRear.setPower(backRightPower);

                leftArm.setPower(armPower);
                rightArm.setPower(armPower);

                pickUp.setPower(pickUpPwr);

                leftWheel.setPower(LWPower);
                rightWheel.setPower(RWPower);

                // Retrieve Rotational Angles and Velocities
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                // Adding telemetry readouts
                telemetry.addData(">", "Robot Running");
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("FLM",frontLeftPower);
                telemetry.addData("RLM",backLeftPower);
                telemetry.addData("FRM",frontRightPower);
                telemetry.addData("RRM",backRightPower);
                telemetry.addData("Y Encoder",y);
                telemetry.addData("X Encoder",x);
                telemetry.addData("PickUp Position", pickUp.getCurrentPosition());
                telemetry.addData("Arm Power", armPower);
                telemetry.addData("Arm Position", rightArm.getCurrentPosition());
                telemetry.addData("Arm Target", rightArm.getTargetPosition());
                telemetry.addData("Drone", servpos);
                telemetry.update();
            }
        }
    }
}