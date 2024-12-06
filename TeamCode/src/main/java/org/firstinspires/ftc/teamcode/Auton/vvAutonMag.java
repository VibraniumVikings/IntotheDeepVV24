/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.vvHardwareMag;
import org.firstinspires.ftc.vision.VisionPortal;


import java.util.List;

/*
 * This OpMode is the basis for our speed test comparing Magnus (312 RPM) chassis to Indra
 * Use the Dpad to determine the test case (Forward, Left & Spin, Right & Spin)
 */
@Autonomous(name = "AutonMagnus", group = "Concept")
@Disabled
public class vvAutonMag extends LinearOpMode {

    vvHardwareMag robot = new vvHardwareMag(this);

    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    public void runOpMode() {

        //int encX = 0;
        //int encY = 0;
        double x = 0;
        double heading = 0;

        robot.init();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                    autonDriveFwd();

                    autonDriveLeft();

                    autonDriveBwd();

                    autonDriveRight();

                //encY = -robot.parallelEncoder.getCurrentPosition();
                //encX = robot.perpendicularEncoder.getCurrentPosition(); //parallel, forward encoder distance is 0

                //telemetry.addData("Y Encoder",y);
                //telemetry.addData("X Encoder",x);

                // Push telemetry to the Driver Station.
                telemetry.update();

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 30.0)) {
                    telemetry.addData("Path", ": %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();

                }


            }
        }
    }   // end runOpMode()

    private void autonDriveFwd() {
        // Wait for the game to start (driver presses PLAY)
        while (opModeIsActive()) {

            // Step 1:  Drive forward for 3 seconds

            robot.driveRobot(1, FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Stop
            robot.driveRobot(0, 0, 0, 0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }
    private void autonDriveLeft() {
        // Wait for the game to start (driver presses PLAY)
        while (opModeIsActive()) {

            // Step 1:  Drive left for 3 seconds

            robot.driveRobot(1, 0, -FORWARD_SPEED, 0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Spin left for 1 seconds

            //robot.driveRobot(0, 0, 0, -TURN_SPEED);

            //runtime.reset();
            //while (opModeIsActive() && (runtime.seconds() < 1)) {
                //telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                //telemetry.update();
            }

            // Step 4:  Stop
            robot.driveRobot(0, 0, 0, 0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }

    private void autonDriveRight() {
        // Wait for the game to start (driver presses PLAY)
        while (opModeIsActive()) {

            // Step 1:  Drive left for 3 seconds

            robot.driveRobot(1, 0, FORWARD_SPEED, 0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Spin left for 1 seconds

            //robot.driveRobot(0, 0, 0, TURN_SPEED);

            //runtime.reset();
            //while (opModeIsActive() && (runtime.seconds() < 1)) {
                //telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                //telemetry.update();
            }

            // Step 4:  Stop
            robot.driveRobot(0, 0, 0, 0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }

    private void autonDriveBwd() {
        // Wait for the game to start (driver presses PLAY)
        while (opModeIsActive()) {

            // Step 1:  Drive forward for 3 seconds

            robot.driveRobot(1, -FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Stop
            robot.driveRobot(0, 0, 0, 0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }
}// end class
