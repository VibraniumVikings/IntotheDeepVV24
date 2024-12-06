package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Core.vvHardwareITD;

/**
 * ITD (into the deep) teleOp for test and shakedown
 *
 * Need to confirm the arm, extender, wrist, and claw to have a full test case
 * Also need the telemetry to read all sensor values
 */

@TeleOp(name="vvArmLift", group="2-TeleOp")
@Disabled
public class vvArmLift extends LinearOpMode {

    //vvHardware class external pull
    vvHardwareITD   robot       = new vvHardwareITD(this);

    @Override
    public void runOpMode() throws InterruptedException {
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        int x = 0;
        int y = 0;
        double drivePower = 0.5; //global drive power level
        double armBump = 0;
        double extBump = 0;
        double armPower = 0;
        double extPower = 0;
        double liftPower = 0;
        int armBumpInc = 50;
        int extBumpInc = 100;
        int extLoc = 0;
        double wristPos = 0;
        double clawPos = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            while (opModeIsActive()) {
                driveY = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x * 1;
                turn = gamepad1.right_stick_x;
                //armBump = -gamepad2.left_stick_y;
                //extBump = -gamepad2.right_stick_y;
                armPower = -gamepad2.left_stick_y;
                extPower = -gamepad2.left_stick_x;
                liftPower = -gamepad2.right_stick_y;

                y = robot.parallelEncoder.getCurrentPosition();
                x = robot.perpendicularEncoder.getCurrentPosition(); //parallel, forward encoder distance is 0

                if (gamepad1.right_bumper) {
                    // button is transitioning to a pressed state. So increment drivePower by 0.1
                    drivePower = Math.min(drivePower + 0.05,0.9);
                }
                else if (gamepad1.left_bumper) {
                    // button is transitioning to a pressed state. So increment drivePower by -0.1
                    drivePower = Math.max(drivePower - 0.05, 0.1);
                }

                if (gamepad1.dpad_left) { //arm ascent grab
                    robot.extArmPos(robot.extArmAscentGrab,robot.extArmEPower);
                    robot.armPos(robot.armAscent, robot.armEPower);
                    robot.moveWristCarry();
                }
                if (gamepad1.dpad_right) { //arm ascent lift
                    robot.extArmPos(robot.extArmAscentLift,robot.extArmEPower+0.3);
                    robot.armPos(robot.armAscent, robot.armEPower);
                    robot.moveWristCarry();
                }
                if (gamepad1.dpad_up) { //lift to grab position
                    robot.liftUp();
                }
                if (gamepad1.dpad_down) { //lift to down position, for robot lift
                    robot.liftDown();
                }
                if (gamepad1.x) { //wrist drop
                    robot.moveWristFloor();
                }
                if (gamepad1.b) { //floor pick
                    robot.extArmPos(robot.extArmFLoorPick,robot.extArmEPower+0.3);
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristCarry();
                }
                if (gamepad1.options) { //reset
                    robot.armPos(0, robot.armEPower);
                    robot.extArmPos(0,robot.extArmEPower);
                    robot.moveWristCarry();
                }

                // Methods called for motion
                robot.driveRobotFC(drivePower, driveY, strafe, turn);

                robot.moveArm(armPower);
                robot.moveExt(extPower);
                robot.moveLift(liftPower);

                wristPos = robot.wrist.getPosition();
                clawPos = robot.claw.getPosition();

                if (gamepad2.right_bumper)
                    robot.openClaw();
                if (gamepad2.left_bumper)
                    robot.closeClaw();
                if (gamepad2.x)
                    robot.longClaw();

                if (gamepad2.a) { //Near floor pick
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick,robot.extArmEPower);
                    robot.moveWristFloor();
                }
                if (gamepad2.b) { //Wall pick
                    robot.armPos(robot.armWall, robot.armEPower);
                    robot.extArmPos(robot.extArmLowCe,robot.extArmEPower);
                    robot.moveWristWall();
                }

                if (gamepad2.y) { //Submersible pick
                    robot.armPos(robot.armFloorSub, robot.armEPower);
                    robot.moveWristFloor();
                    robot.extArmPos(robot.extArmFloorSub, robot.extArmEPower);
                }

                if (gamepad2.dpad_up) { //High Basket
                    robot.armPos(robot.armHighBa, robot.armEPower);
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                    robot.moveWristHighBw();
                }
                if (gamepad2.dpad_right) { //Low Basket
                    robot.armPos(robot.armLowBa, robot.armEPower);
                    robot.extArmPos(robot.extArmLowBe, robot.extArmEPower);
                    robot.moveWristLowBw();
                }
                if (gamepad2.dpad_left) { //High Chamber
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                    robot.moveWristHighCw();
                }
                if (gamepad2.dpad_down) { //Low Chamber
                    robot.armPos(robot.armLowCa, robot.armEPower);
                    robot.extArmPos(robot.extArmLowCe, robot.extArmEPower);
                    robot.moveWristLowCW();
                }

                if (gamepad2.options) { //backward basket drop
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.extArmPos(robot.extArmHighBe,robot.extArmEPower);
                    robot.moveWristHighBw();
                }

                if (armBump >0.8) {
                    robot.armPos(robot.arm.getCurrentPosition()+armBumpInc,robot.armEPower);
                }
                if (armBump <-0.8) {
                    robot.armPos(robot.arm.getCurrentPosition()-armBumpInc,robot.armEPower);
                }
                if (extBump>0.8) {
                    robot.extArmPos(robot.arm.getCurrentPosition()+extBumpInc,robot.extArmEPower);
                }
                if (extBump<-0.8) {
                    robot.extArmPos(robot.arm.getCurrentPosition()-extBumpInc,robot.extArmEPower);
                }

                // Retrieve Rotational Angles and Velocities
                YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);

// Adding telemetry readouts
                telemetry.addData(">", "Robot Running");
                telemetry.addData("Drive Power", drivePower);
                //telemetry.addData("Y", driveY);
                //telemetry.addData("strafe", strafe);
                //telemetry.addData("turn", turn);
                telemetry.addData("Y Encoder",y);
                telemetry.addData("X Encoder",x);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
                telemetry.addData("Extend Position", robot.extend.getCurrentPosition());
                telemetry.addData("Left Lift Position", robot.leftLift.getCurrentPosition());
                telemetry.addData("Right Lift Position", robot.rightLift.getCurrentPosition());
                telemetry.addData("Wrist", wristPos);
                telemetry.addData("Claw", clawPos);
                
                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);

            }
        }
    }
}

