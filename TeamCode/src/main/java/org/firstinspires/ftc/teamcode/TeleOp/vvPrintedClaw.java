package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Core.vvHardwareITDClaw;

/**
 * ITD (into the deep) teleOp with Printed Claw (sweeper)
 *
 */

@TeleOp(name="vvPrintedClaw", group="1-TeleOp")

public class vvPrintedClaw extends LinearOpMode {

    //vvHardware class external pull
    vvHardwareITDClaw   robot       = new vvHardwareITDClaw(this);
    //vvLimeColor limelight = new vvLimeColor(this);

    @Override
    public void runOpMode() throws InterruptedException {
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        int x = 0;
        int y = 0;
        double drivePower = 0.7; //global drive power level
        double armBump = 0;
        double extBump = 0;
        int armBumpInc = 50;
        int extBumpInc = 100;
        int armLoc = 0;
        int extLoc = 0;
        double wristPos = 0;
        double clawPos = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        //limelight.init();

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
                armBump = -gamepad2.left_stick_y;
                extBump = -gamepad2.right_stick_y;

                y = robot.parallelEncoder.getCurrentPosition();
                x = robot.perpendicularEncoder.getCurrentPosition(); //parallel, forward encoder distance is 0

                armLoc = robot.arm.getCurrentPosition();
                extLoc = robot.extend.getCurrentPosition();

                if (gamepad1.right_bumper) {
                    // button is transitioning to a pressed state. So increment drivePower by 0.1
                    drivePower = Math.min(drivePower + 0.05,0.95);
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
                    robot.moveWristFloor();
                }
                if (gamepad1.dpad_up) { //lift to grab position
                    robot.liftUp();
                    //robot.armPos(robot.armLowCa, robot.armEPower);
                    //robot.extArmPos(robot.extArmLowCe, robot.extArmEPower);
                    //robot.moveWristLowCW();
                }
                if (gamepad1.dpad_down) { //lift to down position, for robot lift
                    robot.liftDown();
                    //robot.armPos(robot.floorArm, robot.armEPower);
                    //robot.extArmPos(robot.extArmFLoorPick,robot.extArmEPower);
                    //robot.moveWristFloor();
                }
                if (gamepad1.x) { //wrist drop
                    robot.moveWristFloor();
                }
                if (gamepad1.b) { //carry
                    robot.extArmPos(robot.extArmFLoorPick,robot.extArmEPower);
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristCarry();
                }
                if (gamepad1.options) { //reset
                    robot.armPos(0, robot.armEPower);
                    robot.extArmPos(0,robot.extArmEPower);
                    robot.moveWristCarry();
                }

                // Methods called for motion
                robot.driveRobot(drivePower, driveY, strafe, turn);

                //robot.moveArm(armPower);
                //robot.moveExt(extPower);

                wristPos = robot.wrist.getPosition();
                clawPos = robot.claw.getPosition();

                if (gamepad2.right_bumper)
                    robot.openClaw();

                if (gamepad2.left_bumper)
                    robot.closeClaw();

                /*if (gamepad2.x) { //Rear high basket drop
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                    robot.moveWristLowCW();
                }*/
                if (gamepad2.x) { //Chamber clip
                    robot.armPos(robot.armHighCa-150, robot.armEPower);
                    sleep(350);
                    robot.openClaw();
                }
                if (gamepad2.y) { //Carry position
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(50, robot.extArmEPower);
                    robot.moveWristCarry();
                }
                if (gamepad2.a) { //intermediate pick
                    robot.armPos(robot.armFloorInt, robot.armEPower);
                    robot.moveWristLowBw();
                    robot.extArmPos(robot.extArmFloorInt, robot.extArmEPower);
                }
                if (gamepad2.b) { //Wall pick
                    robot.armPos(robot.armWall, robot.armEPower);
                    robot.extArmPos(robot.extArmLowCe,robot.extArmEPower);
                    robot.moveWristWall();
                }
                if (gamepad2.dpad_down) { //Near floor pick
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick,robot.extArmEPower);
                    robot.moveWristLowBw();
                }
                if (gamepad2.dpad_right) { //Submersible pick
                    robot.armPos(robot.armFloorSub, robot.armEPower);
                    robot.moveWristLowBw();
                    robot.extArmPos(robot.extArmFloorSub, robot.extArmEPower);
                }

                if (gamepad2.dpad_up) { //High Basket
                    robot.armPos(robot.armHighBa, robot.armEPower);
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                    robot.moveWristHighBw();
                }
                /*if (gamepad2.dpad_right) { //Low Basket
                    robot.armPos(robot.armLowBa, robot.armEPower);
                    robot.extArmPos(robot.extArmLowBe, robot.extArmEPower);
                    robot.moveWristLowBw();
                }*/
                if (gamepad2.dpad_left) { //High Chamber
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                    robot.moveWristHighCw();
                }
                /*if (gamepad2.dpad_down) { //Low Chamber
                    robot.armPos(robot.armLowCa, robot.armEPower);
                    robot.extArmPos(robot.extArmLowCe, robot.extArmEPower);
                    robot.moveWristLowCW();
                }*/

                if (gamepad2.start) { //arm extension reset
                    robot.extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(20);
                    robot.extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.closeClaw();
                    robot.armPos(0, robot.armEPower);
                    robot.extArmPos(0,robot.extArmEPower);
                    robot.moveWristCarry();
                }

                if (armBump>0.8) {
                    robot.armPos(armLoc+armBumpInc,robot.armEPower);
                }
                if (armBump<-0.8) {
                    robot.armPos(armLoc-armBumpInc,robot.armEPower);
                }
                if (extBump>0.8) {
                    robot.extArmPos(extLoc+extBumpInc,robot.extArmEPower);
                }
                if (extBump<-0.8) {
                    robot.extArmPos(extLoc-extBumpInc,robot.extArmEPower);
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
                telemetry.addData("Y Encoder",((0.944882*2*Math.PI*y)/2000));
                telemetry.addData("X Encoder",((0.944882*2*Math.PI*x)/2000));
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

