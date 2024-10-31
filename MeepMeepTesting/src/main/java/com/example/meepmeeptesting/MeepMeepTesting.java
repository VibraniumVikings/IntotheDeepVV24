package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-12, -65, Math.toRadians(90)))
                                .forward(40)
                                .strafeLeft(6)
                                .turn(Math.toRadians(160))
                                .forward(24)
                                .back(8)
                                .turn(Math.toRadians(160))
                                .forward(16)
                                .turn(Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(48,-51),Math.toRadians(0))
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\Adambots 1\\Documents\\field-2024-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .addEntity(myBot)
                .start();

        /*meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start(); */
    }
}
/* High Basket Set 29Oct
TrajectorySequence fwdHighCmbr = vvdrive.trajectorySequenceBuilder(startPose) //Tile Start Position
                .forward(27)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow1 = vvdrive.trajectorySequenceBuilder(fwdHighCmbr.end())
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.extArmPos(0, robot.armEPower))
                .strafeLeft(65)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristFloor();
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                })
                .waitSeconds(0)
                .forward(8)
                .build();
        TrajectorySequence yellow1Drop = vvdrive.trajectorySequenceBuilder(yellow1.end())
                .back(18)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                })
                .turn(Math.toRadians(-60))
                .waitSeconds(0)
                .build();
        TrajectorySequence ascentPark = vvdrive.trajectorySequenceBuilder(yellow1Drop.end())
                .forward(12)
                .turn(Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(0, robot.armEPower);
                    robot.moveWristCarry();
                    robot.extArmPos(robot.extArmFLoorPick, robot.armEPower);
                })
                .forward(24)
                .strafeRight(16)
                .waitSeconds(0)
                .build();


 */