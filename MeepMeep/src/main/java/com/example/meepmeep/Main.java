package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.awt.Color;
import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class Main {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(60), Math.toRadians(60), 14.07)
                .setDimensions(15, 15)
                .setColorScheme(new ColorScheme() {
                    @Override
                    public boolean isDark() {
                        return true;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return Color.BLACK;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return Color.BLUE;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return Color.BLUE;
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return Color.RED;
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return Color.RED;
                    }

                    @Override
                    public double getAXIS_NORMAL_OPACITY() {
                        return 0;
                    }

                    @Override
                    public double getAXIS_HOVER_OPACITY() {
                        return 0;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJCETORY_PATH_COLOR() {
                        return Color.BLUE;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return Color.RED;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return Color.RED;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return Color.GRAY;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return Color.RED;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return Color.BLACK;
                    }

                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return Color.BLACK;
                    }
                })
                .followTrajectorySequence(drive ->
               drive.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                       .splineToLinearHeading(new Pose2d(9, -40, Math.toRadians(120)), Math.toRadians(120))
                       .setReversed(true)
                       .splineToLinearHeading(new Pose2d(49.35, -35.82, Math.toRadians(0.00)), Math.toRadians(0.00))
                       .splineToLinearHeading(new Pose2d(26.87, -13, Math.toRadians(0)), Math.toRadians(180))
                       .waitSeconds(0.5)
                       .splineToLinearHeading(new Pose2d(-54, -13, Math.toRadians(0)), Math.toRadians(180))
                       .splineToLinearHeading(new Pose2d(-57, -13, Math.toRadians(0)), Math.toRadians(180))
                       .splineToLinearHeading(new Pose2d(31, -13, Math.toRadians(0)), Math.toRadians(0))
                       .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                       .splineToLinearHeading(new Pose2d(50, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                       .build()


                );



        Image img = null;
        try { img = ImageIO.read(new File("/Users/suchir/Documents/field.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.75f)
                .addEntity(myBot)
                .start();
    }
}