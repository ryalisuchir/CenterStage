package org.firstinspires.ftc.teamcode.Drive.OpModes.States.AdvancedAutonomous60;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.New.RRBRTeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Photon
public class CameraTest extends OpMode {
    private VisionPortal visionPortal;
    private RRBRTeamPropProcessor colorMassDetectionProcessor;

    @Override
    public void init() {

        colorMassDetectionProcessor = new RRBRTeamPropProcessor();

        colorMassDetectionProcessor.setDetectionColor(false); //false is blue, true is red

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();

    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position: ", colorMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State: ", visionPortal.getCameraState());
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
       //do nothing
    }

    @Override
    public void loop() {
        telemetry.update();
    }

    @Override
    public void stop() {
        visionPortal.close();
        telemetry.addLine("Closed Camera.");
        telemetry.update();
    }
}