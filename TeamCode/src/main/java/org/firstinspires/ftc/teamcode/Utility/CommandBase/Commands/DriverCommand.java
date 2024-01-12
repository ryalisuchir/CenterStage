package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DriverCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier leftY, leftX, rightX;

    public DriverCommand(DriveSubsystem drive, DoubleSupplier leftY,
                               DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble());
    }

}