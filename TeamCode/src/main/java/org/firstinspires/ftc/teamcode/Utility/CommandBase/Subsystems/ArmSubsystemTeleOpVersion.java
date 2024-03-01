package org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.Hardware.CustomPIDController;
import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;

@Config
public class ArmSubsystemTeleOpVersion extends SubsystemBase {
    public final DcMotorEx arm;



    public ArmSubsystemTeleOpVersion(DcMotorEx a) {
        arm = a;
    }

    public void loop() {

        if (Globals.IS_AT_REST) {

        };

        if (Globals.IS_SCORING) {

        };

        if (Globals.IS_INTAKING) {

        };

        if ((arm.getPower() > 0 && arm.getCurrentPosition() > 430 ) || (arm.getCurrentPosition() < 20 && arm.getPower() < 0)) {
            arm.setPower(0);
        }
        if (arm.getPower() < 0 && arm.getCurrentPosition() < 230) {
            arm.setPower(0.0006);
        }
        if (arm.getPower() > 0 && arm.getCurrentPosition() > 350) {
            arm.setPower(0.33);
        }






    }

    public void setPosition(int power) {
        arm.setPower(power);
    }
    public void armOuttakeTeleOp() {
        arm.setPower(.5);
    }
    public void armCoastTeleOp() {
        arm.setPower(-.43);
    }

}