package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;

@TeleOp
public class Main extends OpMode{

    MecanumDrivetrain drivetrain = new MecanumDrivetrain();

    double forwards, strafe, rotation;

    boolean resetYaw;

    @Override
    public void init(){
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop(){
        forwards = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotation = gamepad1.right_stick_x;

        resetYaw = gamepad1.left_bumper;

        if(resetYaw) {
            drivetrain.resetYaw();
        }

        drivetrain.FieldRelative(forwards, strafe, rotation);

    }
}
