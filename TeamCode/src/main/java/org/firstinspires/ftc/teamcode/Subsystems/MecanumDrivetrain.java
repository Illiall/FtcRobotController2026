package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrivetrain {

    DcMotor FrontLeftMotor;
    DcMotor FrontRightMotor;
    DcMotor BackLeftMotor;
    DcMotor BackRightMotor;

    double FLpower;
    double FRpower;
    double BLpower;
    double BRpower;

    double theta;
    double distance;
    double newForwards;
    double newStrafe;

    IMU imu;

    ImuOrientationOnRobot Orientation;
    public void init(HardwareMap hardwareMap){
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BR");

        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        Orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            );

        imu.initialize(new IMU.Parameters(Orientation));
    }
    public double getYawRadians(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetYaw(){
        imu.resetYaw();
    }

    public void Drive(double forwards, double strafe, double rotation){
        FLpower = forwards + strafe + rotation;
        FRpower = forwards - strafe - rotation;
        BLpower = forwards - strafe + rotation;
        BRpower = forwards + strafe - rotation;

        double maxPower = Constants.Drivetrain.MaxPower;
        double maxSpeed = Constants.Drivetrain.MaxSpeed;

        maxPower = Math.max(maxPower, Math.abs(FLpower));
        maxPower = Math.max(maxPower, Math.abs(FRpower));
        maxPower = Math.max(maxPower, Math.abs(BLpower));
        maxPower = Math.max(maxPower, Math.abs(BRpower));

        FrontLeftMotor.setPower(maxSpeed * (FLpower / maxPower));
        FrontRightMotor.setPower(maxSpeed * (FRpower / maxPower));
        BackLeftMotor.setPower(maxSpeed * (BLpower / maxPower));
        BackRightMotor.setPower(maxSpeed * (BRpower / maxPower));
    }

    public void FieldRelative(double forwards, double strafe, double rotation){
        theta = Math.atan2(forwards, strafe);

        distance = Math.hypot(strafe, forwards);

        theta = AngleUnit.normalizeRadians(getYawRadians());

        newForwards = distance * Math.sin(theta);
        newStrafe = distance * Math.cos(theta);

        Drive(newForwards, newStrafe, rotation);
    }

}
