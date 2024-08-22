package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.Objects;


public class Drive {

    private final DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    // TODO : НЕ КОЛХОЗИТЬ
    private final LocalizerKolhoz localizer;


    public Drive(HardwareMap hardwareMap, Pose2d startPose) {
        // Motors
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Localizer
        localizer = new LocalizerKolhoz(hardwareMap);
        localizer.setPoseEstimate(startPose);
    }

    public void setMotorPowers(double lf, double lb, double rf, double rb) {
        leftFrontDrive.setPower(lf);
        leftBackDrive.setPower(lb);
        rightFrontDrive.setPower(rf);
        rightBackDrive.setPower(rb);
    }

    public void setMotorPowers(double[] motorPowers) {
        leftFrontDrive.setPower(motorPowers[0]);
        leftBackDrive.setPower(motorPowers[1]);
        rightFrontDrive.setPower(motorPowers[2]);
        rightBackDrive.setPower(motorPowers[3]);

    }

    public double[] setPowersByPose(Pose2d pose) {
        double axial = pose.getX();
        double lateral = -pose.getY();
        double yaw = pose.getHeading();

        double heading = localizer.getPoseEstimate().getHeading();

        double rotLateral = lateral * Math.cos(-heading) - axial * Math.sin(-heading);
        double rotAxial   = lateral * Math.sin(-heading) + axial * Math.cos(-heading);

        rotLateral = rotLateral * 1.1;

        double leftFrontPower  = rotAxial + rotLateral + yaw;
        double rightFrontPower = rotAxial - rotLateral - yaw;
        double leftBackPower   = rotAxial - rotLateral + yaw;
        double rightBackPower  = rotAxial + rotLateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        setMotorPowers(leftFrontPower,
                leftBackPower,
                rightFrontPower,
                rightBackPower);
        return new double[] {leftFrontPower, leftBackPower,
        rightFrontPower, rightBackPower};
    }

    public void updateLocalizer() {
        localizer.update();
    }

    public Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

    public double[] getMotorPowers() {
        return new double[] {leftFrontDrive.getPower(), leftBackDrive.getPower(),
        rightFrontDrive.getPower(), rightBackDrive.getPower()};
    }

    public List<Double> getWheelVelocities() {
        return localizer.getWheelVelocities();
    }

    public List<Double> getWheelPositions() {
        return localizer.getWheelPositions();
    }

    public double getParralelVelocity() {
        return Objects.requireNonNull(localizer.getWheelVelocities()).get(1);
    }

    public double getPerpendicularVelocity() {
        return Objects.requireNonNull(localizer.getWheelVelocities()).get(3);
    }

    public double getVelocityInTicks() {
        List<Double> wheelVelocities = localizer.getWheelVelocities();
        assert wheelVelocities != null;
        return Math.sqrt(wheelVelocities.get(1) * wheelVelocities.get(1) +
                wheelVelocities.get(3) * wheelVelocities.get(3));
    }
}
