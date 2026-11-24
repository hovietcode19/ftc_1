package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    private final IMU imu;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    public Drivetrain(HardwareMap hardwareMap) {

        frontLeft  = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT);
        backRight  = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        imu = hardwareMap.get(IMU.class, Constants.IMU_NAME);
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(params);
    }
    // getting angle :D
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    // reset imu :D
    public void resetHeading() {
        imu.resetYaw();
    }
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // set chế độ run with encoder
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // set run mode
    public void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
    public void driveFieldCentric(double fieldForward, double fieldStrafe, double turn) {
        double h = getHeading();
        // returning to the back angle (transform robot's vector to field's vector) :D
        double cos = Math.cos(-h);
        double sin = Math.sin(-h);

        double strafe  = fieldStrafe * cos - fieldForward * sin;
        double forward = fieldStrafe * sin + fieldForward * cos;
        // filter by constant
        strafe *= Constants.STRAFE_CORRECTION;

        drive(forward, strafe, turn);
    }

    public void drive(double forward, double strafe, double turn) {

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(
                1.0,
                Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);


    }
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("FL (Power | Pos)", "%.2f | %d", frontLeft.getPower(), frontLeft.getCurrentPosition());
        telemetry.addData("FR (Power | Pos)", "%.2f | %d", frontRight.getPower(), frontRight.getCurrentPosition());
        telemetry.addData("BL (Power | Pos)", "%.2f | %d", backLeft.getPower(), backLeft.getCurrentPosition());
        telemetry.addData("BR (Power | Pos)", "%.2f | %d", backRight.getPower(), backRight.getCurrentPosition());
    }
    public void stop() {
        drive(0, 0, 0);
    }
}