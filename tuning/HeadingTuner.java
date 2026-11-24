package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.HeadingController;

@TeleOp(name = "Heading Tuner", group = "Tuning")
public class HeadingTuner extends LinearOpMode {

    private boolean prevUp, prevDown, prevLeft, prevRight, prevLb, prevRb;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        HeadingController pid = new HeadingController();
        ElapsedTime stepTimer = new ElapsedTime();

        double startError = 0, maxOvershoot = 0, settleTime = -1;

        // check for status in init
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Xoay robot sang TRÁI bằng tay: heading phải TĂNG");
            telemetry.addData("Heading", Math.toDegrees(robot.drivetrain.getHeading()));
            telemetry.update();
        }

        waitForStart();
        robot.drivetrain.resetHeading();
        pid.setTarget(0);

        while (opModeIsActive()) {
            double heading = Math.toDegrees(robot.drivetrain.getHeading());

            // set target
            boolean changed = false;
            double t = 0;
            if (gamepad1.a)         { t = 0;    changed = true; }
            else if (gamepad1.b)    { t = -90;  changed = true; }
            else if (gamepad1.x)    { t = 90;   changed = true; }
            else if (gamepad1.y)    { t = 170;  changed = true; }
            else if (gamepad1.back) { t = -170; changed = true; }

            if (changed && t != pid.getTarget()) {
                pid.setTarget(t);
                startError = pid.getError(heading);
                maxOvershoot = 0;
                settleTime = -1;
                stepTimer.reset();
            }

            // recalibrate constants
            if (gamepad1.dpad_up && !prevUp)       pid.kP += 0.002;
            if (gamepad1.dpad_down && !prevDown)   pid.kP = Math.max(0, pid.kP - 0.002);
            if (gamepad1.dpad_right && !prevRight) pid.kD += 0.0002;
            if (gamepad1.dpad_left && !prevLeft)   pid.kD = Math.max(0, pid.kD - 0.0002);
            if (gamepad1.right_bumper && !prevRb)  pid.kS += 0.01;
            if (gamepad1.left_bumper && !prevLb)   pid.kS = Math.max(0, pid.kS - 0.01);
            prevUp = gamepad1.dpad_up;     prevDown = gamepad1.dpad_down;
            prevLeft = gamepad1.dpad_left; prevRight = gamepad1.dpad_right;
            prevLb = gamepad1.left_bumper; prevRb = gamepad1.right_bumper;

            // operation
            double turn = pid.update(heading);
            robot.drivetrain.drive(0, 0, turn);

            // assessment
            double error = pid.getError(heading);
            // deviation from target
            double beyond = -Math.signum(startError) * error;
            if (beyond > maxOvershoot)
                maxOvershoot = beyond;
            if (pid.isSettled(heading) && settleTime < 0)
                settleTime = stepTimer.seconds();
            if (Math.abs(error) > 2 * Constants.HEADING_TOLERANCE_DEG)
                settleTime = -1;

            telemetry.addData("kP", pid.kP);
            telemetry.addData("kD", pid.kD);
            telemetry.addData("kS", pid.kS);
            telemetry.addData("Mục tiêu (độ)", pid.getTarget());
            telemetry.addData("Heading (độ)", heading);
            telemetry.addData("Lỗi (độ)", error);
            telemetry.addData("Vận tốc xoay (độ/s)", pid.getRate());
            telemetry.addData("Turn ra", turn);
            telemetry.addData("Vọt lố tối đa (độ)", maxOvershoot);
            telemetry.addData("Thời gian ổn định (s)", settleTime < 0 ? "chưa" : settleTime);
            telemetry.update();
        }

        robot.stopAll();
    }
}