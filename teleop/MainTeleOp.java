package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.control.HeadingController;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();
        HeadingController heading = new HeadingController();
        ElapsedTime loopTimer = new ElapsedTime();
        heading.setTarget(Math.toDegrees(robot.drivetrain.getHeading()));
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = Math.min(loopTimer.seconds(), 0.1);
            loopTimer.reset();
            double headingDeg = Math.toDegrees(robot.drivetrain.getHeading());

            if (gamepad1.options) {
                robot.drivetrain.resetHeading();
                heading.setTarget(0);
                heading.resetState();
                headingDeg = 0;
            }
            double scale = gamepad1.right_bumper ? Constants.SLOW_MODE_SCALE : 1.0;

            double forward = -gamepad1.left_stick_y * scale;
            double strafe  =  gamepad1.left_stick_x * scale;


            double joy = gamepad1.right_stick_x;     // positive orientation = right turn
            if (Math.abs(joy) > Constants.TURN_INPUT_DEADBAND) {
                double newTarget = heading.getTarget()
                        - joy * Constants.MAX_TURN_RATE_DEG * scale * dt;   // right turning = decreasing heading
                double lead = HeadingController.wrapDeg(newTarget - headingDeg);
                lead = Math.max(-Constants.MAX_LEAD_DEG, Math.min(Constants.MAX_LEAD_DEG, lead));
                heading.setTarget(headingDeg + lead);
            }

            // pull back to original position
            double ff = 0;
            if (Math.abs(joy) > Constants.TURN_INPUT_DEADBAND) {
                ff = joy * Constants.TURN_FF_POWER * scale;
            }
            double turn = heading.update(headingDeg) + ff;
            turn = Math.max(-1.0, Math.min(1.0, turn));
            robot.drivetrain.driveFieldCentric(forward, strafe, turn);

            telemetry.addData("Chạy chậm", gamepad1.right_bumper);
            telemetry.addData("Mục tiêu (độ)", heading.getTarget());
            telemetry.addData("Heading (độ)", headingDeg);
            telemetry.addData("Lỗi (độ)", heading.getError(headingDeg));
            robot.drivetrain.updateTelemetry(telemetry);
            telemetry.update();
        }

        robot.stopAll();
    }
}