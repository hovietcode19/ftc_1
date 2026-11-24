package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * hằng số ALLIANCE quyết định team (location for the upper/lower field), Pinpoint tự nhận vị trí
 * xuất phát theo đội đã code, telemetry mặc định vị trí xuất phát cấu hình ở zone_tile.
 *
 * HỆ TRỤC SÂN (gốc ở tâm sân, đơn vị mm):
 *   +X = hướng lên trên về phía sân trên
 *   +Y = hướng sang trái theo hình sân
 *   heading 0 độ = robot quay mặt về +X (lên trên), cclw = +Y, cw = -Y
 *
 * Ô lưới: hàng 0 = trên cùng, cột 0 = ngoài cùng bên trái.
 */
@TeleOp(name = "TurretShootingZone")
public class TurretShootingZone extends LinearOpMode {

    private enum Alliance { RED, BLUE }

    private static final Alliance ALLIANCE = Alliance.RED; // Alliance.BLUE;

    // Field dimensions
    static final double TILE = 609.6;      // 1 ô = 24 inch
    static final double HALF = 3 * TILE;   // sân 6x6 ô, nửa sân = 3 ô

    //  Offset pod odometry

    static final double X_POD_OFFSET_MM = 0.0;
    static final double Y_POD_OFFSET_MM = -130.0;

    // Encoder orientation
    static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // beginning tile
    // RED: lower field, row 6 and column 2
    static final double RED_START_X_MM   = -2.5 * TILE;
    static final double RED_START_Y_MM   =  1.5 * TILE;
    static final double RED_START_HEADING_DEG = 0;   // forward to X axis

    // BLUE: upper field, row 0 and column 4
    static final double BLUE_START_X_MM  =  2.5 * TILE;
    static final double BLUE_START_Y_MM  = -1.5 * TILE;
    static final double BLUE_START_HEADING_DEG = 180;  // reverse to X axis

    // shooting zone
    static final double ZONE_WIDTH_TILES = 5;   // 5 tiles width
    static final double ZONE_DEPTH_TILES = 2;   // 2 tiles depth
    static final double ZONE_MARGIN_MM = 0;

    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp1");

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_ENCODER_DIRECTION, Y_ENCODER_DIRECTION);

        // reset encoder and IMU
        pinpoint.resetPosAndIMU();

        final Alliance alliance = ALLIANCE;
        boolean poseApplied = false;

        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();


            if (!poseApplied
                    && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                applyStartPose(alliance);
                poseApplied = true;
            }

            telemetry.addLine("Giữ robot đứng yên cho Pinpoint hiệu chuẩn");
            showTelemetry(alliance, poseApplied);
        }

        while (opModeIsActive()) {
            pinpoint.update();
            showTelemetry(alliance, poseApplied);
        }
    }

    private void applyStartPose(Alliance alliance) {
        if (alliance == Alliance.RED) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.MM,
                    RED_START_X_MM, RED_START_Y_MM,
                    AngleUnit.DEGREES, RED_START_HEADING_DEG));
        } else {
            pinpoint.setPosition(new Pose2D(DistanceUnit.MM,
                    BLUE_START_X_MM, BLUE_START_Y_MM,
                    AngleUnit.DEGREES, BLUE_START_HEADING_DEG));
        }
    }

    private void showTelemetry(Alliance alliance, boolean poseApplied) {
        Pose2D pos = pinpoint.getPosition();
        double x = pos.getX(DistanceUnit.MM);
        double y = pos.getY(DistanceUnit.MM);
        double heading = pos.getHeading(AngleUnit.DEGREES);

        String teamText = (alliance == null) ? "CHƯA CHỌN"
                : (alliance == Alliance.RED ? "ĐỎ" : "XANH");

        telemetry.addData("Đội", teamText);
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Đã set vị trí xuất phát", poseApplied);
        telemetry.addData("X (mm)", "%.0f", x);
        telemetry.addData("Y (mm)", "%.0f", y);
        telemetry.addData("Heading (độ)", "%.1f", heading);
        telemetry.addData("Vùng bắn", zoneMessage(alliance, x, y));
        telemetry.update();
    }

    private String zoneMessage(Alliance alliance, double x, double y) {
        if (alliance == null) return "Chưa chọn đội";

        String color = (alliance == Alliance.RED) ? "đỏ" : "xanh";
        int side = zoneSide(alliance, x, y);

        // Trong vùng bắn: "Vùng bắn đỏ sân dưới"
        if (side != 0) {
            return "Vùng bắn " + color + " sân " + (side > 0 ? "trên" : "dưới");
        }

        String field = (x >= 0) ? "trên" : "dưới";
        return "Ngoài vùng bắn màu " + color + " sân " + field;
    }

    /**
     * Vùng bắn mỗi đội gồm 2 khối (1 ở sân trên, 1 ở sân dưới), kích thước
     * ZONE_WIDTH_TILES x ZONE_DEPTH_TILES (6x2):
     *   Đỏ : bám mép TRÁI sân  -> Y từ (HALF - rộng) đến HALF
     *   Xanh: bám mép PHẢI sân -> Y từ -HALF đến (-HALF + rộng)
     *   Sân trên: bám mép trên  -> X từ (HALF - sâu) đến HALF
     *   Sân dưới: bám mép dưới  -> X từ -HALF đến (-HALF + sâu)
     *
     * @return +1 = sân trên, -1 = sân dưới, 0 = ngoài vùng bắn
     */
    private int zoneSide(Alliance alliance, double x, double y) {
        double m = ZONE_MARGIN_MM;
        double w = ZONE_WIDTH_TILES * TILE;
        double d = ZONE_DEPTH_TILES * TILE;

        double yMin = (alliance == Alliance.RED) ? HALF - w : -HALF;
        double yMax = (alliance == Alliance.RED) ? HALF      : -HALF + w;

        if (y < yMin - m || y > yMax + m) return 0;

        if (x >= HALF - d - m && x <= HALF + m) return 1;      // sân trên
        if (x <= -HALF + d + m && x >= -HALF - m) return -1;   // sân dưới
        return 0;
    }
}