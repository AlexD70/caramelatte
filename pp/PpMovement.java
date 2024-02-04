package org.firstinspires.ftc.teamcode.pp;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.pp.MathFunct.*;


import org.firstinspires.ftc.teamcode.pp.MathFunct;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.drive.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.util.MecanumDriveEx;
import org.opencv.core.Mat;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import org.opencv.core.Point;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;

import java.util.ArrayList;

public class PpMovement {
    public static StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    public static Pose2d myPose = myLocalizer.getPoseEstimate();
    public static MecanumDriveEx drive = new MecanumDriveEx(hardwareMap);


    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(myPose.getX(),myPose.getY()),
                allPoints.get(0).followDistance);

        goToPosition(followMe.x,followMe.y,followAngle,followMe.moveSpeed, followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i ++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation,followRadius,startLine.toPoint(),
                    endLine.toPoint());

            double closestAngle = 9999999;

            for(Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - myPose.getY(), thisIntersection.x - myPose.getX());
                double deltaAngle = Math.abs(AngleWrap(angle - myPose.getHeading()));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }

            }
        }
        return followMe;
    }


    public static void goToPosition(double x, double y, double heading, double movementSpeed, double turnSpeed){

        heading = Math.toRadians(heading);

        double distanceToPoint = Math.hypot(x - myPose.getX(), y -myPose.getY());

        double absoluteAngle = Math.atan2(y-myPose.getY(),x-myPose.getX());
        double relativeAngle = AngleWrap(absoluteAngle - myPose.getHeading());

        double relativeXtoPoint = Math.cos(relativeAngle) * distanceToPoint;
        double relativeYtoPoint = Math.sin(relativeAngle) * distanceToPoint;

        double movementXPower = relativeXtoPoint / (Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));
        double movementYPower = relativeYtoPoint / (Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));
        double relativeTurnAngle = relativeAngle - Math.toRadians(180) + heading;

        double xPower = movementXPower * movementSpeed;
        double yPower = movementYPower * movementSpeed;
        double hPower = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turnSpeed;

        if (distanceToPoint < 10)
            hPower = 0;

        Pose2d drivePowers = new Pose2d(xPower,yPower,hPower);


        drive.setDrivePower(drivePowers);

    }
}
