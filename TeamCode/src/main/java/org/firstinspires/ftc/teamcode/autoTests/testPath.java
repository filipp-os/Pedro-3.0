package org.firstinspires.ftc.teamcode.autoTests;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "testPath", group = "Autonomous")
public class testPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(35.6064, 4.235, 90);
    private final Pose point1 = poseFactory.of(34.9828, 4.1938, 90);
    private final Pose point1Control1 = poseFactory.of(44.1929, 27.3304, 0);
    private final Pose point1Control2 = poseFactory.of(53.2317, 36.9002, 0);
    private final Pose point1Control3 = poseFactory.of(86.6075, 3.9346, 0);
    private final Pose point2 = poseFactory.of(34.5643, 4.092, 90);
    private final Pose point2Control1 = poseFactory.of(37.4302, 50.1729, 0);
    private final Pose point2Control2 = poseFactory.of(-17, 8.8592, 0);
    private final Pose point3 = poseFactory.of(26.5266, 57, 90);
    private final Pose point4 = poseFactory.of(46.0776, 60.1341, 90);
    private final Pose point4Control1 = poseFactory.of(32.469, 82.1175, 0);
    private final Pose point5 = poseFactory.of(43.9978, 5.6796, 90);



    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return curve(start, point1Control1, point1Control2, point1Control3, point1).constant(point1);
    }

    public Path path2() {
        return curve(point1, point2Control1, point2Control2, point2).constant(point2);
    }

    public Path path3() {
        return line(point2, point3).constant(point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4).constant(point4);
    }

    public Path path5() {
        return line(point4, point5).constant(point5);
    }
}

