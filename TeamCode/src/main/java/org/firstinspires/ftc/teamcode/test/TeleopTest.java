package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.BlockerTask;
import org.firstinspires.ftc.teamcode.task.FlywheelTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.limelight.Coords;

@Config
@TeleOp(name = "Test TeleOp", group = "Test")
public class TeleopTest extends LinearOpMode {

    /*
    setup:
    corner of tape measure = on the corner tile under the goal, 10 inches behind the atag
    other end of tape measure = on the edge of the far wall, exactly 2 tiles away from the opposite alliance edge
    goal edge @ ~9.5 inches (sqrt(10^2-3^2))

    dist (inch) ||  RPM   ||  angle (shooter)
   <------------------------------>
        150     ||  4550  ||     0.47 XXXX
        140     ||  4050  ||     0.47 DONE
        130     ||  3910  ||     0.46 DONE
        120     ||  3650  ||     0.44 DONE
        110     ||  3480  ||     0.40 DONE
        100     ||  3400  ||     0.37 DONE
         90     ||  3300  ||     0.35 DONE
         80     ||  3300  ||     0.36 DONE
         70     ||  3050  ||     0.28 DONE
         60     ||  2900  ||     0.26 DONE
         50     ||  2800  ||     0.24 DONE
         40     ||  2600  ||     0.12 DONE
         35     ||  2500  ||     0.06 DONE
         20     ||  N/A   ||     N/A
         10     ||  N/A   ||     N/A
     */

    public static int FLYWHEEL_RPM = 2500;

    // Pivot DOWN: 0.57
    // Pivot FULL EXTENSION: 1
    public static double PIVOT_POS = 0.4, SERVO_SKIP_CORRECTION = 0.01, INTAKE_POWER = 0.8, INTAKE_IDLE_POWER = 0.0;
    public static double TURRET_TICKS_PER_RADIANS = 103.8*2.0, TURRET_TARGET_RAD = 0.0;
    public static boolean isRed = false;
    public static boolean rampUp, flywheelActive, aiming, autoaim = true, turretActive = false, turretAutoAim = true, updateLimelight = true;
    private Task task;
    public static int SORT_SLEEP_TIME = 100;

    public static double PARK_POSITION = 1.0;

    public MultipleTelemetry multipleTelemetry;
    private SensorUpdateThread sensorUpdateThread;

    public GamePad gp1, gp2;
    private RobotBase robot;

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);
        robot.init(new Pose(0, 0, Math.PI));

        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
        sensorUpdateThread = new SensorUpdateThread(robot);

        waitForStart();

        robot.startTeleopDrive();
        robot.startLimelight();
        //robot.startArtifactPipeline();
        robot.startAprilTagPipeline();

        robot.setLimelightAllianceColor(isRed);

        sensorUpdateThread.start();

        while (opModeIsActive()) {
            update();
            if(updateLimelight) {
                robot.updateLimelight();
                robot.updateRobotPoseUsingLimelight();
            }
//            Pose limelightPose = robot.getLimelightRobotPose();
//            multipleTelemetry.addData("robot position using limelight", limelightPose.getX()+" "+limelightPose.getY()+" "+limelightPose.getHeading());
//            Pose offsetPose = robot.limelightTransOffset;
//            multipleTelemetry.addData("robot offset using limelight", offsetPose.getX()+" "+offsetPose.getY()+" "+offsetPose.getHeading());

            Pose clusterPose = robot.getTargetArtifactClusterPose();
            multipleTelemetry.addData("closest cluster pose", clusterPose.getX()+" "+clusterPose.getY()+" "+clusterPose.getHeading());
            Coords clusterCoords = robot.getTargetArtifactClusterCoords();
            multipleTelemetry.addData("closest cluster coords", clusterCoords.getX()+" "+clusterCoords.getY()+" "+clusterCoords.getAngle());
            if (gp1.onceB()){
                if(task!=null) {
                    task.cancel();
                }
                task = new SeriesTask(
                        new FlywheelTask(robot, 1200, 500),
                        new ParallelTask(
                                new UnboundedIntakeTask(robot, 0.6, false),
                                new SeriesTask(
                                        new ParallelTask(
                                                new RampTask(robot, RampTask.Position.UP),
                                                new BlockerTask(robot, BlockerTask.Position.OPEN)
                                        ),
                                        new SleepTask(SORT_SLEEP_TIME),
                                        new ParallelTask(
                                                new RampTask(robot, RampTask.Position.DOWN),
                                                new BlockerTask(robot, BlockerTask.Position.CLOSE)
                                        )
                                )
                        ),
                        new IntakeTask(robot, 1.0, false, 1000)
                );
            } else if(task==null) {
                if(turretActive) {
                    robot.setTurretTargetPosition(TURRET_TARGET_RAD);
                } else if(turretAutoAim) {
                    robot.turretAutoAim();
                }else {
                    robot.setTurretTargetPosition(0);
                }

                if(autoaim) {
                    FLYWHEEL_RPM = robot.calcFlywheelRpm();
                    PIVOT_POS = robot.calcPivotPosition();
                }

                robot.setPivotPosition(PIVOT_POS);


//            if(autoaim) {
//                robot.setPivotPosition(calcPivotPosition(robot.getDistToGoalInches()));
//                FLYWHEEL_RPM = calcFlywheelRpm(robot.getDistToGoalInches());
//            } else {
//                robot.setPivotPosition(PIVOT_POS);
//            }
                robot.setPivotPosition(PIVOT_POS);
//
                if(gp1.onceX()) flywheelActive = !flywheelActive;
                if(flywheelActive) {
                    robot.setFlywheelTargetVelocity(FLYWHEEL_RPM);
                } else {
                    robot.setFlywheelTargetVelocity(0);
                }
//
                if(gp1.rightTrigger()>0.3) {
                    robot.runIntakeWithPower(INTAKE_POWER);
                } else if(gp1.leftTrigger()>0.3) {
                    robot.runIntakeReversed();
                } else robot.runIntakeWithPower(INTAKE_IDLE_POWER);

//            if(gp1.rightBumper()) {
//                robot.runPusher();
//            } else if(gp1.leftBumper()) {
//                robot.runPusherReversed();
//            } else robot.stopPusher();

                if(gp1.onceA()) {
                    rampUp = !rampUp;
                }
//

                if(rampUp) {
                    robot.setRampPosition(RampTask.Position.UP);
                    robot.setBlockerPosition(BlockerTask.Position.OPEN);
                } else {
                    robot.setRampPosition(RampTask.Position.DOWN);
                    robot.setBlockerPosition(BlockerTask.Position.CLOSE);
                }

                robot.leftPark.setPosition(PARK_POSITION);
                robot.rightPark.setPosition(PARK_POSITION);

                drive();

            }

            gp1.update();
            gp2.update();

//            multipleTelemetry.addData("dist to goal", robot.getDistToGoalInches());
//            multipleTelemetry.addData("robot pos", robot.getPose().getX()+" "+robot.getPose().getY()+" "+robot.getPose().getHeading());
            multipleTelemetry.addData("current rpm", robot.getFlywheelVelocityRpm());
            multipleTelemetry.addData("target rpm", FLYWHEEL_RPM);
            multipleTelemetry.addData("turret pos", robot.getTurretAngleTicks());
            Pose limelightPose = robot.getLimelightRobotPose();
            multipleTelemetry.addData("robot position using limelight", limelightPose.getX()+" "+limelightPose.getY()+" "+limelightPose.getHeading());
            multipleTelemetry.addData("distance to goal", robot.getVectorToGoal().getMagnitude());
            multipleTelemetry.update();
        }
        sensorUpdateThread.interrupt();
        robot.stopLimelight();
    }

    private void update() {
        robot.updateEverything();
        if(task != null && task.perform()) task = null;
    }

    private void drive() {
        if(gp1.onceY()) {
            aiming = !aiming;
            if(!aiming) {
                robot.startTeleopDrive();
            } else {
                robot.holdPoint(new BezierPoint(robot.getPose()), robot.getVectorToGoal().getTheta(), false);
            }
        }
        if(aiming) {
        } else {
            double x = 0, y = 0, a = 0;
            if (gp1.dpadLeft() || gp1.dpadRight()) {
                a = 0.5 * (gp1.dpadLeft() ? 1 : -1);
            } else if (gp1.dpadUp() || gp1.dpadDown()) {
                x = 0.3 * (gp1.dpadUp() ? 1 : -1);
            } else {
                x = -gp1.leftStickY();
                y = -gp1.leftStickX();
                a = -gp1.rightStickX() * 0.5;
            }
            double pow = Math.sqrt(x * x + y * y);
//        double limPow = driveLim.calculate(pow);
//        if (pow > MIN_DRIVE_POW) {
//            x *= limPow / pow;
//            y *= limPow / pow;
//        }
//        a = turnLim.calculate(a);
//        telemetry.addData("driving", x + " " + y + " " + a);

//        robot.setDrivePowers(x, y, a);
//            if(Math.abs(a)<0.05) robot.updateRobotPoseUsingLimelight();
            robot.setTeleOpDrive(x, y, a, true);
        }
    }

    private double calcPivotPosition(double x) {
        double coef[] = {
                -6.653177,
                0.7294976,
                -0.03392037,
                0.0008750063,
                -0.00001355061,
                1.290464 * Math.pow(10.0, -7.0),
                -7.400309 * Math.pow(10.0, -10.0),
                2.344162 * Math.pow(10.0, -12.0),
                -3.150742 * Math.pow(10.0, -15.0)

        };
        x = Utils.clamp(x, 35, 150);

        double pos = 0;
        for(double i =0; i<9.0; i++){
            pos+=Math.pow(x, i) *coef[(int)i];
        }

        return pos;
    }
    private int calcFlywheelRpm(double distToGoalInches) {
        double coef[] = {
                -144032.529,
                17312.5221,
                -880.32487,
                25.271665,
                -0.45127892,
                0.0052033398,
                -0.00003878999,
                1.8057785 * Math.pow(10.0, -7.0),
                -4.7719158 * Math.pow(10.0, -10.0),
                5.4638853 * Math.pow(10.0, -13.0)};

        double rpm = 0;
        distToGoalInches = Utils.clamp(distToGoalInches, 35, 150);
        for(double i = 0; i<10.0; i++){
            rpm+=Math.pow(distToGoalInches, i) * coef[(int)i];
        }

        return (int)rpm;
    }
}