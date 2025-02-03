package frc.modules;

// import static com.team303.robot.Robot.heldObject;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import frc.robot.Robot;
import frc.robot.Robot.FieldPosition;
import frc.robot.Robot.ReefPosition;
// import com.team303.robot.Robot.HeldObject;
// import com.team303.robot.commands.led.LEDBounce;
// import frc.robot.util.Alert;
import frc.robot.util.ReefState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
// import frc.robot.util.Alert.AlertType;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorControlModule extends SubsystemBase {
    public static final ShuffleboardTab OPERATOR_TAB = Shuffleboard.getTab("Operator");
    public static final NetworkTable operator = NetworkTableInstance.getDefault().getTable("Operator");

    public static final String[] sideNames = { "A Side", "B Side", "C Side", "D Side", "E Side", "F Side", "G Side",
            "H Side", "I Side", "J Side", "K Side", "L Side" };

    public static long[][] nodeStateValues = new long[12][4];
    public static long[][] nodeSuperStateValues = new long[12][4];
    public static long[][] integratedNodeValues = new long[12][4];
    public static GenericEntry strategyEntry;

    // public static boolean coopertitionBonusAchieved;

    public boolean queueManualOverride = false;
    // public boolean suggestManualOverride = false;

    public Point hoverValue = new Point(0,0);
    public Point queuedValue;

    public Timer timer = new Timer();
    // private final Alert lol = new Alert("Operator Terminal", "wow it
    // works",AlertType.kError);

    private final Alert logQueueOnFilledNode = new Alert("Operator Terminal",
            "Attempted to queue on already-filled node, queue not performed", AlertType.kWarning);
    private final Alert logCommandLoopOverrun = new Alert("Operator Terminal",
            "Command loop overrun", AlertType.kError);
    private final Alert logNoMoreSpace = new Alert("Operator Terminal","No more space to place coral ;-;",AlertType.kWarning);

    public static enum NodeState {
        NONE(0),
        CORAL(1);

        public final int value;

        private NodeState(int value) {
            this.value = value;
        }
    }

    public static enum NodeSuperState {
        NONE(0),
        HOVER(2),
        QUEUED(3),
        IN_PROGRESS(4);

        public final int value;

        private NodeSuperState(int value) {
            this.value = value;
        }
    }

    public static enum ScoreStrategy {
        MAX_POINTS,
        MIN_TIME;
    }

    ScoreStrategy currentStrategy;

    public OperatorControlModule() {
        for (String s : sideNames) {
            OPERATOR_TAB.add(s, new ReefState(0, 0, 0, 0)).withWidget("ReefSelector").buildInto(operator, operator);
        }
        currentStrategy = ScoreStrategy.MAX_POINTS;
        strategyEntry = OPERATOR_TAB.add("Current Strategy", "Error: No strategy set").getEntry();
    }

    public void moveUp() {
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        do {
            hoverValue.y = (hoverValue.y + 1) % 4;

        } while (nodeSuperStateValues[hoverValue.x][hoverValue.y] != NodeSuperState.NONE.value
                && nodeSuperStateValues[hoverValue.x][hoverValue.y] != NodeSuperState.QUEUED.value);
        nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
    }

    public void moveDown() {
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        do {
            hoverValue.y = (hoverValue.y - 1) % 4;
            if (hoverValue.y < 0)
                hoverValue.y += 4;
        } while (nodeSuperStateValues[hoverValue.x][hoverValue.y] != NodeSuperState.NONE.value
                && nodeSuperStateValues[hoverValue.x][hoverValue.y] != NodeSuperState.QUEUED.value);
        nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
    }

    public void moveClockwise() {
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        do {
            hoverValue.x = (hoverValue.x - 1) % 12;
            if (hoverValue.x < 0)
                hoverValue.x += 12;
        } while (nodeSuperStateValues[hoverValue.x][hoverValue.y] != NodeSuperState.NONE.value
                && nodeSuperStateValues[hoverValue.x][hoverValue.y] != NodeSuperState.QUEUED.value);
        nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
    }

    public void moveCounterclockwise() {
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        do {
            hoverValue.x = (hoverValue.x + 1) % 12;
        } while (nodeSuperStateValues[hoverValue.x][hoverValue.y] != NodeSuperState.NONE.value
                && nodeSuperStateValues[hoverValue.x][hoverValue.y] != NodeSuperState.QUEUED.value);
        nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
    }

    public void moveRight() {
        // if(hoverValue.x<=3||hoverValue.x>=10) {
        // moveCounterclockwise();
        // } else {
        moveClockwise();
        // }

    }

    public void moveLeft() {
        // if(hoverValue.x<=3||hoverValue.x>=10) {
        // moveClockwise();
        // } else {
        moveCounterclockwise();
        // }

    }

    public void lockIn() {
        nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.IN_PROGRESS.value;
    }

    public void lockOut() {
        nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
        nodeStateValues[queuedValue.x][queuedValue.y] = NodeState.CORAL.value;
        queuedValue=null;
        autoHover();
    }
    public FieldPosition getQueuedPosition() {
        System.out.println("yeee");
        if(queuedValue==null) {
            return FieldPosition.CURRENT_POSE;
        }
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            switch (queuedValue.x) {
                case 0:
                    return FieldPosition.BLUE_REEF_A;
                case 1:
                    return FieldPosition.BLUE_REEF_B;
                case 2:
                    return FieldPosition.BLUE_REEF_C;
                case 3:
                    return FieldPosition.BLUE_REEF_D;
                case 4:
                    return FieldPosition.BLUE_REEF_E;
                case 5:
                    return FieldPosition.BLUE_REEF_F;
                case 6:
                    return FieldPosition.BLUE_REEF_G;
                case 7:
                    return FieldPosition.BLUE_REEF_H;
                case 8:
                    return FieldPosition.BLUE_REEF_I;
                case 9:
                    return FieldPosition.BLUE_REEF_J;
                case 10:
                    return FieldPosition.BLUE_REEF_K;
                case 11:
                    return FieldPosition.BLUE_REEF_L;
                default:
                    System.out.println("getQueuedPosition() somehow returned value <=0 or >=12");
                    return FieldPosition.CURRENT_POSE;
            }
        } else {
            switch (queuedValue.x) {
                case 0:
                    return FieldPosition.RED_REEF_A;
                case 1:
                    return FieldPosition.RED_REEF_B;
                case 2:
                    return FieldPosition.RED_REEF_C;
                case 3:
                    return FieldPosition.RED_REEF_D;
                case 4:
                    return FieldPosition.RED_REEF_E;
                case 5:
                    return FieldPosition.RED_REEF_F;
                case 6:
                    return FieldPosition.RED_REEF_G;
                case 7:
                    return FieldPosition.RED_REEF_H;
                case 8:
                    return FieldPosition.RED_REEF_I;
                case 9:
                    return FieldPosition.RED_REEF_J;
                case 10:
                    return FieldPosition.RED_REEF_K;
                case 11:
                    return FieldPosition.RED_REEF_L;
                default:
                    System.out.println("getQueuedPosition() somehow returned value <=0 or >=12");
                    return FieldPosition.CURRENT_POSE;
            }
        }
    }

    public void setPiece() {
        if (nodeStateValues[hoverValue.x][hoverValue.y] == NodeState.CORAL.value) {
            nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.NONE.value;
            autoHover();
        } else {
            nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.CORAL.value;
            autoHover();
        }
    }

    public void queuePlacement() {
        if (hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
            queueManualOverride = false;
            queuedValue = null;
        } else {
            if (nodeStateValues[hoverValue.x][hoverValue.y] != NodeState.NONE.value) {
                logQueueOnFilledNode.set(true);
                return;
            }
            logQueueOnFilledNode.set(false);
            if (queuedValue == null) {
                nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
                queueManualOverride = true;
                queuedValue = new Point(hoverValue);
            } else {
                nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
                queueManualOverride = true;
                nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
                queuedValue.x = hoverValue.x;
                queuedValue.y = hoverValue.y;
            }
        }
    }

    public void toggleStrategy() {
        if (currentStrategy == ScoreStrategy.MAX_POINTS) {
            currentStrategy = ScoreStrategy.MIN_TIME;
        } else {
            currentStrategy = ScoreStrategy.MAX_POINTS;
        }

    }

    public void autoHover() {
        // if (queueManualOverride) {
        //     return;
        // }
        if(hoverValue!=null){
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;            
        }
        // if (queuedValue != null) {
        //     nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
        // }

        // get current pose
        // find distance between curpose and all scoring poses
        // sort by distance to scoring pose
        Pose2d currentPose = Robot.swerve.getPose();
        ArrayList<Pair<Double,FieldPosition>> ordered_set = new ArrayList<>();
        for (ReefPosition position : ReefPosition.values()){
            FieldPosition converted = FieldPosition.valueOf(position.name());
            //only use reefpositions of current alliance
            if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue && converted.name().substring(0,3).equals("RED")) {
                continue;
            } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red && converted.name().substring(0,4).equals("BLUE")) {
                continue;
            } else if(!DriverStation.getAlliance().isPresent() && converted.name().substring(0,4).equals("BLUE")) {
                continue;
            }
            ordered_set.add(new Pair<Double,FieldPosition>(currentPose.getTranslation().getDistance(Robot.swerve.calculateFieldPosition(converted).getTranslation()),converted));
        }
        Collections.sort(ordered_set,new Comparator<Pair<Double,FieldPosition>>(){

            public int compare(Pair<Double,FieldPosition> o1, Pair<Double,FieldPosition> o2)
            {
               return o1.getFirst().compareTo(o2.getFirst());
            }
          });
        if (currentStrategy == ScoreStrategy.MAX_POINTS) {
            //MAX_POINTS strategy
            // check all l4's, then l3's, then l2's, then l1's
            for(int i=3;i>=0;i--){
                for (int j=0;j<ordered_set.size();j++){
                    String side = ordered_set.get(j).getSecond().name();
                    if(nodeStateValues[side.charAt(side.length()-1)-65][i] == NodeState.NONE.value){
                        if(hoverValue!=null) {
                            nodeSuperStateValues[hoverValue.x][hoverValue.y]=NodeSuperState.NONE.value;
                        }
                        hoverValue.x=side.charAt(side.length()-1)-65;
                        hoverValue.y=i;
                        nodeSuperStateValues[hoverValue.x][hoverValue.y]=NodeSuperState.HOVER.value;
                        return;
                    }
                }
            }
            logNoMoreSpace.set(true);
            return;
        } else {
            //MIN_TIME strategy
            // check all of closest side, then next closest side etc.
            for(int j=0;j<ordered_set.size();j++){
                for (int i=3;i>=0;i--){
                    String side = ordered_set.get(j).getSecond().name();
                    if(nodeStateValues[side.charAt(side.length()-1)-65][i] == NodeState.NONE.value){
                        if(hoverValue!=null) {
                            nodeSuperStateValues[hoverValue.x][hoverValue.y]=NodeSuperState.NONE.value;
                        }
                        hoverValue.x=side.charAt(side.length()-1)-65;
                        hoverValue.y=i;
                        nodeSuperStateValues[hoverValue.x][hoverValue.y]=NodeSuperState.HOVER.value;
                        return;
                    }
                }
            }
            logNoMoreSpace.set(true);
            return;
        }
    }

    @Override
    public void periodic() {
        timer.start();
        if(hoverValue == null) {
            autoHover();
        }
        if((DriverStation.isFMSAttached() || (!DriverStation.isAutonomous() && !DriverStation.isTeleop() && !DriverStation.isTest()))&&DriverStation.getMatchTime()<30){
            if(currentStrategy == ScoreStrategy.MAX_POINTS) {
                currentStrategy = ScoreStrategy.MIN_TIME;
            }
        }
        strategyEntry.setString(currentStrategy.name());
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 4; j++) {
                if (nodeSuperStateValues[i][j] == NodeSuperState.NONE.value) {
                    integratedNodeValues[i][j] = nodeStateValues[i][j];
                } else {
                    integratedNodeValues[i][j] = nodeSuperStateValues[i][j];
                }
            }
            operator.getSubTable(sideNames[i]).getEntry("State").setIntegerArray(integratedNodeValues[i]);
        }
        if (timer.hasElapsed(0.02)) {
            logCommandLoopOverrun.set(true);
        } else {
            logCommandLoopOverrun.set(false);
        }
    }
}