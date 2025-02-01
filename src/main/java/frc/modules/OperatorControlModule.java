package frc.modules;

// import static com.team303.robot.Robot.heldObject;

import java.awt.Point;

// import com.team303.robot.Robot.HeldObject;
// import com.team303.robot.commands.led.LEDBounce;
// import frc.robot.util.Alert;
import frc.robot.util.ReefState;
// import frc.robot.util.Alert.AlertType;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
    // public static final SendableChooser<HeldObject> heldObjectChooser = new SendableChooser<HeldObject>();
    // public static HeldObject heldObjectIn;

    // public static final GenericEntry[][] nodes = new GenericEntry[3][9];
    public static final String[] sideNames = {"A Side","B Side","C Side","D Side","E Side","F Side","G Side","H Side","I Side","J Side","K Side", "L Side"};

    public static long[][] nodeStateValues = new long[12][4];
    public static long[][] nodeSuperStateValues = new long[12][4];
    public static long[][] integratedNodeValues = new long[12][4];

    // public static boolean coopertitionBonusAchieved;

    // public boolean queueManualOverride = false;
    // public boolean suggestManualOverride = false;

    public Point hoverValue = new Point(0, 0);
    public Point queuedValue = new Point(-1,-1);

    public Timer timer = new Timer();
    // private final Alert lol = new Alert("Operator Terminal", "wow it works",AlertType.kError);

    private final Alert logQueueOnFilledNode = new Alert("Operator Terminal",
            "Attempted to queue on already-filled node, queue not performed", AlertType.kWarning);
    // private final Alert logNoMorePieceSpaceCones = new Alert("Operator Terminal",
    //         "No more space to place cones, queue canceled", AlertType.WARNING);
    // private final Alert logNoMorePieceSpaceCubes = new Alert("Operator Terminal",
    //         "No more space to place cubes, queue canceled", AlertType.WARNING);
    private final Alert logCommandLoopOverrun = new Alert("Operator Terminal",
            "Command loop overrun", AlertType.kError);

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
        INVALID(4);

        public final int value;

        private NodeSuperState(int value) {
            this.value = value;
        }
    }

    

    public OperatorControlModule() {
        timer.start();
        for(String s: sideNames){
            OPERATOR_TAB.add(s,new ReefState(0,0,0,0)).withWidget("ReefSelector").buildInto(operator,operator);
        }
    }





    public void moveUp() {
        System.out.println("down");
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        do {
            hoverValue.y=(hoverValue.y+1)%4;

        } while (nodeSuperStateValues[hoverValue.x][hoverValue.y]!=NodeSuperState.NONE.value && nodeSuperStateValues[hoverValue.x][hoverValue.y]!=NodeSuperState.QUEUED.value);
        nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
    }
    public void moveDown() {
        System.out.println("up");
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        } do {
            hoverValue.y=(hoverValue.y-1)%4;
            if(hoverValue.y<0) hoverValue.y+=4;
        } while(nodeSuperStateValues[hoverValue.x][hoverValue.y]!=NodeSuperState.NONE.value && nodeSuperStateValues[hoverValue.x][hoverValue.y]!=NodeSuperState.QUEUED.value);
        nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
    }
    public void moveClockwise() {
        System.out.println("left");
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        do {
            hoverValue.x=(hoverValue.x-1)%12;
            if(hoverValue.x<0) hoverValue.x+=12;
        } while(nodeSuperStateValues[hoverValue.x][hoverValue.y]!=NodeSuperState.NONE.value && nodeSuperStateValues[hoverValue.x][hoverValue.y]!=NodeSuperState.QUEUED.value);
        nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
    }
    public void moveCounterclockwise() {
        System.out.println("right");
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        do {
            hoverValue.x=(hoverValue.x+1)%12;
        } while(nodeSuperStateValues[hoverValue.x][hoverValue.y]!=NodeSuperState.NONE.value && nodeSuperStateValues[hoverValue.x][hoverValue.y]!=NodeSuperState.QUEUED.value);
        nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
    }

    public void moveRight() {
        // if(hoverValue.x<=3||hoverValue.x>=10) {
        //     moveCounterclockwise();
        // } else {
            moveClockwise();
        // }

    }

    public void moveLeft() {
        // if(hoverValue.x<=3||hoverValue.x>=10) {
        //     moveClockwise();
        // } else {
            moveCounterclockwise();
        // }

    }

    // private void changeTarget(int x, int y) {
	// 	System.out.print(x + " : " + y);
	// 	if (!hoverValue.equals(queuedValue)) {
	// 		nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
	// 	} else {
	// 		nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
	// 	}
	// 	if (nodeSuperStateValues[x][y] == NodeSuperState.NONE.value
	// 			|| nodeSuperStateValues[x][y] == NodeSuperState.QUEUED.value) {
	// 		nodeSuperStateValues[x][y] = NodeSuperState.HOVER.value;
	// 		hoverValue.x = x;
	// 		hoverValue.y = y;
	// 		System.out.println("manual Select");
	// 		return;
	// 	}
	// }


    // public void manualSuggest() {
    //     int state = (int) hpSuggestion.getInteger(0);
    //     if (state == 2) {
    //         hpSuggestion.setInteger(--state);
    //     } else {
    //         hpSuggestion.setInteger(++state);
    //     }
    //     suggestManualOverride = true;
    // }

    // public void setPiece() {
    //     if (nodeStateValues[hoverValue.x][hoverValue.y] == NodeState.CUBE.value
    //             || nodeStateValues[hoverValue.x][hoverValue.y] == NodeState.CONE.value) {
    //         nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.NONE.value;
    //         autoQueuePlacement();
    //     } else if (hoverValue.x > 1) {
    //         nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.CUBE.value;
    //         autoQueuePlacement();
    //     } else if (hoverValue.y % 3 == 0 || hoverValue.y % 3 == 2) {
    //         nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.CONE.value;
    //         autoQueuePlacement();
    //     } else {
    //         nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.CUBE.value;
    //         autoQueuePlacement();
    //     }
    //     if (heldObject == HeldObject.NONE) {
    //         autoSuggestPiece();
    //     }
    // }

    // public void queuePlacement() {
    //     if (nodeSuperStateValues[hoverValue.x][hoverValue.y] == NodeSuperState.QUEUED.value) {
    //         nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
    //         queueManualOverride = false;
    //         queuedValue = null;
    //     } else {
    //         if (nodeStateValues[hoverValue.x][hoverValue.y] != NodeState.NONE.value) {
    //             logQueueOnFilledNode.set(true);
    //             return;
    //         }
    //         logQueueOnFilledNode.set(false);
    //         if (queuedValue == null) {
    //             nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
    //             queueManualOverride = true;
    //             queuedValue = new Point(hoverValue);
    //         } else {
    //             nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
    //             queueManualOverride = true;
    //             nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
    //             queuedValue.x = hoverValue.x;
    //             queuedValue.y = hoverValue.y;
    //         }
    //     }
    // }

    // private boolean partOfCompleteLink(int i, int j) {
    //     int baseNineIndex = 9 * i + j;
    //     if (baseNineIndex % 9 == 0) {
    //         return linkComplete[i * 7];
    //     } else if (baseNineIndex % 9 == 1) {
    //         return (linkComplete[i * 7 + 1] || linkComplete[i * 7]);
    //     } else if (baseNineIndex % 9 >= 2 && baseNineIndex % 9 <= 6) {
    //         return linkComplete[i * 7 + (j - 2)] || linkComplete[i * 7 + j - 1] || linkComplete[i * 7 + j];
    //     } else if (baseNineIndex % 9 == 7) {
    //         return (linkComplete[i * 7 + j - 2] || linkComplete[i * 7 + j - 1]);
    //     } else {
    //         return linkComplete[i * 7 + 6];
    //     }
    // }

    // public void autoSuggestPiece() {
    //     if (suggestManualOverride || heldObject != HeldObject.NONE) {
    //         return;
    //     }
    //     // First priority is to complete coopertition bonus
    //     if (!coopertitionBonusAchieved) {
    //         for (int i = 0; i < 3; i++) {
    //             if ((nodeStateValues[i][3] != NodeState.NONE.value
    //                     && nodeStateValues[i][4] != NodeState.NONE.value)
    //                     || (nodeStateValues[i][4] != NodeState.NONE.value
    //                             && nodeStateValues[i][5] != NodeState.NONE.value)) {
    //                 hpSuggestion.setInteger(NodeState.CONE.value);
    //                 // // // // // new LEDBounce(Color.kYellow);
    //                 return;
    //             } else if ((nodeStateValues[i][3] != NodeState.NONE.value
    //                     && nodeStateValues[i][5] != NodeState.NONE.value)) {
    //                 hpSuggestion.setInteger(NodeState.CUBE.value);
    //                 // // // // // // new LEDBounce(Color.kViolet);
    //                 return;

    //             }
    //         }
    //     }
    //     // Second priority is to complete a link
    //     for (int i = 0; i < 3; i++) {
    //         if ((nodeStateValues[i][0] != NodeState.NONE.value
    //                 && nodeStateValues[i][1] != NodeState.NONE.value)
    //                 && !partOfCompleteLink(i, 0) && !partOfCompleteLink(i, 1)) {
    //             if (i == 2) {
    //                 hpSuggestion.setInteger(NodeState.CUBE.value);
    //                 // // // // // new LEDBounce(Color.kViolet);

    //                 return;
    //             }
    //             hpSuggestion.setInteger(NodeState.CONE.value);
    //             // // // // // new LEDBounce(Color.kYellow);
    //             return;
    //         } else if ((nodeStateValues[i][1] != NodeState.NONE.value
    //                 && nodeStateValues[i][2] != NodeState.NONE.value)
    //                 && !partOfCompleteLink(i, 1) && !partOfCompleteLink(i, 2)) {
    //             if (i == 2) {
    //                 hpSuggestion.setInteger(NodeState.CUBE.value);
    //                 // // // // // new LEDBounce(Color.kViolet);

    //                 return;
    //             }
    //             hpSuggestion.setInteger(NodeState.CONE.value);
    //             // // // // // new LEDBounce(Color.kYellow);

    //             return;
    //         } else if ((nodeStateValues[i][0] != NodeState.NONE.value
    //                 && nodeStateValues[i][2] != NodeState.NONE.value)
    //                 && !partOfCompleteLink(i, 0) && !partOfCompleteLink(i, 2)) {
    //             if (i == 2) {
    //                 hpSuggestion.setInteger(NodeState.CUBE.value);
    //                 // // // // // new LEDBounce(Color.kViolet);

    //                 return;
    //             }
    //             hpSuggestion.setInteger(NodeState.CUBE.value);

    //             return;
    //         }
    //         for (int j = 1; j < 5; j++) {
    //             if ((nodeStateValues[i][j] != NodeState.NONE.value
    //                     && nodeStateValues[i][j + 1] != NodeState.NONE.value) && !partOfCompleteLink(i, j)
    //                     && !partOfCompleteLink(i, j + 1)) {
    //                 if (i == 2) {
    //                     hpSuggestion.setInteger(NodeState.CUBE.value);
    //                     // // // // // new LEDBounce(Color.kViolet);

    //                     return;
    //                 }
    //                 if ((j + 2) % 3 == 1) {
    //                     hpSuggestion.setInteger(NodeState.CUBE.value);
    //                     // // // // new LEDBounce(Color.kViolet);

    //                     return;
    //                 }
    //                 hpSuggestion.setInteger(NodeState.CONE.value);
    //                 // // // // new LEDBounce(Color.kYellow);

    //                 return;
    //             } else if ((nodeStateValues[i][j + 1] != NodeState.NONE.value
    //                     && nodeStateValues[i][j + 2] != NodeState.NONE.value)
    //                     && !partOfCompleteLink(i, j + 1) && !partOfCompleteLink(i, j + 2)) {
    //                 if (i == 2) {
    //                     hpSuggestion.setInteger(NodeState.CUBE.value);
    //                     // // // // new LEDBounce(Color.kViolet);

    //                     return;
    //                 }
    //                 if (j % 3 == 1) {
    //                     hpSuggestion.setInteger(NodeState.CUBE.value);
    //                     // // // // new LEDBounce(Color.kViolet);

    //                     return;
    //                 }
    //                 hpSuggestion.setInteger(NodeState.CONE.value);
    //                 // // // // new LEDBounce(Color.kYellow);

    //                 return;
    //             } else if ((nodeStateValues[i][j] != NodeState.NONE.value
    //                     && nodeStateValues[i][j + 2] != NodeState.NONE.value)
    //                     && !partOfCompleteLink(i, j) && !partOfCompleteLink(i, j + 2)) {
    //                 if (i == 2) {
    //                     hpSuggestion.setInteger(NodeState.CUBE.value);
    //                     // // // // new LEDBounce(Color.kViolet);

    //                     return;
    //                 }
    //                 if ((j + 1) % 3 == 1) {
    //                     hpSuggestion.setInteger(NodeState.CUBE.value);
    //                     // // // // new LEDBounce(Color.kViolet);

    //                     return;
    //                 }
    //                 hpSuggestion.setInteger(NodeState.CONE.value);
    //                 // // // // new LEDBounce(Color.kYellow);

    //                 return;
    //             }
    //         }

    //         if ((nodeStateValues[i][6] != NodeState.NONE.value
    //                 && nodeStateValues[i][7] != NodeState.NONE.value)
    //                 && !partOfCompleteLink(i, 6) && !partOfCompleteLink(i, 7)) {
    //             if (i == 2) {
    //                 hpSuggestion.setInteger(NodeState.CUBE.value);
    //                 // // // // new LEDBounce(Color.kViolet);

    //                 return;
    //             }
    //             hpSuggestion.setInteger(NodeState.CONE.value);
    //             // // // // new LEDBounce(Color.kYellow);
    //             return;
    //         } else if ((nodeStateValues[i][7] != NodeState.NONE.value
    //                 && nodeStateValues[i][8] != NodeState.NONE.value)
    //                 && !partOfCompleteLink(i, 7) && !partOfCompleteLink(i, 8)) {
    //             if (i == 2) {
    //                 hpSuggestion.setInteger(NodeState.CUBE.value);
    //                 // // // // new LEDBounce(Color.kViolet);

    //                 return;
    //             }
    //             hpSuggestion.setInteger(NodeState.CONE.value);
    //             // // // // new LEDBounce(Color.kYellow);
    //             return;
    //         } else if ((nodeStateValues[i][6] != NodeState.NONE.value
    //                 && nodeStateValues[i][8] != NodeState.NONE.value)
    //                 && !partOfCompleteLink(i, 6) && !partOfCompleteLink(i, 7)) {
    //             if (i == 2) {
    //                 hpSuggestion.setInteger(NodeState.CUBE.value);
    //                 // // // // new LEDBounce(Color.kViolet);

    //                 return;
    //             }
    //             hpSuggestion.setInteger(NodeState.CUBE.value);

    //             return;
    //         }

    //     }
    //     // Otherwise, there is no valid reason to choose between cones and cubes. Random
    //     // choice
    //     hpSuggestion.setInteger((int) (Math.random() * 2) + 1);

    // }

    // public void autoQueuePlacement() {
    //     if (queueManualOverride) {
    //         return;
    //     }
    //     if (queuedValue != null) {
    //         nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
    //     }
    //     if (heldObject == HeldObject.CONE) {
    //         // First priority is to achieve coopertition bonus link (all priorities
    //         // automatically go for highest possible)
    //         if (!coopertitionBonusAchieved) {
    //             for (int i = 0; i < 3; i++) {
    //                 int j = 3;
    //                 if ((nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 1] != NodeState.NONE.value) && !partOfCompleteLink(i, j)
    //                         && !partOfCompleteLink(i, j + 1)) {
    //                     queuedValue = new Point(i, j + 2);
    //                     nodeSuperStateValues[i][j + 2] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 } else if (nodeStateValues[i][j + 1] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 2] != NodeState.NONE.value && !partOfCompleteLink(i, j + 1)
    //                         && !partOfCompleteLink(i, j + 2)) {
    //                     queuedValue = new Point(i, j);
    //                     nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 } else if (i == 2 && ((nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 1] != NodeState.NONE.value && !partOfCompleteLink(i, j)
    //                         && !partOfCompleteLink(i, j + 1))
    //                         || (nodeStateValues[i][j + 1] != NodeState.NONE.value
    //                                 && nodeStateValues[i][j + 2] != NodeState.NONE.value
    //                                 && !partOfCompleteLink(i, j + 2) && !partOfCompleteLink(i, j + 1))
    //                         || (nodeStateValues[i][j] != NodeState.NONE.value
    //                                 && nodeStateValues[i][j + 2] != NodeState.NONE.value && !partOfCompleteLink(i, j)
    //                                 && !partOfCompleteLink(i, j + 2)))) {
    //                     for (int k = 0; k < 3; k++) {
    //                         if (nodeStateValues[i][j + k] == NodeState.NONE.value) {
    //                             queuedValue = new Point(i, j + k);
    //                             nodeSuperStateValues[i][j + k] = NodeSuperState.QUEUED.value;
    //                             return;
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //         // Next priority is to complete a link
    //         for (int i = 0; i < 3; i++) {
    //             for (int j = 0; j < 8; j += 3) {
    //                 if ((nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 1] != NodeState.NONE.value) && !partOfCompleteLink(i, j)
    //                         && !partOfCompleteLink(i, j + 1)) {
    //                     queuedValue = new Point(i, j + 2);
    //                     nodeSuperStateValues[i][j + 2] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 } else if (nodeStateValues[i][j + 1] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 2] != NodeState.NONE.value && !partOfCompleteLink(i, j + 1)
    //                         && !partOfCompleteLink(i, j + 2)) {
    //                     queuedValue = new Point(i, j);
    //                     nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 } else if (i == 2 && ((nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 1] != NodeState.NONE.value && !partOfCompleteLink(i, j)
    //                         && !partOfCompleteLink(i, j + 1))
    //                         || (nodeStateValues[i][j + 1] != NodeState.NONE.value
    //                                 && nodeStateValues[i][j + 2] != NodeState.NONE.value
    //                                 && !partOfCompleteLink(i, j + 2) && !partOfCompleteLink(i, j + 1))
    //                         || (nodeStateValues[i][j] != NodeState.NONE.value
    //                                 && nodeStateValues[i][j + 2] != NodeState.NONE.value && !partOfCompleteLink(i, j)
    //                                 && !partOfCompleteLink(i, j + 2)))) {
    //                     for (int k = 0; k < 3; k++) {
    //                         if (nodeStateValues[i][j + k] == NodeState.NONE.value) {
    //                             queuedValue = new Point(i, j + k);
    //                             nodeSuperStateValues[i][j + k] = NodeSuperState.QUEUED.value;
    //                             return;
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //         // Next priority is to make 2/3 link
    //         for (int i = 0; i < 3; i++) {
    //             for (int j = 0; j < 8; j += 3) {
    //                 // Case 1: Cube node in link is filled
    //                 if (nodeStateValues[i][(j / 3) * 3 + 1] != NodeState.NONE.value
    //                         && nodeStateValues[i][j] == NodeState.NONE.value) {
    //                     queuedValue = new Point(i, j);
    //                     nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
    //                     return;
    //                     // Case 2: First cone node in link is filled
    //                 } else if (nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 2] == NodeState.NONE.value) {
    //                     queuedValue = new Point(i, j + 2);
    //                     nodeSuperStateValues[i][j + 2] = NodeSuperState.QUEUED.value;
    //                     return;
    //                     // Case 3: Second cone node in link is filled
    //                 } else if (nodeStateValues[i][j] == NodeState.NONE.value
    //                         && nodeStateValues[i][j + 2] != NodeState.NONE.value) {
    //                     queuedValue = new Point(i, j);
    //                     nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 } else if (i == 2 && ((nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 1] != NodeState.NONE.value)
    //                         || (nodeStateValues[i][j + 1] != NodeState.NONE.value
    //                                 && nodeStateValues[i][j + 2] != NodeState.NONE.value)
    //                         || (nodeStateValues[i][j] != NodeState.NONE.value
    //                                 && nodeStateValues[i][j + 2] != NodeState.NONE.value))) {
    //                     for (int k = 0; k < 3; k++) {
    //                         if (nodeStateValues[i][j + k] == NodeState.NONE.value) {
    //                             queuedValue = new Point(i, j + k);
    //                             nodeSuperStateValues[i][j + k] = NodeSuperState.QUEUED.value;
    //                             return;
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //         // Last priority is to just place wherever empty
    //         for (int i = 0; i < 3; i++) {
    //             for (int j = 0; j < 8; j++) {
    //                 if (nodeStateValues[i][j] == NodeState.NONE.value && (j % 3 == 0 || j % 3 == 2)) {
    //                     queuedValue = new Point(i, j);
    //                     nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 }
    //             }
    //         }
    //         logNoMorePieceSpaceCones.set(true);
    //         if (queuedValue != null) {
    //             nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
    //         }
    //         queuedValue = null;
    //     } else if (heldObject == HeldObject.CUBE) {
    //         // First priority is to achieve coopertition bonus link (all priorities
    //         // automatically go for highest possible)
    //         if (!coopertitionBonusAchieved) {
    //             for (int i = 0; i < 3; i++) {
    //                 int j = 3;
    //                 if ((nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 2] != NodeState.NONE.value)) {
    //                     queuedValue = new Point(i, j + 1);
    //                     nodeSuperStateValues[i][j + 1] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 } else if (i == 2 && ((nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 1] != NodeState.NONE.value)
    //                         || (nodeStateValues[i][j + 1] != NodeState.NONE.value
    //                                 && nodeStateValues[i][j + 2] != NodeState.NONE.value)
    //                         || (nodeStateValues[i][j] != NodeState.NONE.value
    //                                 && nodeStateValues[i][j + 2] != NodeState.NONE.value))) {
    //                     for (int k = 0; k < 3; k++) {
    //                         if (nodeStateValues[i][j + k] == NodeState.NONE.value) {
    //                             queuedValue = new Point(i, j + k);
    //                             nodeSuperStateValues[i][j + k] = NodeSuperState.QUEUED.value;
    //                             return;
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //         // Next priority is to complete a link
    //         for (int i = 0; i < 3; i++) {
    //             for (int j = 0; j < 8; j += 3) {
    //                 if ((nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][j + 2] != NodeState.NONE.value)
    //                         && nodeStateValues[i][j + 1] == NodeState.NONE.value) {
    //                     queuedValue = new Point(i, j + 1);
    //                     nodeSuperStateValues[i][j + 1] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 }
    //             }
    //         }
    //         // Next priority is to make 2/3 link
    //         for (int i = 0; i < 3; i++) {
    //             for (int j = 0; j < 8; j++) {
    //                 if ((nodeStateValues[i][j] != NodeState.NONE.value
    //                         && nodeStateValues[i][(j / 3) * 3 + 1] == NodeState.NONE.value)) {
    //                     queuedValue = new Point(i, (j / 3) * 3 + 1);
    //                     nodeSuperStateValues[i][(j / 3) * 3 + 1] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 }
    //             }
    //         }
    //         // Last priority is to just place wherever empty
    //         for (int i = 0; i < 3; i++) {
    //             for (int j = 1; j < 8; j += 3) {
    //                 if (nodeStateValues[i][j] == NodeState.NONE.value && j % 3 == 1) {
    //                     queuedValue = new Point(i, j);
    //                     nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
    //                     return;
    //                 }
    //             }
    //         }
    //         logNoMorePieceSpaceCubes.set(true);
    //         if (queuedValue != null) {
    //             nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
    //         }
    //         queuedValue = null;
    //     } else {
    //         return;
    //     }
    // }

    @Override
    public void periodic() {

        // lol.set(true);
    //     //check if something was selected by touchscreen
        // for (int i = 0; i < nodes.length; i++) {
		// 	for (int j = 0; j < nodes[i].length; j++) {
		// 		if (nodes[i][j].getInteger(0) == 6) {
		// 			switch (nodeSuperStateValues[i][j]) {
		// 				case 0:
		// 					changeTarget(i, j);
		// 					queuePlacement();
		// 					break;
		// 				case 3:
		// 					queuePlacement();
		// 					break;
		// 				case 4:
		// 					queueManualOverride = false;
		// 					setPiece();
		// 					break;
		// 			}
		// 		}
        //         if (nodes[i][j].getInteger(0)== 7) {
        //             nodeStateValues[i][j]=0;
        //             nodeSuperStateValues[i][j]=0;
        //         }
		// 	}
		// }
    //     // Check which links are complete
    //     for (int i = 0; i < 3; i++) {
    //         for (int j = 1; j < 8;) {
    //             if (nodeStateValues[i][j] != NodeState.NONE.value && nodeStateValues[i][j + 1] != NodeState.NONE.value
    //                     && nodeStateValues[i][j - 1] != NodeState.NONE.value) {
    //                 linkComplete[7 * i + (j - 1)] = true;
    //                 if ((7 * i + j) % 7 == 1) {
    //                     linkComplete[7 * i + j] = false;
    //                     linkComplete[7 * i + j + 1] = false;
    //                 } else if ((7 * i + j) % 7 == 4) {
    //                     linkComplete[7 * i + j] = false;
    //                     linkComplete[7 * i + j + 1] = false;
    //                     linkComplete[7 * i + j - 2] = false;
    //                     linkComplete[7 * i + j - 3] = false;
    //                 } else if ((7 * i + j) % 7 == 0) {
    //                     linkComplete[7 * i + j - 2] = false;
    //                     linkComplete[7 * i + j - 3] = false;
    //                 }
    //                 j = ((j - 1) / 3 + 1) * 3 + 1;

    //             } else {
    //                 linkComplete[7 * i + (j - 1)] = false;
    //                 j++;
    //             }
    //         }
    //     }
    //     if (heldObject != heldObjectIn) {
    //         for (int i = 0; i < 3; i++) {
    //             for (int j = 0; j < 9; j++) {
    //                 if (nodeSuperStateValues[i][j] == NodeSuperState.INVALID.value
    //                         || nodeSuperStateValues[i][j] == NodeSuperState.QUEUED.value) {
    //                     nodeSuperStateValues[i][j] = NodeSuperState.NONE.value;
    //                 }
    //             }
    //         }
    //         heldObject = heldObjectIn;
    //         autoQueuePlacement();
    //     }
    //     if (suggestManualOverride && heldObject != HeldObject.NONE) {
    //         suggestManualOverride = false;
    //     }
    //     for (int i = 0; i < 3; i++) {
    //         if ((nodeStateValues[i][3] != NodeState.NONE.value && nodeStateValues[i][4] != NodeState.NONE.value
    //                 && nodeStateValues[i][5] != NodeState.NONE.value)) {
    //             coopertitionBonusAchieved = true;
    //             break;
    //         }
    //     }
    //     if (heldObject == HeldObject.CONE) {
    //         for (int i = 0; i < 2; i++) {
    //             for (int j = 1; j < 8; j += 3) {
    //                 nodeSuperStateValues[i][j] = NodeSuperState.INVALID.value;
    //             }
    //         }
    //     } else if (heldObject == HeldObject.CUBE) {
    //         for (int i = 0; i < 2; i++) {
    //             for (int j = 0; j < 9; j++) {
    //                 if (j % 3 == 0 || j % 3 == 2) {
    //                     nodeSuperStateValues[i][j] = NodeSuperState.INVALID.value;
    //                 }
    //             }
    //         }
    //     } else {
    //         for (int i = 0; i < 2; i++) {
    //             for (int j = 0; j < 9; j++) {
    //                 if (nodeSuperStateValues[i][j] == NodeSuperState.INVALID.value) {
    //                     nodeSuperStateValues[i][j] = NodeSuperState.NONE.value;
    //                 }
    //             }
    //         }
    //     }
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 4; j++) {
                if (nodeSuperStateValues[i][j] == NodeSuperState.NONE.value) {
                    integratedNodeValues[i][j]=nodeStateValues[i][j];
                } else {
                    integratedNodeValues[i][j]=nodeSuperStateValues[i][j];
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