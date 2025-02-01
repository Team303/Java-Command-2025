package frc.robot.util;

import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ReefState implements Sendable {
   private long[] values;

   public ReefState(long L1, long L2, long L3, long L4) {
      this.values = new long[]{L1,L2,L3,L4};
   }

   public long[] getState() {
      return values;
   }

   public void setState(long[] values) {
      this.values=values;
   }

   public Map<String, Object> asMap() {
      return Map.of("L1", values[0], "L2", values[1], "L3", values[2], "L4", values[3]);
   }

   @Override
   public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("ReefState");
      builder.addIntegerArrayProperty("State",this::getState,this::setState);

   }
}