package frc.robot.helper;

import java.util.Map;
import java.util.HashMap;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterLookUp // changes distance to speed and hood angle
{
    private Double distanceFactor = 1.0; // allows for easy minor adjustments of shooter if needed (multiplier)
    private NetworkTableInstance ntInst = null;
    private NetworkTable ntTble = null;
    private Map<Double, ShooterValueSet> valueTable = null; // table that assocaited distance to hood and speed values
    
    public ShooterLookUp()
    {
        ntInst = NetworkTableInstance.getDefault();
        ntTble = ntInst.getTable("ContourTable"); // gets the networktable where the target information is stored
        ntTble.getEntry("TargetDistanceTest").setDouble(0);
        valueTable = new HashMap<Double, ShooterValueSet>();


        fillTableVaules();
    }

    private void fillTableVaules() // adds the key value pairs to the lookup table <KEY, HoodAngle, ShooterRPM>
    {
        addTableValue(1.0, 10.0, 4010.0);
        addTableValue(2.0, 20.0, 4020.0);
        addTableValue(3.0, 30.0, 4030.0);
        addTableValue(4.0, 40.0, 4040.0);
        addTableValue(5.0, 50.0, 4050.0);
        addTableValue(6.1, 60.0, 4061.0);
        addTableValue(7.5, 70.0, 4075.0);
    }

    private void addTableValue(Double Key, Double HoodAngle, Double ShooterSpeed) // appedns a set of values to the look up table system
    {
        valueTable.put(Key, new ShooterValueSet(HoodAngle, ShooterSpeed));
    }

    public ShooterValueSet getCurrentValues(Boolean Interpolate) // gets the hood and shooter speed values; interpolate ~ if true finds values between two existing
    {
        //-- remove line below & comment when value table is complete Double distanceFromTarget = ntTble.getEntry("TargetDistance").getDouble(0);
        Double distanceFromTarget = ntTble.getEntry("TargetDistanceTest").getDouble(0);


        if(Interpolate){
            ShooterValueKey positveValueKey = findClosestPostiveKey(distanceFromTarget);
            ShooterValueKey negativeValueKey = findClosestNegativeKey(distanceFromTarget);

            return interpolate(positveValueKey, negativeValueKey);
        }

        ShooterValueKey valueKey = findClosestKey(distanceFromTarget * distanceFactor);
        return valueTable.get(valueKey.key); // returns the values that match the key
    }

    public ShooterValueSet getCustomValues(Double Distance, Boolean Interpolate) // interpolate ~ if true finds values between two existing
    {
        if(Interpolate){
            ShooterValueKey positveValueKey = findClosestPostiveKey(Distance);
            ShooterValueKey negativeValueKey = findClosestNegativeKey(Distance);

            return interpolate(positveValueKey, negativeValueKey);
        }
       
        ShooterValueKey valueKey = findClosestKey(Distance);
        return valueTable.get(valueKey.key); // returns the values that match the key
    }

    private ShooterValueKey findClosestKey(Double Distance)
    {
        Set<Double> keys = valueTable.keySet(); // gets all valid keys for the value table
        ShooterValueKey[] keyArray = (ShooterValueKey[]) keys.toArray();
        ShooterValueKey closestKey = new ShooterValueKey(0.0, -1.0); // the keys that is the closest to the distance

        for(ShooterValueKey key: keyArray) // goes through each key and find the differnce between the key and distance
        {
            if(Math.abs(Distance - key.key) < closestKey.devationFromDistance | closestKey.devationFromDistance < 0)
            {
                closestKey.devationFromDistance = Math.abs(Distance - key.key);
                closestKey.key = key.key;
            }
        }

        return closestKey;
    }

    private ShooterValueKey findClosestPostiveKey(Double Distance) // key is greater than distance
    {
        Set<Double> keys = valueTable.keySet(); // gets all valid keys for the value table
        Double[] keyArray = (Double[]) keys.toArray();
        ShooterValueKey closestKey = new ShooterValueKey(0.0, -1.0); // the keys that is the closest to the distance

        for(Double key: keyArray) // goes through each key and find the differnce between the key and distance
        {
            if((Distance - key <= 0) && (Math.abs(Distance - key) < closestKey.devationFromDistance | closestKey.devationFromDistance < 0))
            {
                closestKey.devationFromDistance = Math.abs(Distance - key);
                closestKey.key = key;
            }
        }

        return closestKey;
    }
    
    private ShooterValueKey findClosestNegativeKey(Double Distance) // key is less than distance
    {
        Set<Double> keys = valueTable.keySet(); // gets all valid keys for the value table
        Double[] keyArray = (Double[]) keys.toArray();
        ShooterValueKey closestKey = new ShooterValueKey(0.0, -1.0); // the keys that is the closest to the distance

        for(Double key: keyArray) // goes through each key and find the differnce between the key and distance
        {
            if((Distance - key >= 0) && (Math.abs(Distance - key) < closestKey.devationFromDistance | closestKey.devationFromDistance < 0))
            {
                closestKey.devationFromDistance = Math.abs(Distance - key);
                closestKey.key = key;
            }
        }

        return closestKey;
    }

    private ShooterValueSet interpolate(ShooterValueKey Key1, ShooterValueKey Key2) // creates a new set of values by taking wieghted average
    {
        Double wieght1 = Key1.devationFromDistance / (Key1.devationFromDistance + Key2.devationFromDistance);
        Double wieght2 = 1 - wieght1; // creates the weights for taking a wieghed average

        ShooterValueSet valueSet1 = valueTable.get(Key1.key);
        ShooterValueSet valueSet2 = valueTable.get(Key2.key);

        double hoodAngle = takeWieghtedAverage(valueSet1.hoodAngle, valueSet2.hoodAngle, wieght1, wieght2);
        double shooterRPM = takeWieghtedAverage(valueSet1.shooterRPM, valueSet2.shooterRPM, wieght1, wieght2);

        return new ShooterValueSet(hoodAngle, shooterRPM);
    }

    private Double takeWieghtedAverage(Double Value1, Double Value2, Double Wieght1, Double Wieght2) // takes the wieghted average of two values
    {
        return (((Value1 * Wieght1) + (Value2 * Wieght2)) / (Wieght1 + Wieght2));
    }
}