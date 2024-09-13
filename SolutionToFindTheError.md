# Solution to Find The Error

## Solution to Error #1
The error with this line is the person who wrote this forgot to use "AnalogOperation." inside of the brackets. The correct answer would look like so:
```java
this.driver.getAnalog(AnalogOperation.ArmShoulderAdjustment);
``` 

## Solution to Error #2
The error with this line is the person who wrote this had not used the name of the tuning constant, and when using anything with tuning constants, we always use those instead of raw values.
```java
TuningConstants.TUNING_CONSTANT;
```

## Solution to Error #3
The error with this line is the person who wrote this did not use the phrase "this." where it needed to be done. "this." makes sure to access the variable inside the file, and it most commonly is used within our code, or we would reference the class we would get it from, like "class1318.variable"
```java
wristGoal.updatePosition(this.currentDesiredWristPosition);
```