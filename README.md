# the-cube

Monorepo for the full self-balancing reaction wheel project. The goal is to build a cube that can balance on two axes using a PID / PD-based controller, and later eventually replace the PID / PD with a CNN.
This is an attempt to Combine embedded, AI and control system. 

## Phase 1
Getting reliable read and writes of the GY-521 sensor and the Nidec motor. 

![The Cube prototype](img/image1.png)

### Sensor readings
GY-521 readings and prints:
- raw acceleration
- gyroscope data
- tilt angle

### 2. Test the motor separately
Verify that the motor can:
- spin forward
- spin backward
- stop

### 3. Connect sensor data to motor
Once the sensor and motor work independently:
- tilt forward -> motor spins in one direction
- tilt backward -> motor spins in the other direction



