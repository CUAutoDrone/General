# Optimizing the PID Control Loop for Quadcopters in Python
* *Chris Gyurgyik, Cornell University Aerial Robotics*

___Please note: I have not tested any of this, simply because I don't have access to sensors, or drones. I can tell you that the output does spit out numbers, and the module compiles. So that's a start.___

Detailed below is my informal approach as to how I could help achieve a more fluid PID control loop designed in Python. I will be documenting this continuously as I get more research done. Presented below is the set of code before I touched it:

```
        #Initialize PID Control Stuff
	is_first_loop = 0
	err_sum = 0
	sys_time = pi.get_current_tick()
 	#flight loop
	while(ARM == 1):
 		#Get Delta Time
		sys_time_new = pi.get_current_tick()
		dt = (sys_time_new-sys_time)/1e6
		#correct for rollover
		if dt<0:
			dt=0
		sys_time = sys_time_new
 		# #Maps control input into angles
		control_angles = map_control_input()
 		#Get accelerometer and gyroscope data and compute angles
		accel_data = get_acceleration_data(pi,MPU6050_handle)-acc_offsets
		gyro_data = get_gyroscope_data(pi,MPU6050_handle)-gyro_offsets
		euler_state = calculate_angles(pi,accel_data,gyro_data,dt,euler_state)
 		#Compute errors in pitch and roll and yawrate
		#err = np.array([control_angles[0]-euler_state[0],
		# 				control_angles[1]-euler_state[1],
		# 				control_angles[2]-gyro_data[2]])
		err = np.array([0-euler_state[0],
		 				0-euler_state[1],
		 				0])
 		#compute error integral
		err_sum = err_sum+err
		#PID law
		if is_first_loop == 0:
		 	u = np.multiply(Kp,err)+np.multiply(Ki,dt*err_sum)
			is_first_loop = 1
		else:
			u = np.multiply(Kp,err)+np.multiply(Ki,dt*err_sum)+np.multiply(Kd,(err-prev_err)/dt)
		prev_err = err
 		#Map controls into vector
		ctrl = np.array([control_angles[3],u[0],u[1],u[2]])
 		#Map control angles into output signal and set motors
```

## Step 0: What is PID Control?
PID stands for Proportional, Integral, Derivative. It helps in tuning the vehicle to ensure that corrective responses are applied in response to a control function. In the world of quadcopters, it determines how fast the motors should spin in response to outside forces. To do this, it corrects errors between measured values from sensors and the desired rotation speeds. The P term looks at present errors, I term looks at the forces over time, and the D term looks at future errors, or how fast you are approaching your desired rotation speeds. More information can be found at the links below.
```
https://en.wikipedia.org/wiki/PID_controller
https://oscarliang.com/quadcopter-pid-explained-tuning/
```

## Step 0.5: Making the code readable and approachable
Part of my task was giving the drone flight controller a more "Object Oriented" approach, rather than just all the code to each sensor, motor, etc. in one module. This comes into play since the PID loop is obviously part of the flight controller. First, depicted below is my updated_PID method:

```
    # Updates current PID
    def compute_PID(self, pi):
        # get delta time
        sys_time_new = pi.get_current_tick()
        dt = (sys_time_new - self.imu.sys_time) / 1e6

        # correct for rollover
        if dt < 0:
            dt = 0
        self.imu.sys_time = sys_time_new

        # Maps control input into angles
        control_angles = self.receiver.map_control_input()

        # Calculate Euler angles
        # TODO: see IMU calculate_angles method; possible bug with dt
        # TODO: fix: replace dt with self.sys_time
        self.imu.calculate_angles(pi, dt)

        # Compute errors in pitch and roll and yaw rate
        error = np.array([0 - self.imu.euler_state[0],
                          0 - self.imu.euler_state[1],
                          0])

        # compute error integral
        self.imu.error_sum += error

        # computer delta error
        delta_error = error - self.imu.prev_error

        # PID law
        if dt > 0:
            u = np.multiply(self.Kp, error) + np.multiply(self.Ki, dt * self.imu.error_sum) + \
                np.multiply(self.Kd, delta_error / dt)
        else:
            u = np.multiply(self.Kp, error) + np.multiply(self.Ki, dt * self.imu.error_sum)

        self.imu.prev_error = error

        # Map controls into vector
        ctrl = np.array([control_angles[3], u[0], u[1], u[2]])

        wm = Motor.map_motor_output(ctrl)
        print(wm)

        Motor.set_motor_pulse(pi, self.motor.MOTOR1, wm[0])
        Motor.set_motor_pulse(pi, self.motor.MOTOR2, wm[1])
        Motor.set_motor_pulse(pi, self.motor.MOTOR3, wm[2])
        Motor.set_motor_pulse(pi, self.motor.MOTOR4, wm[3])
```
While not detailed too carefully, there are still a couple minor bugs involved with the compute_angles method, though this will focus on the PID loop. Other things I looked at to optimize the code included the removal of numpy.pow(x,2) to just x*x, which is much faster. To give a clearer picture of the OOP approach, I made a class for each of the following: flight controller, IMU, motor, and receiver. 

Here is the loop where the PID loop occurs:
```
# obtains current system time for PID control
            self.imu.sys_time = pi.get_current_tick()

            # flight loop
            while self.receiver.ARM is True and self.armed is True:
                # PID loop
                self.compute_PID(pi)
```

## Step 1: Calling the PID loop in regular intervals
This is where the PID loop optimization actually begins. One of the big steps to improve consistency is to allow the PID loop to be called in regular intervals, where your delta time is always equal to some constant. This also allows for your derivative and integral calculations to be simplified, although I won't follow through with this yet. As far as determining the delta time constant, it should be based on how long the average PID loop takes for your vehicle (which is hopefully in the 100 - 1000 Hz range). Here is the updated code for the PID loop:

```
    # Updates current PID
    def compute_PID(self):

        # begin the thread
        threading.Timer(self.imu.sample_time, self.compute_PID).start()

        # used to determine how long one iteration of the method takes
        start_time = self.pi.get_current_tick()

        # Maps control input into angles
        control_angles = self.receiver.map_control_input()

        # Calculate Euler angles
        self.imu.calculate_angles(self.pi)

        # Compute errors in pitch and roll and yaw rate
        error = np.array([0 - self.imu.euler_state[0],
                          0 - self.imu.euler_state[1],
                          0])

        # compute error integral
        self.imu.error_sum += error

        # computer delta error
        delta_error = error - self.imu.prev_error

        # PID law
        if self.imu.sample_time > 0:
            u = np.multiply(self.Kp, error) + np.multiply(self.Ki, self.imu.sample_time * self.imu.error_sum) + \
                np.multiply(self.Kd, delta_error / self.imu.sample_time)
        else:
            u = np.multiply(self.Kp, error) + np.multiply(self.Ki, self.imu.sample_time * self.imu.error_sum)

        self.imu.prev_error = error

        # Map controls into vector
        ctrl = np.array([control_angles[3], u[0], u[1], u[2]])

        wm = Motor.map_motor_output(ctrl)
        print(wm)

        Motor.set_motor_pulse(self.pi, self.motor.MOTOR1, wm[0])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR2, wm[1])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR3, wm[2])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR4, wm[3])

        # time of end of the method
        end_time = self.pi.get_current_tick()
        self.imu.actual_time_length_of_PID_loop = (start_time - end_time) / 1e6

```

As well as the flight loop:
```
           # flight loop
            while self.receiver.ARM is True and self.armed is True:
                self.compute_PID()
```

I initially thought about using the approach of just having an if statement that asks whether the current delta time is equal to or greater than my sample_time. Instead, my approach uses threading, where the method updatePID is called over each sample_time interval. I've also included a actual_time_length_of_PID_loop (temporary name) variable to determine how much time each method call takes.

I was able to slightly test this, but that did not involve any real sensor readings so further testing is still required.

Sources:
```
http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-sample-time/
http://support.motioneng.com/downloads-notes/tuning/scaling_pid_param.htm
```

## Step 2: Live tuning changes
One thing I found that may be useful is to allow for live tuning changes. In other words, change the P value, I value, and D value while the drone is flying. Because the I term looks at forces over time, it will a cause an exaggerated change (since it unnecesarrily include past errors), and thus we want to eliminate that. In the next set of code, I made some small optimizations, but the main focus is creating an I_term variable that will replace the error sum. If Ki sees no changes, I_term will act the same as error_sum. Otherwise, I_term will allow Ki to only affect present errors rather than errors that occurred in the past, as we want. Here is the code:

```
    # Updates current PID
    def compute_PID(self):

        # begin the thread
        threading.Timer(self.imu.sample_time, self.compute_PID).start()

        # used to determine how long one iteration of the method takes
        start_time = self.pi.get_current_tick()

        # Maps control input into angles
        control_angles = self.receiver.map_control_input()

        # Calculate Euler angles
        self.imu.calculate_angles(self.pi)

        # Compute errors in pitch and roll and yaw rate
        error = np.array([0 - self.imu.euler_state[0],
                          0 - self.imu.euler_state[1],
                          0])

        #I_term
        self.imu.I_term += np.multiply(self.Ki, error)

        # compute d error
        d_error = error - self.imu.prev_error

        # PID Law
        u = np.multiply(self.Kp, error) + self.imu.I_term + \
            np.multiply(self.Kd, d_error / self.imu.sample_time)

        self.imu.prev_error = error

        # Map controls into vector
        ctrl = np.array([control_angles[3], u[0], u[1], u[2]])

        wm = Motor.map_motor_output(ctrl)
        print(wm)

        Motor.set_motor_pulse(self.pi, self.motor.MOTOR1, wm[0])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR2, wm[1])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR3, wm[2])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR4, wm[3])

        # time of end of the method
        end_time = self.pi.get_current_tick()
        self.imu.actual_time_length_of_PID_loop = (start_time - end_time) / 1e6
```
For a clearer description with pictures, please look at the following source where I found the majority of my information:
```
http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
```
## Step 3: Limiting PID output
In this step we're looking to restrict the output of the PID gains. If the PWM output only accepts values from 0 - 255, there is no reason why the PID gains should go above or below that. Anything outside this external limit can cause what is known as "windup reset," and causes the integral to continue to sum to enormous numbers. Then a lag occurs as it winds back down. Our goal here is to prevent the integral windup, as well as any output values that may break our bounds. Thus, we will add a function that will do this for both the I_term and the output values. The code is presented below for the PID loop:
```
    # Updates current PID
    def compute_PID(self):

        # begin the thread
        threading.Timer(self.imu.sample_time, self.compute_PID).start()

        # used to determine how long one iteration of the method takes
        start_time = self.pi.get_current_tick()

        # Maps control input into angles
        control_angles = self.receiver.map_control_input()

        # Calculate Euler angles
        self.imu.calculate_angles(self.pi)

        # Compute errors in pitch and roll and yaw rate
        error = np.array([0 - self.imu.euler_state[0],
                          0 - self.imu.euler_state[1],
                          0])

        # I_term replaces the error sum since it allows for smoother live PID tunings
        self.imu.I_term += np.multiply(self.Ki, error)

        # compute d error
        d_error = error - self.imu.prev_error

        # PID Law
        output = np.multiply(self.Kp, error) + self.imu.I_term + \
            np.multiply(self.Kd, d_error / self.imu.sample_time)

        # ensures that the output also falls within output limitations,
        # as well as clamps the I_term after the output has been computed
        output = self.imu.check_output_limitations(output[0], output[1], output[2])

        self.imu.prev_error = error

        # Map controls into vector
        ctrl = np.array([control_angles[3], output[0], output[1], output[2]])

        wm = Motor.map_motor_output(ctrl)
        print(wm)

        Motor.set_motor_pulse(self.pi, self.motor.MOTOR1, wm[0])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR2, wm[1])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR3, wm[2])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR4, wm[3])

        # time of end of the method
        end_time = self.pi.get_current_tick()
        self.imu.actual_time_length_of_PID_loop = (start_time - end_time) / 1e6
```
The code for the function is presented here:
```
    # checks the limitations on each variable to avoid reset windup
    def check_output_limitations(self, a, b, c):

        if a > self.output_max:
            self.I_term[0] -= a - self.output_max
            a = self.output_max
        elif a < self.output_min:
            self.I_term[0] += self.output_min - a
            a = self.output_min

        if b > self.output_max:
            self.I_term[1] -= b - self.output_max
            b = self.output_max
        elif b < self.output_min:
            self.I_term[1] += self.output_min - b
            b = self.output_min

        if c > self.output_max:
            self.I_term[2] -= c - self.output_max
            c = self.output_max
        elif c < self.output_min:
            self.I_term[2] += self.output_min - c
            c = self.output_min

        return np.array([a,b,c])
```
One of the comments in the brettbeauregard.com source provided below mentioned that the integral should be clamped after the output is computed but before it is clamped. This reduces the windup to nearly zero. This is exactly how my function proceeds. Check out a better explanation and great resource(s) on windup reset:
```
http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/
https://www.cds.caltech.edu/~murray/wiki/images/6/6f/Recitation_110_nov_17.pdf
```

## Step 4: Getting rid of derivative kick
Since error is equal to the setpoint minus the input values, we can see large spikes in values if the setpoint is changed at all. This is true because there is an "instantaneous" change in error, and thus in practice we can see large numbers spit out. A way to fix this is manipulating the equation for d_error = d_setpoint - d_input. If our d_setpoint is constant, we get d_error = - d_input, and this holds true until our d_setpoint is no long constant. We can then change the code so that we read the calculate d_input instead of d_error, and in our PID law, subtract (Kd * d_input/sample_time). This will replace what was previously there, adding (Kd * d_error/sample_time). I'm not entirely sure how much these spikes will affect the drone PID loop, but I figured the calculation processing requirements are nearly identical so it doesn't hurt to add this safety measure. Here is the new code:
```
    # Updates current PID
    def compute_PID(self):

        # begin the thread
        threading.Timer(self.imu.sample_time, self.compute_PID).start()

        # used to determine how long one iteration of the method takes
        start_time = self.pi.get_current_tick()

        # Maps control input into angles
        control_angles = self.receiver.map_control_input()

        # Calculate Euler angles
        self.imu.calculate_angles(self.pi)

        # Compute errors in pitch and roll and yaw rate
        # error = setpoint - input
        error = np.array([0 - self.imu.euler_state[0],
                          0 - self.imu.euler_state[1],
                          0])

        # I_term replaces the error sum since it allows for smoother live PID tunings
        self.imu.I_term += np.multiply(self.Ki, error)

        # compute d input
        # d_input = input - last input
        d_input = np.array([self.imu.euler_state[0]-self.imu.prev_d_input[0],
                            self.imu.euler_state[1]-self.imu.prev_d_input[1], 0])

        # PID Law
        output = np.multiply(self.Kp, error) + self.imu.I_term - \
                 np.multiply(self.Kd, self.imu.prev_d_input / self.imu.sample_time)

        # ensures that the output also falls within output limitations,
        # as well as clamps the I_term after the output has been computed
        output = self.imu.check_output_limitations(output[0], output[1], output[2])

        self.imu.prev_d_input = d_input

        # Map controls into vector
        ctrl = np.array([control_angles[3], output[0], output[1], output[2]])

        wm = Motor.map_motor_output(ctrl)
        print(wm)

        Motor.set_motor_pulse(self.pi, self.motor.MOTOR1, wm[0])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR2, wm[1])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR3, wm[2])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR4, wm[3])

        # time of end of the method
        end_time = self.pi.get_current_tick()
        self.imu.actual_time_length_of_PID_loop = (start_time - end_time) / 1e6
```
Sources: 
```
http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
https://barela.wordpress.com/2013/07/19/pid-controller-basics-eliminating-derivative-kick/
```
