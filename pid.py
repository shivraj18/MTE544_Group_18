# PID

from rclpy.time import Time
from utilities import Logger

# Controller type
P   = 0 # poportional
PD  = 1 # proportional and derivative
PI  = 2 # proportional and integral
PID = 3 # proportional, integral, derivative

class PID_ctrl:
    
    def __init__(self, type_, kp=1.2,kv=0.8,ki=0.2, history_length=3, filename_="errors.csv"):
        
        # Data for the controller
        self.history_length=history_length
        self.history=[]
        self.type=type_

        # Controller gains
        self.kp=kp    # proportional gain
        self.kv=kv    # derivative gain
        self.ki=ki    # integral gain
        
        self.logger=Logger(filename_)
        # Remeber that you are writing to the file named filename_ or errors.csv the following:
            # error, error_dot, error_int and time stamp
    
    def update(self, stamped_error, status):
        
        if status == False:
            self.__update(stamped_error)
            return 0.0
        else:
            return self.__update(stamped_error)

        
    def __update(self, stamped_error):
        
        latest_error=stamped_error[0]
        stamp=stamped_error[1]
        
        self.history.append(stamped_error)        
        
        if (len(self.history) > self.history_length):
            self.history.pop(0)
        
        # If insufficient data points, use only the proportional gain
        if (len(self.history) != self.history_length):
            return self.kp * latest_error
        
        # Compute the error derivative
        dt_avg=0
        error_dot=0
        
        for i in range(1, len(self.history)):
            
            t0=Time.from_msg(self.history[i-1][1])
            t1=Time.from_msg(self.history[i][1])
            
            dt=(t1.nanoseconds - t0.nanoseconds) / 1e9
            
            dt_avg+=dt

            # use constant dt if the messages arrived inconsistent
            # for example dt=0.1 overwriting the calculation          
            
            # TODO Part 5: calculate the error dot 
            # PART 5 CHRISTIAN ADDED CODE -----------------------------------------------------------------------------------
            error_dot+= (latest_error - stamp) / dt # using stamp may be wrong but this should be right! 
            # PART 5 CHRISTIAN ADDED CODE END -----------------------------------------------------------------------------------
            
        error_dot/=len(self.history)
        dt_avg/=len(self.history)
        
        # Compute the error integral
        sum_=0
        for hist in self.history:
            # TODO Part 5: Gather the integration
            # PART 5 CHRISTIAN ADDED CODE -----------------------------------------------------------------------------------
            sum_+=latest_error # Are we accessing the error correctly here?
            # PART 5 CHRISTIAN ADDED CODE -----------------------------------------------------------------------------------
            pass
        
        error_int=sum_*dt_avg
        
        # TODO Part 4: Log your errors
        # ADDED CODE -----------------------------------------------------------------------------------

        self.logger.log_values( stamped_error[0], error_dot, error_int, stamped_error[1])
        
        # ADDED CODE END -------------------------------------------------------------------------------

        # TODO Part 4: Implement the control law of P-controller
        if self.type == P:
            # error = [[calculate_linear_error()],[calculate_angular_error()]]
            return self.kp * stamped_error[0] 
        
        # TODO Part 5: Implement the control law corresponding to each type of controller
        # PART 5 CHRISTIAN ADDED CODE IN THIS SECTION ----------------------------------------------------------------------------------
        # All I did here was remove the lines that said "pass" and change kp to self.kp, and change "kd" to "kv" for derivative gain
        # Do we need to implement anything else? 
        elif self.type == PD:
            return (self.kp * stamped_error[0]) + (self.kv * error_dot) 
        
        elif self.type == PI:
            return (self.kp * stamped_error[0]) + (self.ki * error_int) 
        
        elif self.type == PID:
            return (self.kp * stamped_error[0]) + (self.ki * error_int) + (self.kv * error_dot)  