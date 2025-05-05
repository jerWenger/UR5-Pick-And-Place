import math

class Bottle():
    def __init__(self, color, x, y, theta):
        self.color = color #might start using a probability vector for color
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = -0.15
        self.uncertainty = 1 #in meters, ideally as low as possible
        self.status = "inFrame"

    def __str__(self):
        return f'position: ({round(self.x,2)}, {round(self.y,2)}), velocity: {self.velocity} \n uncertainty: {self.uncertainty} {self.status}'

    def get_status(self):
        return self.status
    
    def get_color(self):
        return self.color

    def get_pos(self):
        return self.x, self.y
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def set_x(self, xval):
        self.x = xval

    def set_y(self, yval):
        self.y = yval
    
    def get_velocity(self):
        return self.velocity
    
    def set_velocity(self,v):
        self.velocity = v

    def get_uncertainty(self):
        return self.uncertainty

    def same_as(self, other):
        return other.color == self.color and abs(other.get_x() - self.x) <= 0.1 and abs(other.get_y() - self.y) <= 0.1
            
    def update(self, old, dt = 0.1):
        if self.same_as(old):
            #self bottle is kept, old bottle gets deleted
            old_x, old_y = old.get_pos()
            old_v = old.get_velocity()
            x = self.x

            alpha = 0.5 #lp filter constant
            new_v = (x-old_x)/dt
            self.velocity = .4*old_v + .6*new_v

            beta = 0.6 #position filter constant 
            est_x = old_x + old_v*dt  #expected position based on previous velocity
            #absolute distance between expected position and measure position in m
            diff = math.sqrt((est_x-self.x)**2 + (old_y-self.y)**2)
            self.uncertainty = diff*alpha + old.get_uncertainty()*(1-alpha)
            self.x = est_x*beta + self.x*(1-beta)
            self.y = old_y*beta + self.y*(1-beta)
            if self.uncertainty < .02 and self.x <= .35:
                self.status = "ready"

    def step_pos(self):
        newx = self.x + self.velocity*.1
        self.x = newx
    
    #not used
    def calculate_avg_vel(self, v_arr):
        if len(v_arr)<=1:
            return 0
        else:
            values = [v_arr[i]-v_arr[i-1] for i in range(1,len(v_arr))]
            return sum(values)/len(values)
        