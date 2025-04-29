class Bottle():
    def __init__(self, color, x, y, theta):
        self.color = color
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = 0
        if x < .1:
            self.status = "ready"
        else:
            self.status = "inFrame"

    def __str__(self):
        return f'position: ({round(self.x,2)}, {round(self.y,2)}), velocity: {self.velocity} \n {self.status}'

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

    def same_as(self, other):
        return other.color == self.color and abs(other.get_x() - self.x) <= 0.1 and abs(other.get_y() - self.y) <= 0.1
            
    def update(self, old, timestep = 1/30):
        if self.same_as(old):
            #self bottle is kept, old bottle gets deleted
            old_x, old_y = old.get_pos()
            x = self.x
            alpha = 0.5 #lp filter constant
            new_v = (x-old_x)/timestep
            self.velocity = alpha*old.get_velocity() + (1-alpha)*new_v

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
        