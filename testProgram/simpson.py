def get_angle_pitch(x_newrate,dt):
    
    pitch_mem[0] = x_newrate
    pitch_mem[1] = pitch_mem[0]
    
    if((x_count % 2) == 1):
        pitch_mem[1] *= 4
        
    else: 
        pitch_mem[1] *= 2
    
    pitch_total += pitch_mem[1]    
    
    pitch_angle = (dt/3) *(pitch_total+x_newrate)
    x_count += 1
    
    return pitch_angle

def get_angle_roll(y_newrate,dt):
    
    roll_mem[0] = y_newrate
    roll_mem[1] = roll_mem[0]
    
    if((y_count % 2) == 1):
        roll_mem[1] *= 4
        
    else: 
        roll_mem[1] *= 2
    
    roll_total += roll_mem[1]    
    
    roll_angle = (dt/3) *(roll_total+y_newrate)
    y_count += 1
    
    return roll_angle

def get_angle_yaw(z_newrate,dt):
    
    yaw_mem[0] = z_newrate
    yaw_mem[1] = yaw_mem[0]
    
    if((z_count % 2) == 1):
        roll_mem[1] *= 4
        
    else: 
        yaw_mem[1] *= 2
    
    yaw_total += yaw_mem[1]    
    
    yaw_angle = (dt/3) *(yaw_total+z_newrate)
    z_count += 1
    
    return yaw_angle


dt = nowTime - lastReadTime
x_count = 1
y_count = 1
z_count = 1

pitch_mem=[0,0]
roll_mem=[0,0]
yaw_mem=[0,0]
 
angle_pitch=0
angle_roll=0
angle_yaw=0

pitch_total=0
roll_total=0
yaw_total=0

get_angle_pitch()
get_angle_roll()
get_angle_yaw()

