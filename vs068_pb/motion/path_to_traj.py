import vs068_pb.config as config
import numpy as np

''' https://www.researchgate.net/publication/328233168_An_Efficient_and_Time-Optimal_Trajectory_Generation_Approach_for_Waypoints_Under_Kinematic_Constraints_and_Error_Bounds?enrichId=rgreq-ac11d1bb244c7f1322909cbcdd1e75ab-XXX&enrichSource=Y292ZXJQYWdlOzMyODIzMzE2ODtBUzo2ODA4MzY2MTc3NTY2NzNAMTUzOTMzNTUwNzU2Mw%3D%3D&el=1_x_2&_esc=publicationCoverPdf '''

def define_trajectory_profile_quad(num_points=10, t_max=5.0, maximum=1):
    #return [maximum - x**2 for x in range(num_points)]

    return [maximum*((4*(t_max*(x)-(x)**2))/(t_max**2)) for x in np.linspace(0, t_max, num=num_points+1)]

def velocity_from_points(points, tick=0.1, start=0.0, end=0.0):
    velocity = [start]

    for point_num in range(1,len(points)):
        velocity.append((points[point_num]-points[point_num-1])/tick)
    
    return velocity


def path_to_traj(path, max_vels=config.vel_lims, t_dur=5.0):
    trajectory = dict()

    path_vels = np.transpose([define_trajectory_profile_quad(len(path), maximum=m, t_max=t_dur) for m in max_vels]).tolist()
    #print(path_vels)
    for point_num in range(len(path)):
        trajectory[point_num] = {
            'p' : path[point_num],
            }

#        

        if point_num == 0:
            trajectory[point_num]['t'] = 0.0
            #trajectory[point_num]['v'] = [0.0 for q in max_vels]
        else:
            ''' Time = Distance/Velocity '''

            trajectory[point_num]['t'] = last_time + max([(trajectory[point_num]['p'][i]-last_config[i])/path_vels[point_num][i] for i in range(len(path[0]))])

        last_time = trajectory[point_num]['t']
        last_config = path[point_num]

    if trajectory[len(trajectory)-1]['t'] != 0 :
        for point_num in range(len(trajectory)):
            trajectory[point_num]['t'] = trajectory[point_num]['t']*t_dur/trajectory[len(trajectory)-1]['t']
            #trajectory[point_num]['v'] = [(trajectory[point_num]['p'][i]-last_config[i])/trajectory[point_num]['t'] for i in range(len(max_vels))]
    #print(trajectory[point_num]['t'])
    return trajectory