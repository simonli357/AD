#!/usr/bin/env python3
# coding=utf-8

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from scipy.interpolate import interp1d
import cv2

class Draw_MPC_tracking(object):
    def __init__(self, u, robot_states: list, ref_states:list, init_state: np.array, export_fig=None
                 , xmin=-40, xmax=40, ymin=-40, ymax=40, waypoints_x = None, waypoints_y = None, 
                  stats = None, costs = None, times=None, objects=None, car_states=None):
        self.objects = objects
        self.car_states = car_states
        self.times = times if times is not None else np.arange(len(robot_states))
        self.u = u
        self.init_state = init_state
        self.robot_states = robot_states
        self.robot_states_np = np.array(robot_states)
        self.rob_radius = 0.3 /15*(xmax-xmin)
        self.waypoints_x = waypoints_x
        self.waypoints_y = waypoints_y
        self.stats = stats
        self.costs = costs
        # self.image = cv2.imread('/home/scandy/Documents/husky_ws/src/control/scripts/imgs/Competition_track_graph.png')
        # self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        # self.image = np.array(self.image).T
        # # resize the image to 500 by 500
        # self.image = cv2.resize(self.image, (100, 100), interpolation=cv2.INTER_AREA)

        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))

        # self.background_image = plt.imread('/home/scandy/Documents/husky_ws/src/control/scripts/imgs/Competition_track_graph.png') 

        # init for plot
        self.animation_init()

        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)
        self.ref_states = np.array(ref_states)  
        self.robot_states = np.array(robot_states)
        self.interpolate_trajectories()
        self.calculate_statistics()
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('MPC Tracking Performance')
        if export_fig is not None:
            self.plot_and_save(output_path=export_fig.replace('gifs', 'plots')+ '.png')
        # self.ax.legend(loc='center', frameon=True, bbox_to_anchor=(0.8, 0.1))
        plt.grid('--')
        if export_fig is not None:
            self.ani.save(export_fig+'.gif', writer='pillow', fps=50)
        plt.show()

    def draw_static_objects(self):
        for obj in self.objects:
            obj_pose = obj['pose']
            obj_type = obj['type']
            # Label text for each object
            self.ax.text(obj_pose[0] + 0.1, obj_pose[1] + 0.1, '{} ({:.2f}, {:.2f})'.format(obj_type, *obj_pose), 
                        color='blue', fontsize=12, bbox=dict(facecolor='white', alpha=0.5, edgecolor='none'))
            # Object position marker
            self.ax.scatter(obj_pose[0], obj_pose[1], c='blue', s=100)  
            
        for car in self.car_states:
            self.ax.scatter(car[0], car[1], c='red', s=100, marker='s')  # Square for car
            self.ax.text(car[0] + 0.1, car[1] + 0.1, '{} ({:.2f}, {:.2f}, {:.2f})'.format("car", *car), 
                        color='red', fontsize=12, bbox=dict(facecolor='white', alpha=0.5, edgecolor='none'))

    def animation_init(self, ):
        self.recent_positions = []
        self.max_recent_positions = 5
        cos_o, sin_o = np.cos(self.init_state[2]), np.sin(self.init_state[2])
        corner_x = self.init_state[0] - 0.25 * cos_o + 0.15 * sin_o
        corner_y = self.init_state[1] - 0.25 * sin_o - 0.15 * cos_o
        self.residual_rectangles = [
            mpatches.Rectangle(
                # (self.init_state[0] - 0.25, self.init_state[1] - 0.15), 
                (corner_x, corner_y), 
                0.5, 0.3, 
                color='blue', 
                fill=False, 
                angle=np.degrees(self.init_state[2])
            ) for _ in range(self.max_recent_positions)
        ]
        for rect in self.residual_rectangles:
            self.ax.add_patch(rect)

        self.init_robot_position = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        if self.objects is not None:
            self.draw_static_objects()
        self.ax.add_artist(self.init_robot_position)
        self.robot_body = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.robot_body) 
        self.robot_arr = mpatches.Arrow(self.init_state[0], self.init_state[1],
                                        self.rob_radius * np.cos(self.init_state[2]),
                                        self.rob_radius * np.sin(self.init_state[2]), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        return
    
    def animation_loop(self, indx):
        # position = self.robot_states[indx][:2]
        orientation = self.robot_states[indx][2]

        self.recent_positions.append(self.robot_states[indx])
        if len(self.recent_positions) > self.max_recent_positions:
            self.recent_positions.pop(0)
        for i, pos in enumerate(self.recent_positions):
            cos_o, sin_o = np.cos(pos[2]), np.sin(pos[2])
            corner_x = pos[0] - 0.25 * cos_o + 0.15 * sin_o
            corner_y = pos[1] - 0.25 * sin_o - 0.15 * cos_o
            self.residual_rectangles[i].set_xy((corner_x, corner_y))
            self.residual_rectangles[i].angle = np.degrees(pos[2])

        self.robot_body.center = self.robot_states[indx][:2]
        self.robot_arr.remove()
        self.robot_arr = mpatches.Arrow(self.robot_states[indx][0], self.robot_states[indx][1], self.rob_radius * np.cos(orientation),
                                        self.rob_radius * np.sin(orientation), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        # self.ax.set_title('MPC Tracking Performance: ' + str(indx)+ ' v:' + str(round(self.u[indx][0], 2)) + ' steer:' + str(round(self.u[indx][1], 2)))
        self.ax.set_title(str(indx)+') t:'+ str(round(self.times[indx],2)) + ' v:' + str(round(self.u[indx][0], 2)) + ' steer:' + str(round(self.u[indx][1], 2)) + ' x:' + str(round(self.robot_states[indx,0], 2)) + ' y:' + str(round(self.robot_states[indx,1], 2)) + ' yaw:' + str(round(self.robot_states[indx,2], 2)))
        return self.robot_arr, self.robot_body
    
    def interpolate_trajectories(self):        
        print("ref state shape1: ", self.ref_states.shape)
        print("robot state shape: ", self.robot_states.shape)
        if len(self.ref_states) == 0:
            print("No reference states provided. Using robot states as reference.")
            self.ref_states = self.robot_states
        if len(self.ref_states) < len(self.robot_states):
            print("Robot states are longer than reference states. Using reference states as reference.")
            # extend ref state with extra values in robot state
            ref_states = np.concatenate((self.ref_states, np.tile(self.ref_states[-1], (len(self.robot_states) - len(self.ref_states), 1))))
        else:
            print("Reference states are longer than robot states. Using robot states as reference.")
            ref_states = self.ref_states

        t = np.linspace(0, 1, len(ref_states))  
        print("ref state shape2: ", ref_states.shape)
        self.ref_traj_interp_x = interp1d(t, ref_states[:, 0], kind='cubic')
        self.ref_traj_interp_y = interp1d(t, ref_states[:, 1], kind='cubic')
        
        self.true_traj_interp_x = interp1d(t, self.robot_states[:, 0], kind='cubic')
        self.true_traj_interp_y = interp1d(t, self.robot_states[:, 1], kind='cubic')

    def calculate_statistics(self):
        if len(self.ref_states) < len(self.robot_states):
            robot_states = self.robot_states[:len(self.ref_states)]
        else:
            robot_states = self.robot_states
        self.mean_error_x = np.mean(np.abs(robot_states[:, 0] - self.ref_states[:, 0]))
        self.mean_error_y = np.mean(np.abs(robot_states[:, 1] - self.ref_states[:, 1]))
        self.max_error_x = np.max(np.abs(robot_states[:, 0] - self.ref_states[:, 0]))
        self.max_error_y = np.max(np.abs(robot_states[:, 1] - self.ref_states[:, 1]))
        self.min_error_x = np.min(np.abs(robot_states[:, 0] - self.ref_states[:, 0]))
        self.min_error_y = np.min(np.abs(robot_states[:, 1] - self.ref_states[:, 1]))

    def plot_and_save(self, output_path='output.png'):
        # Plot the interpolated trajectories
        t = np.linspace(0, 1, 1000)  # 1000 points for a smooth curve
        ref_x = self.ref_traj_interp_x(t)
        ref_y = self.ref_traj_interp_y(t)
        true_x = self.true_traj_interp_x(t)
        true_y = self.true_traj_interp_y(t)
        
        # Plot the trajectories
        self.ax.plot(ref_x, ref_y, label='Reference trajectory', linewidth=1.25, color='purple')
        self.ax.plot(true_x, true_y, label='Robot trajectory', linewidth=1.25, color='orange')
        
        self.ax.legend(loc='center', frameon=True, bbox_to_anchor=(0.83, 0.04))
        
        if self.stats is not None:
            stats_text = f"""Statistics\nAverage Speed: {self.stats[0]:.3f}\nAverage Steer: {self.stats[1]:.3f}\nAverage Delta Speed: {self.stats[2]:.3f}\nAverage Delta Steer: {self.stats[3]:.3f}\nAverage X Error: {self.stats[4]:.3f}\nAverage Y Error: {self.stats[5]:.3f}\nAverage Yaw Error: {self.stats[6]:.3f}"""
            costs_text = f"""Cost Values\nXY Cost: {self.costs[0]:.2f}\nYaw Cost: {self.costs[1]:.2f}\nV Cost: {self.costs[2]:.2f}\nSteer Cost: {self.costs[3]:.2f}\nDelta V Cost: {self.costs[4]:.2f}\nDelta Steer Cost: {self.costs[5]:.2f}"""
            self.ax.text(0.123, 0.685, stats_text, transform=self.ax.transAxes, verticalalignment='center', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            self.ax.text(0.123, 0.325, costs_text, transform=self.ax.transAxes, verticalalignment='center', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        plt.savefig(output_path)
    
if __name__ == '__main__':
    import os
    current_path = os.path.dirname(os.path.realpath(__file__))
    wpts_x = np.loadtxt(os.path.join(current_path, 'paths', 'waypoints_x.txt'))
    wpts_y = np.loadtxt(os.path.join(current_path, 'paths', 'waypoints_y.txt'))
    # ref_states = np.loadtxt(os.path.join(current_path, 'paths', 'state_refs2.txt'))
    current_path = current_path.replace('scripts', 'src')
    robot_states = np.loadtxt(os.path.join(current_path, 'simX.txt'))
    ref_states = np.loadtxt(os.path.join(current_path, 'state_refs.txt'))
    if len(ref_states) > len(robot_states):
        wpts_x = wpts_x[:len(robot_states)]
        wpts_y = wpts_y[:len(robot_states)]
        ref_states = ref_states[:len(robot_states)]
    u = np.loadtxt(os.path.join(current_path, 'simU.txt'))
    print(robot_states.shape, u.shape)
    # Draw_MPC_tracking(u, robot_states, ref_states, robot_states[0], waypoints_x=wpts_x, waypoints_y=wpts_y, export_fig='test')
    Draw_MPC_tracking(u, robot_states, ref_states, robot_states[0], xmax=21, export_fig='test')
    #park
    # Draw_MPC_tracking(u, robot_states, ref_states, robot_states[0], waypoints_x=wpts_x, waypoints_y=wpts_y, export_fig='test', xmin=-2, xmax=2, ymin=-1, ymax=1)

    