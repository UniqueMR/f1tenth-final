import pandas as pd
import numpy as np
from scipy.interpolate import CubicSpline

# create dense points
class waypoints():
    def __init__(self, raw_data_path, output_path, sampling_rate = 30, num_points=100):
        self.raw_data_path = raw_data_path
        self.sampling_rate = sampling_rate
        self.output_path = output_path
        self.num_points = num_points

        self.df = pd.read_csv(self.raw_data_path, sep=',', usecols=[0,1])
    
    def create_dense_pts(self):
        # sampling way points
        interval = self.sampling_rate
        sampled_waypoints_list = []
        for idx, waypoint in self.df.iterrows():
            if idx % int(interval) == 0:
                sampled_waypoints_list.append(waypoint.values)

        sampled_waypoints = np.array(sampled_waypoints_list)

        # create interpolation    
        # x_coords = sampled_waypoints[2:-2, 0]
        # y_coords = sampled_waypoints[2:-2, 1]

        x_coords = sampled_waypoints[:, 0]
        y_coords = sampled_waypoints[:, 1] 

        x_coords[-1] = x_coords[0]
        y_coords[-1] = y_coords[0]

        t = np.arange(len(x_coords))

        # Create periodic boundary conditions to handle the loop
        cs_x = CubicSpline(t, x_coords, bc_type='periodic')
        cs_y = CubicSpline(t, y_coords, bc_type='periodic')

        # Interpolate at a high resolution to create the dense trajectory
        dense_t = np.linspace(t.min(), t.max(), num=self.num_points)
        self.dense_x = cs_x(dense_t)
        self.dense_y = cs_y(dense_t)

        return self.dense_x, self.dense_y

    def create_csv_file(self):
        # Calculate the difference in coordinates between the current point and the next point
        dx = np.round(np.diff(self.dense_x), decimals=4)
        dy = np.round(np.diff(self.dense_y), decimals=4)

        yaw = np.arctan2(dy, dx)

        if (np.any(np.abs(dy/dx) == np.inf)):
            inf_idxs = np.where(dy/dx == np.inf)[0]
            mod_idxs = inf_idxs-1
            yaw[inf_idxs] = yaw[mod_idxs]

        # print(yaw)
        # new dataframe to store the values
        df_out = pd.DataFrame()
        df_out[0] = self.dense_x[:-1]
        df_out[1] = self.dense_y[:-1]
        df_out[2] = 2.5 # speed
        df_out[3] = yaw

        df_out.to_csv(self.output_path, index=False, header=False)
    
    def main(self):
        self.create_dense_pts()
        self.create_csv_file()

if __name__ == '__main__':
    waypoints = waypoints('sampled.csv', 'interpolated.csv', sampling_rate=1, num_points=500)
    waypoints.main()




