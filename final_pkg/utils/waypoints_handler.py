import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# Load existing waypoints from CSV
csv_file_path = './origin.csv'
waypoints_df = pd.read_csv(csv_file_path, usecols=[0, 1])
waypoints = np.array(waypoints_df)
waypoints_new = waypoints[:1]

fig, ax = plt.subplots(figsize=(10, 5))
plt.subplots_adjust(bottom=0.2)  # Make room for the 'Done' button

# Plot existing waypoints
ax.set_aspect('equal', 'box')
ax.plot(waypoints[:, 0], waypoints[:, 1], marker='o')
ax.set_title('Click to Add Waypoints')
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.grid(True)

# Event handler to capture clicks and add waypoints
def onclick(event):
    global waypoints
    global waypoints_new
    if event.inaxes == ax:  # Only add points within the axes
        ix, iy = event.xdata, event.ydata
        print(f"New waypoint: ({ix}, {iy})")

        # Append the new waypoint to the array
        waypoints_new = np.vstack((waypoints_new, [ix, iy]))

        # Update the plot with the new waypoint
        ax.plot(ix, iy, marker='o', color='red')
        fig.canvas.draw()

# Button click event handler
def finish(event):
    global waypoints
    global waypoints_new
    plt.close(fig)  # Close the plot window

    # Save the new set of waypoints to a new CSV file
    new_csv_file_path = './sampled.csv'
    np.savetxt(new_csv_file_path, waypoints_new[1:], delimiter=',', header='x,y', comments='')
    print(f"Waypoints saved to {new_csv_file_path}")

# Creating the 'Done' button
ax_done = plt.axes([0.7, 0.05, 0.1, 0.075])  # Adjust the rectangle where the button is placed
btn_done = Button(ax_done, 'Done')
btn_done.on_clicked(finish)  # Register the event handler

# Connect the click event to the handler function
fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()