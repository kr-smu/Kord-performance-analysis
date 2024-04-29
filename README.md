# Kord-performance-analysis
This repository contains tools for analyzing the performance of the Kord API, specifically focusing on the timing of various functions and their impact on real-time control loops.

**Files:**

**ex_move_linear:** Example code for linear movement with timing measurements.

**ex_move_joints:** Example code for joint movement with timing measurements.

**diff_api.py:** Python script to visualize timing differences using matplotlib.

**ex_move_direct:** Example code for direct joint control movement with timing measurements.

**ex_joints_changes:** Patch file demonstrating modifications made to the example code for timing analysis.

**Analysis Process:**

**Modifications to Example Code:** The example code files (ex_move_linear.txt and ex_move_joints.txt) are modified to include:

- Pre-allocation of arrays for efficiency.
- Recording of raw timestamps before and after key operations (waitSync, moveJ/moveL, fetchData).

**Data Collection:** Run the modified example code to collect timing data for each iteration of the control loop. This data is saved to a CSV file (stats_file_kord_api.csv).

**Visualization:** Use the diff_api.py script to plot the timing differences between the recorded timestamps. This helps visualize the time taken by each operation and identify potential bottlenecks or inconsistencies.

**Analysis:** Analyze the timing data and visualizations to understand the performance characteristics of the Kord API and its impact on real-time control. This information can be used to optimize control loops and improve the overall performance of robot applications.

**Additional Notes:**

- The ex_joints_changes file shows the specific modifications made to the example code for timing analysis.
- The ex_move_direct file provides an example for analyzing the timing of direct joint control movement.
- The analysis process can be applied to other Kord API functions and examples to gain a comprehensive understanding of performance.

![Zoomed out plot](image-1.png)
![Zoomed in plot ](image.png)
