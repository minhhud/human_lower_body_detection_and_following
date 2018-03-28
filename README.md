# human_low_body_detection_and_following
We propose a robust method for human leg detection and tracking by using low-cost RGB-D camera. Two issuses are mainly takled:
+ Exploit three-dimensional characteristics and RGB features of low-body part to overcome the malfunction in distinguishing human legs and similar types of shape (table legs, pillars, etc.) 
+ Reliable mechanism to track the target leg while multi-legs are detected simultaneously in complex environment.

A. Leg Detector: 
Flow-chart of leg detector (Figure 1) describes 4 modules:
+ Retrieving leg model (i)
+ Pairing of legs (ii)
+ Masking with low-body part (iii)
+ Applying Support Vector Machine (SVM) classifier with Histogram Oriented Gradient (iv)

B. Leg follow:
+ After detection, the low-body part is tracked using a Kalman filter.
+ A combination of similarity of distance between the center point of detected lower body part and Kinect (sim_dist), and the Color Histogram of leg region (sim_color) are calculated and added up together. 
