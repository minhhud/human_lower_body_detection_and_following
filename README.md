# human_low_body_detection_and_following
We propose a robust method for human leg detection, tracking and following by using low-cost RGB-D camera. Two issuses are mainly takled:
+ Exploit three-dimensional characteristics and RGB features of low-body part to overcome the malfunction in distinguishing human legs and similar types of shape (table legs, pillars, etc.) 
+ Reliable mechanism to track the target leg while multi-legs are detected simultaneously in complex environment.

A. Leg Detector: 
Leg detector includes 4 modules:
+ Retrieving leg model 
+ Pairing of legs 
+ Masking with low-body part
+ Applying Support Vector Machine (SVM) classifier with Histogram Oriented Gradient 

B. Leg follow:
+ After detection, the low-body part is tracked using a Kalman filter.

C. Database: 
+ Our own training data is quite heavily so we are not upload to github. 
