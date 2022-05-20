# SFSU-ENGR-sEMG-Gesture-Recognition-Project
Gesture Recognition with sEMG Engineering Project by Philip Liang and Benediction Bora.
This Gesture Recognition with sEMG Project uses the SFSU ENGR478 sample ADC project developed by Dr. Xiaorong Zhang as the base outline and setup. 
It was modified and expanded upon for the development of our project.
Our project collects training and testing data from 4 MyoWare sEMG sensors placed radially around the forearm with the reference node at the elbow joint.
The raw sEMG data is captured by the ADC of a Tiva TM4C123G Microcontroller to perform feature extraction.
The sampling rate of our ADC is 1000 Hz. Every 50ms, our program extracts 4 time-domain features (MAV, ZC, WL, T) from each channel of the recorded raw sEMG.
The extracted features are then outputted via UART to a serial terminal for viewing and recording at approximately 320 Hz.
We collected 2 sets of 5-second training data for each of following the 5 gestures: Rest (0), Hand Closed (1), Hand Open (1), Point Index (3), and Devil Horns (4).
Between each recording session, the MyoWare sensors were removed and replaced to introduce sensor shifts in our training data to improve robustness.
The features for each gesture were recorded and saved in separate csv files for uploading to our LDA model that was built in Python.
Our Python program combines the uploaded training data into 1 feature matrix and feeds it into the SciKit Learn Linear Discriminant Analysis model.
Our LDA model is trained and tested with 100-fold cross-validation without shuffling, achieving an accuracy of 85%.
We also used testing feature data from shifted sensors to make predictions with our model and test its robustness, achieving a comparable 82% accuracy.
Finally, our model's predicted gesture output can be used to control a robotic arm to form the predicted gesture
A 2nd Tiva will read the prediction via UART and output the corresponding control signals to the 5V servos of the robotic hand to form the predicted gesture.
