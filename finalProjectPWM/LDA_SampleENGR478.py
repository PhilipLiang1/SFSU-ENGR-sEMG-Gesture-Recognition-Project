import pandas as pd
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
from sklearn import metrics
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import style
import pickle
import serial 
##Start reading data from UART here in Python
'''
serialString = ""                           # Used to hold data coming over UART

port = "com5"
baud = 115200

#configure serial port
ser = serial(port, baud,timeout=1)

if ser.isOpen():
    print(ser.name + "is open...")

while True:
    command = input("Enter command or 'exit': ")
    if command == 'exit':
        ser.close()
        exit()
    else:
        ser.write(command.encode('ascii'))
        out = ser.read()
        print('Receiving...'+ str(out))
        



while(1):

    # Wait until there is data waiting in the serial buffer
    if(serialPort.in_waiting > 0):

        # Read data out of the buffer until a carraige return / new line is found
        serialString = serialPort.readline()

        # Print the contents of the serial data
        print(serialString.decode('Ascii'))

        # Tell the device connected over the serial port that we recevied the data!
        # The b at the beginning is used to indicate bytes!
        serialPort.write(b"Thank you for sending data \r\n")
##End of reading data from UART



FEATURE_NAMES = ['MAV0', 'ZC0', 'WL0', 'T0','MAV1', 'ZC1', 'WL1', 'T1',
                 'MAV2','ZC2', 'WL2', 'T2', 'MAV3', 'ZC3', 'WL3', 'T3', 'Gestures']

GESTURES = ['Rest', 'HandClose', 'HandOpen', 'PointIndex', 'DevilHorns']

#EMG_data = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/archive/houses_to_rent.csv")
rest_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/Rest.csv")
rest1_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/Rest1.csv")
handclose_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/HandClose.csv")
handclose1_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/HandClose1.csv")
handopen_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/HandOpen.csv")
handopen1_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/HandOpen1.csv")
pointindex_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/PointIndex.csv")
pointindex1_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/PointIndex1.csv")
devilhorns_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/DevilHorns.csv")
devilhorns1_feat_df = pd.read_csv("~/Desktop/MachineLearning/FinalProject478/DevilHorns1.csv")
#print(EMG_data)

# will use this to create a feature matrix that stacks all the gesture files together
feat_matrix = np.vstack([rest_feat_df.to_numpy(), rest1_feat_df.to_numpy(), 
                handclose_feat_df.to_numpy(), handclose1_feat_df.to_numpy(),
                handopen_feat_df.to_numpy(), handopen1_feat_df.to_numpy(), 
                pointindex_feat_df.to_numpy(), pointindex1_feat_df.to_numpy(),
                devilhorns_feat_df.to_numpy(), devilhorns1_feat_df.to_numpy()])

feat_matrix_df = pd.DataFrame(feat_matrix)
feat_matrix_df.columns = FEATURE_NAMES

X = feat_matrix_df.iloc[:,:16].copy()
# will use this to get a vector of only the labels (gestures) for comparison with predicted gestures
y = feat_matrix_df['Gestures'].copy()

#predictor variable label as x: all predictor variable set in one data frame
#X = EMG_data.iloc[:, 1:6097]
#x = EMG_data[:, 1:6097]


#all the outcome variable also set in on dataframe
#just one single column labeled as "total" in the data set to y
#y = EMG_data.iloc[:, "total"]
#y = EMG_data['total']

#use sklearn to split data

#train_test_split(x, y, test_size=0.2)
#plit the data into four categories
#x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2)
#keep track of best model
best_model = 0

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)

#save the best model if current model has better performance
for _ in range(10): 
    #add optional parameter to set a seed
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)

    #create instance of classifier
    model = LDA()
    model.fit(X_train, y_train)
    #show performance: shows accuracy of our model
    accuracy = model.score(X_train, y_train)
    #y predicted
    y_pred = model.predict(X_test)
    x_pred = model.predict(y_test)
    #print(f"current accuracy is: {accuracy}")
    #compare the accuracy and save the best accuracy model
    if accuracy > best_model:
        best_model = accuracy

        #we begin to load our model here now, model is saved in ~/Dessktop/MachineLearning/FinalProject478
        with open("EMG_sensorModel.pickle", "wb") as EMG:
            pickle.dump(model, EMG)

X_pred = model.transform(X_test)
y_pred = model.transform(y_test)
plt.scatter(X_pred, y_pred, c=y, edgecolor='none', alpha=0.8,cmap=plt.cm.get_cmap('viridis',3))

plt.xlabel('x_predicted')
plt.ylabel('y_predicted')

plt.colorbar()
plt.show()

#here, we can open the model again
pickle_in = open("EMG_sensorModel.pickle", "rb")
model = pickle.load(pickle_in)



#p = FEATURE_NAMES[0]
#style.use("ggplot")

testSet = pd.read_csv('test_data.csv')

model.fit(testSet)
accuracy = model.score(testSet)

print(accuracy)

y_predicted = model.predict(testSet)

print(y_predicted)



testSet.columns = FEATURE_NAMES

x_data = testSet.iloc[:, :16].copy()

x_data = x_data.to_numpy()

y_data = testSet['Gestures'].copy()
y_data = y_data.to_numpy()

pred = model.predict(y_data)


print(pred)

print(classification_report(y_test, y_pred))
#accuracy = model.score(X_train, y_train)
print(f"Accuracy: {accuracy*100:.2f}")


#model.save('FinalProjENGR478.model')
#plt.scatter(FEATURE_NAMES[1], FEATURE_NAMES[14])
#plt.xlabel = ('MAV0')
#plt.ylabel = ('Final GESTURE')
#plt.show()
#print(acc)
#We want to see how well our medl performed
#print(classification_report(y_test, y_pred))
#print(model)

#Lets make a visualization of our prediction now
#the following code is only for visualization, now really required for the
#purpose of our model
#fpr = false positive rate, tpr = truth positive rate, all base on the test data
#auc = area under curve
#fpr, tpr, thresholds = metrics.roc_curve(y_test, y_pred)
#roc_auc = metrics.auc(fpr, tpr)
#roc_auc
#plt.model()
#show auc on the curve with set precision
#plt.plot(fpr, tpr, label="ROC curve (are=%0.2f)" % roc_auc)
#create reference point: dotted line
#plt.plot([0,1],[0,1],'k--')
#plt.legend(loc="lower right")
'''
