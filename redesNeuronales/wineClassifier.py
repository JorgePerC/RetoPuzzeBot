import numpy as np

from sklearn import datasets
from sklearn import svm
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import confusion_matrix
from sklearn.model_selection import cross_validate

import matplotlib.pyplot as plt
import matplotlib.cm as cm

wineDataset = datasets.load_wine()

featureValues = wineDataset.data
labels = wineDataset.target

print("Features: ", len(wineDataset.feature_names))
print(wineDataset.feature_names)

print("===================")
print("Features: ", len(wineDataset.target_names))
print(wineDataset.target_names)

print("===================")

# 5-fold cross-validation
kf = StratifiedKFold(n_splits=5, shuffle = True)
clf = svm.SVC(kernel = 'linear')

# Train SVM classifier with all the available observations
clf = svm.SVC(kernel = 'linear')
clf.fit(featureValues, labels)

dataAcc = 0
recall = np.array([0., 0., 0.])
precision = np.array([0., 0., 0.])

for train_index, test_index in kf.split(featureValues, labels):
    # Training phase
    x_train = featureValues[train_index, :]
    y_train = labels[train_index]
    clf.fit(x_train, y_train)

    # Test phase
    x_test = x[test_index, :]
    y_test = y[test_index]    
    y_pred = clf.predict(x_test)

    # Calculate confusion matrix and model performance
    cm = confusion_matrix(y_test, y_pred)
    print('Confusion matrix\n', cm)
    
    acc += (cm[0,0]+cm[1,1]+cm[2,2])/len(y_test)    

    recall[0] += cm[0,0]/(cm[0,0] + cm[0,1] + cm[0,2])
    recall[1] += cm[1,1]/(cm[1,0] + cm[1,1] + cm[1,2])
    recall[2] += cm[2,2]/(cm[2,0] + cm[2,1] + cm[2,2])

    precision[0] += cm[0,0]/(cm[0,0] + cm[1,0] + cm[2,0])
    precision[1] += cm[1,1]/(cm[0,1] + cm[1,1] + cm[2,1])
    precision[2] += cm[2,2]/(cm[0,2] + cm[1,2] + cm[2,2])