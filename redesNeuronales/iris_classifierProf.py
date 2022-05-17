#------------------------------------------------------------------------------------------------------------------
#   Iris data set classifier.
#------------------------------------------------------------------------------------------------------------------

import numpy as np

from sklearn import datasets
from sklearn import svm
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import confusion_matrix
from sklearn.model_selection import cross_validate

import matplotlib.pyplot as plt
import matplotlib.cm as cm

# Import IRIS data set
iris = datasets.load_iris()


x = iris.data
y = iris.target
targets = iris.target_names
n_targets = len(targets)
features = iris.feature_names
n_features = len(features)

# Plot pairs of variables
cmap = cm.get_cmap('viridis')
fig, axs = plt.subplots(n_features, n_features)

for i in range(n_features):
    for j in range(n_features):
        if i != j:
            for k in range(n_targets):
                axs[i, j].scatter(x[y==k,j], x[y==k,i], label = targets[k], color = cmap(k/(n_targets-1)), 
                                  alpha=0.8, edgecolor='k')
        else:
            axs[i, j].text(0.5, 0.5, features[i], horizontalalignment='center', 
                           verticalalignment='center', style='italic', fontsize=14)
            axs[i, j].xaxis.set_visible(False)
            axs[i, j].yaxis.set_visible(False)
           

axs[n_features//2 - 1, n_features-1].legend(bbox_to_anchor=(1.5, 0.1))
plt.subplots_adjust(right=0.90)
plt.show()

# Train SVM classifier with all the available observations
clf = svm.SVC(kernel = 'linear')
clf.fit(x, y)

# Predict one new sample
print("New evaluations", clf.predict( [[4.4, 2.9, 1.4, 0.2], [0.4,0.1,0.1,0.4], [7.7, 3.0, 6.1, 2.2]]))

# 5-fold cross-validation
kf = StratifiedKFold(n_splits=10, shuffle = True)
clf = svm.SVC(kernel = 'linear')

acc = 0
recall = np.array([0., 0., 0.])
precision = np.array([0., 0., 0.])

for train_index, test_index in kf.split(x, y):
    
    # Training phase
    x_train = x[train_index, :]
    y_train = y[train_index]
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

# Print results
acc = acc/5
print('Acc: ', acc)

precision = precision/5
print('Precision: ', precision)

recall = recall/5
print('Recall: ', recall)

# 5-fold cross-validation using cross_validate
cv_results = cross_validate(clf, x, y, cv=5, scoring = ('accuracy', 'recall_micro'))
print('Acc: ', cv_results['test_accuracy'].sum()/5)
print('Recall: ', cv_results['test_recall_micro'].sum()/5)


#------------------------------------------------------------------------------------------------------------------
#   End of file
#------------------------------------------------------------------------------------------------------------------